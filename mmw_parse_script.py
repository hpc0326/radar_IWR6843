# ****************************************************************************
# * (C) Copyright 2020, Texas Instruments Incorporated. - www.ti.com
# ****************************************************************************
# *
# *  Redistribution and use in source and binary forms, with or without
# *  modification, are permitted provided that the following conditions are
# *  met:
# *
# *    Redistributions of source code must retain the above copyright notice,
# *    this list of conditions and the following disclaimer.
# *
# *    Redistributions in binary form must reproduce the above copyright
# *    notice, this list of conditions and the following disclaimer in the
# *     documentation and/or other materials provided with the distribution.
# *
# *    Neither the name of Texas Instruments Incorporated nor the names of its
# *    contributors may be used to endorse or promote products derived from
# *    this software without specific prior written permission.
# *
# *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# *  PARTICULAR TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# *  A PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  OWNER OR
# *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# *  EXEMPLARY, ORCONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# *  LIABILITY, WHETHER IN CONTRACT,  STRICT LIABILITY, OR TORT (INCLUDING
# *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# *
# ****************************************************************************


# ****************************************************************************
# Sample mmW demo UART output parser script - should be invoked using python3
#       ex: python3 mmw_demo_example_script.py <recorded_dat_file_from_Visualizer>.dat
#
# Notes:
#   1. The parser_mmw_demo script will output the text version 
#      of the captured files on stdio. User can redirect that output to a log file, if desired
#   2. This example script also outputs the detected point cloud data in mmw_demo_output.csv 
#      to showcase how to use the output of parser_one_mmw_demo_output_packet
# ****************************************************************************
import sys
import serial
import time
import numpy as np
from threading import Thread
import os
import sys
import json
import keyboard
import csv

# import the parser function 
from parser_mmw_demo import parser_one_mmw_demo_output_packet

#for GUI module
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import numpy as np


f = open('configuration.json')
data = json.load(f)
    
# Change the configuration file name
configFileName = data['configFileName']
cliPort = data['cliPort']
dataPort = data['dataPort']

# Change the debug variable to use print()
DEBUG = False
DETECTION = 0

# Constants
maxBufferSize = 2**15
CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2**15,dtype = 'uint8')
byteBufferLength = 0
maxBufferSize = 2**15
magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
detObj = {}  
frameData = {}    
currentIndex = 0
pos1 = np.empty((50,3))
# word array to convert 4 bytes to a 32 bit number
word = [1, 2**8, 2**16, 2**24]


class CustomTextItem(gl.GLGraphicsItem.GLGraphicsItem):
	def __init__(self, X, Y, Z, text):
		gl.GLGraphicsItem.GLGraphicsItem.__init__(self)
		self.text = text
		self.X = X
		self.Y = Y
		self.Z = Z

	def setGLViewWidget(self, GLViewWidget):
		self.GLViewWidget = GLViewWidget

	def setText(self, text):
		self.text = text
		self.update()

	def setX(self, X):
		self.X = X
		self.update()

	def setY(self, Y):
		self.Y = Y
		self.update()

	def setZ(self, Z):
		self.Z = Z
		self.update()

	def paint(self):
		a = 0
		'''
		# For some version gl is ok some get following error
		#Error while drawing item <__main__.CustomTextItem object at 0x7fe379b51a60>
		
		self.GLViewWidget.qglColor(QtCore.Qt.cyan)
		self.GLViewWidget.renderText(round(self.X), round(self.Y), round(self.Z), self.text)
		'''
  
class Custom3DAxis(gl.GLAxisItem):
	#Class defined to extend 'gl.GLAxisItem'
	def __init__(self, parent, color=(0.0,0.0,0.0,.6)):
		gl.GLAxisItem.__init__(self)
		self.parent = parent
		self.c = color
		
	def add_tick_values(self, xticks=[], yticks=[], zticks=[]):
		#Adds ticks values. 
		x,y,z = self.size()
		xtpos = np.linspace(0, x, len(xticks))
		ytpos = np.linspace(0, y, len(yticks))
		ztpos = np.linspace(0, z, len(zticks))
		#X label
		for i, xt in enumerate(xticks):
			val = CustomTextItem((xtpos[i]), Y= 0, Z= 0, text='{}'.format(xt))
			val.setGLViewWidget(self.parent)
			self.parent.addItem(val)
		#Y label
		for i, yt in enumerate(yticks):
			val = CustomTextItem(X=0, Y=round(ytpos[i]), Z= 0, text='{}'.format(yt))
			val.setGLViewWidget(self.parent)
			self.parent.addItem(val)
		#Z label
		for i, zt in enumerate(zticks):
			val = CustomTextItem(X=0, Y=0, Z=round(ztpos[i]), text='{}'.format(zt))
			val.setGLViewWidget(self.parent)
			self.parent.addItem(val)

app = QtWidgets.QApplication([])
w = gl.GLViewWidget()
w.show()

JB_RADAR_INSTALL_HEIGHT = 2.46
####### create box to represent device ######
verX = 0.0625
verY = 0.05
verZ = 0.125
zOffSet =  JB_RADAR_INSTALL_HEIGHT
verts = np.empty((2,3,3))
verts[0,0,:] = [-verX, 0,  verZ + zOffSet]
verts[0,1,:] = [-verX, 0, -verZ + zOffSet]
verts[0,2,:] = [verX,  0, -verZ + zOffSet]
verts[1,0,:] = [-verX, 0,  verZ + zOffSet]
verts[1,1,:] = [verX,  0,  verZ + zOffSet]
verts[1,2,:] = [verX,  0, -verZ + zOffSet]

#sensor position red mesh
evmBox = gl.GLMeshItem(vertexes=verts,smooth=False,drawEdges=True,edgeColor=pg.glColor('r'),drawFaces=False)
w.addItem(evmBox)
#############################################

#base grid
g = gl.GLGridItem()
g.setSize(x=50,y=50,z=50)
w.addItem(g)

#x, y, z axis
axis = Custom3DAxis(w, color=(0.2,0.2,0.2,1.0))
axis.setSize(x=5, y=5, z=5)
xt = [0,1,2,3,4,5]  
axis.add_tick_values(xticks=xt, yticks=xt, zticks=xt)
w.addItem(axis)

#red dot
pos = np.zeros((100,3))
color = [1.0, 0.0, 0.0, 1.0]
sp1 = gl.GLScatterPlotItem(pos=pos,color=color,size = 4.0)
w.addItem(sp1)

def update_point():
    ## update volume colors
    global color,pos1
    color = np.empty((len(pos1),4), dtype=np.float32)
    color[:,3] = 1.0
    color[:,0] =  np.clip(pos1[:,0] * 3.0, 0, 1)
    color[:,1] =  np.clip(pos1[:,1] * 1.0, 0,1)
    color[:,2] =  np.clip(pos1[:,2] ** 3, 0, 1)
    #print(len(pos1))
    sp1.setData(pos=pos1,color=color)
    
t = QtCore.QTimer()
t.timeout.connect(update_point)
t.start(50)

# Function to configure the serial ports and send the data from
# the configuration file to the radar
def serialConfig(configFileName):
    
    global CLIport
    global Dataport
    # Open the serial ports for the configuration and the data ports
    
    # Raspberry pi
    #CLIport = serial.Serial('/dev/ttyACM0', 115200)
    #Dataport = serial.Serial('/dev/ttyACM1', 921600)

    # Windows
    CLIport = serial.Serial(cliPort, 115200)
    Dataport = serial.Serial(dataPort, 921600)
    
    # Mac
    #CLIport = serial.Serial('/dev/tty.SLAB_USBtoUART', 115200)
    #Dataport = serial.Serial('/dev/tty.SLAB_USBtoUART10', 921600)

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        CLIport.write((i+'\n').encode())
        print(i)
        time.sleep(0.01)
        
    return CLIport, Dataport

# Function to parse the data inside the configuration file
def parseConfigFile(configFileName):
    configParameters = {} # Initialize an empty dictionary to store the configuration parameters
    
    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        
        # Split the line
        splitWords = i.split(" ")
        
        # Hard code the number of antennas, change if other configuration is used
        numRxAnt = 4
        numTxAnt = 3
        
        # Get the information about the profile configuration
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1
            
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2
                
            digOutSampleRate = int(splitWords[11])
            
        # Get the information about the frame configuration    
        elif "frameCfg" in splitWords[0]:
            
            chirpStartIdx = int(splitWords[1])
            chirpEndIdx = int(splitWords[2])
            numLoops = float(splitWords[3])
            numFrames = int(splitWords[4])
            framePeriodicity = float(splitWords[5])

            
    # Combine the read data to obtain the configuration parameters           
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate)/(2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)
    
    return configParameters

##################################################################################
# USE parser_mmw_demo SCRIPT TO PARSE ABOVE INPUT FILES
##################################################################################
def readAndParseData14xx(Dataport, configParameters):
    #load from serial
    global byteBuffer, byteBufferLength, pos1

    # Initialize variables
    magicOK = 0 # Checks if magic number has been read
    dataOK = 0 # Checks if the data has been read correctly
    frameNumber = 0
    detObj = {}

    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
    byteCount = len(byteVec)

    # Check that the buffer is not full, and then add the data to the buffer
    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
        byteBufferLength = byteBufferLength + byteCount
    
    # Check that the buffer has some data
    if byteBufferLength > 16:
        
        # Check for all possible locations of the magic word
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]

        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc:loc+8]
            if np.all(check == magicWord):
                startIdx.append(loc)

        # Check that startIdx is not empty
        if startIdx:
            
            # Remove the data before the first start index
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength-startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                byteBuffer[byteBufferLength-startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                byteBufferLength = byteBufferLength - startIdx[0]
                
            # Check that there have no errors with the byte buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0

            # Read the total packet length
            totalPacketLen = np.matmul(byteBuffer[12:12+4],word)
            # Check that all the packet has been read
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1
    
    # If magicOK is equal to 1 then process the message
    if magicOK:
        # Read the entire buffer
        readNumBytes = byteBufferLength
        if(DEBUG):
            print("readNumBytes: ", readNumBytes)
        allBinData = byteBuffer
        if(DEBUG):
            print("allBinData: ", allBinData[0], allBinData[1], allBinData[2], allBinData[3])

        # init local variables
        totalBytesParsed = 0
        numFramesParsed = 0

        # parser_one_mmw_demo_output_packet extracts only one complete frame at a time
        # so call this in a loop till end of file
        #             
        # parser_one_mmw_demo_output_packet function already prints the
        # parsed data to stdio. So showcasing only saving the data to arrays 
        # here for further custom processing
        parser_result, \
        headerStartIndex,  \
        totalPacketNumBytes, \
        numDetObj,  \
        numTlv,  \
        subFrameNumber,  \
        detectedX_array,  \
        detectedY_array,  \
        detectedZ_array,  \
        detectedV_array,  \
        detectedRange_array,  \
        detectedAzimuth_array,  \
        detectedElevation_array,  \
        detectedSNR_array,  \
        detectedNoise_array = parser_one_mmw_demo_output_packet(allBinData[totalBytesParsed::1], readNumBytes-totalBytesParsed,DEBUG)
        #print([len(detectedX_array),len(detectedY_array),len(detectedZ_array)])
        # Check the parser result
        if(DEBUG):
            print ("Parser result: ", parser_result)
        if (parser_result == 0): 
            totalBytesParsed += (headerStartIndex+totalPacketNumBytes)    
            numFramesParsed+=1
            if(DEBUG):
                print("totalBytesParsed: ", totalBytesParsed)
            ##################################################################################
            # TODO: use the arrays returned by above parser as needed. 
            # For array dimensions, see help(parser_one_mmw_demo_output_packet)
            # help(parser_one_mmw_demo_output_packet)
            ##################################################################################

            detObj = {"numObj": numDetObj, "range": detectedRange_array, \
                        "x": detectedX_array, "y": detectedY_array, "z": detectedZ_array
                    }
            
            detSideInfoObj = { "doppler" : detectedV_array, "snr" : detectedSNR_array,
                              "noise" : detectedNoise_array
                            }
            dataOK = 1 
            print(detObj)
            # keyboard_detect(detObj, detSideInfoObj)
            pos1X = np.empty((numDetObj,3))
            
            for i in range(numDetObj):
                zt = detectedZ_array[i] * 5 + 5
                xt = detectedX_array[i] * 5
                yt = detectedX_array[i] * 5
                pos1X[i] = (xt,yt,zt)
				
            pos1 = pos1X
            
        else: 
            # error in parsing; exit the loop
            print("error in parsing this frame; continue")

        
        shiftSize = totalPacketNumBytes            
        byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
        byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),dtype = 'uint8')
        byteBufferLength = byteBufferLength - shiftSize
        
        # Check that there are no errors with the buffer length
        if byteBufferLength < 0:
            byteBufferLength = 0
        # All processing done; Exit
        if(DEBUG):
            print("numFramesParsed: ", numFramesParsed)

    return dataOK, frameNumber, detObj


# Funtion to update the data and display in the plot
def update():
    
    dataOk = 0
    global detObj
    x = []
    y = []
    
    # Read and parse the received data
    dataOk, frameNumber, detObj = readAndParseData14xx(Dataport, configParameters)
    if dataOk and len(detObj["x"]) > 0:
        x = detObj["x"]
        y = detObj["y"]
        
    return dataOk, x, y

def keyboard_detect(detObj, detSideInfoObj):
    
    global DETECTION
    
    if 1:
        for index, obj in enumerate(range(detObj['numObj'])):
            """demoOutputWriter.writerow([index, obj, detObj['x'][obj],
                detObj['y'][obj], detObj['z'][obj], detSideInfoObj['doppler'][obj], 
                detSideInfoObj['snr'][obj], detSideInfoObj['noise'][obj]])"""
            
            demoOutputWriter.writerow([detSideInfoObj['doppler'][obj]])
        demoOutputWriter.writerow(["new frame"])   
        
        """if keyboard.read_key() == 'e':
            DETECTION = 0
            demoOutputWriter.writerow(["stop read"])  
            print("end read")"""
            
    else :     
        if keyboard.read_key() == 's':
            DETECTION = 1
            demoOutputWriter.writerow(["start read"])  
            print("start read")
            
            
def get3dBox(targetCloud): 
	xMax = np.max(targetCloud[:,0])
	xr   = np.min(targetCloud[:,0])
	xl = np.abs(xMax-xr)

	yMax = np.max(targetCloud[:,1])
	yr = np.min(targetCloud[:,1])
	yl = np.abs(yMax-yr)
	
	zMax = np.max(targetCloud[:,2])
	zr = np.min(targetCloud[:,2])
	zl = np.abs(zMax-zr)
	
	nop = len(targetCloud)
	return (xr,xl,yr,yl,zr,zl,nop)	
        

def main():        

    # Configurate the serial port and write the config doc.
    CLIport, Dataport = serialConfig(configFileName)

    # Get the configuration parameters from the configuration file
    global configParameters 
    configParameters = parseConfigFile(configFileName)
    

    while 1 :  
        update()
    
    
    CLIport.write(('sensorStop\n').encode())
    CLIport.close()
    Dataport.close()

def uartThread(name):
	# port.flushInput()
	while True:
		main()

thread1 = Thread(target = uartThread, args =("UART",))
thread1.setDaemon(True)
thread1.start()


if __name__ == "__main__":
    democsvfile = open('mmw_demo_output.csv', 'w', newline='')                
    demoOutputWriter = csv.writer(democsvfile, delimiter=',',
                            quotechar='', quoting=csv.QUOTE_NONE)                                    
    demoOutputWriter.writerow(["frame","DetObj#","x","y","z","v","snr","noise"])   
    if (sys.flags.interactive != 1) or not hasattr(QtCore,'PYQT_VERSION'):
        QtWidgets.QApplication.instance().exec()   
    # main()

