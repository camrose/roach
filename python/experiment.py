#!/usr/bin/env python
"""
authors: stanbaek, apullin

"""
from lib import command
import time,sys,os,msvcrt
import serial
import shared

from lib.lib.dictionaries import *
from lib.lib.command_interface import CommandInterface
from lib.lib.telemetry_reader import TelemetryReader
from lib.lib.quaternion import *
from lib.lib.keyboard_interface import KeyboardInterface
from lib.lib.increment_counter import IncrementCounter

from hall_helpers import *

hall_gains = [0, 0, 0, 0, 0]
duration = 0
cycle = 56 #ms for 18Hz wing stroke (possibly roll this in to the thrust settings
ang = [0, 0, 0, 0]
percent_int = [0, 0, 0, 0]
vel = [0, 0, 0, 0]
num_points = 4
thrust = 0


class HallController(object):
    def __init__(self, num_setpoints = 0):
        self.hall_gains = [0, 0, 0, 0, 0]
        self.duration = 0
        self.cycle = 56 #ms for 18Hz wing stroke (possibly roll this in to the thrust settings
        self.delta = [0] * num_setpoints
        self.interval = [0] * num_setpoints
        self.vel = [0] * num_setpoints
        self.counts = 45
        self.set = 0
        self.num_setpoints = num_setpoints
        self.thrust = IncrementCounter( start_value = 11.0, range = (18.0, 0.0), increment = 0.5 )
        print self.num_setpoints

    def setVelProfile(self, ang, percent_seg, thrust):
        if thrust == 0:
            self.cycle = 0
        else:
            self.cycle = int((1000/thrust))
        
        sum = 0
        self.delta[0] = (ang[0]*self.counts)/360
        sum = self.delta[0]
        for i in range(1,len(ang)-1):
            self.delta[i] = ((ang[i] - ang[i-1])*self.counts)/360
            sum = sum + self.delta[i]
        self.delta[len(ang)-1] = self.counts-sum

        sum = 0
        for i in range(0,len(percent_seg)):
            self.interval[i] = (percent_seg[i]*self.cycle)/100
            sum = sum + self.interval[i]
            self.vel[i] = (self.delta[i] << 8)/self.interval[i]
        self.interval[len(percent_seg)-1] = self.interval[len(percent_seg)-1] + self.cycle - sum

        self.checkParams()

    def setGains(self, gains):
        for i in range(0,len(gains)):
            self.hall_gains[i] = gains[i]

    def checkParams(self):
        if len(self.vel) == len(self.interval) == len(self.delta):
            self.set = 1
            
def txCallback(dest, packet):
    shared.xb.tx(dest_addr = dest, data = packet)
    
def rxCallback(packet):
    global telem
    telem.processPacket(packet)

def main():
    addr = '\x10\x21'
    
    ser = setupSerial()

    # Send robot a WHO_AM_I command, verify communications
    queryRobot()
    #Motor gains format:
    #  [ Kp , Ki , Kd , Kaw , Kff     ,  Kp , Ki , Kd , Kaw , Kff ]
    #    ----------LEFT----------        ---------_RIGHT----------
    motorgains = [1800,200,100,0,0, 1800,200,100,0,0]
    duration = 500
    rightFreq = 0
    leftFreq = 0
    phase = 0
    telemetry = True
    repeat = False
    servo = -1.0

    params = hallParams(motorgains, duration, rightFreq, leftFreq, phase, telemetry, repeat, servo)
    setMotorGains(motorgains)

    leadIn = 10
    leadOut = 10
    strideFreq = 5
    phase = 0x8000      # Alternating tripod
    useFlag = 0
    deltas = [.25, 0.25, 0.25, 0.125, 0.125, 0.5]

    manParams = manueverParams(leadIn, leadOut, strideFreq, phase, useFlag, deltas)
    
    #xb_send(0, command.START_SERVO, pack('B', 0))
    
    
    comm = CommandInterface(addr, txCallback)
    telem = TelemetryReader(addr, txCallback)
    hall = HallController(4)
    kbint = KeyboardInterface(comm, hall)
    
    comm.enableDebug()
    telem.setConsoleMode(True)
    telem.setFileMode(True)
    telem.writeHeader()
    
    comm.setSlewLimit(3.0)

    while True:

        if not(params.repeat):
            settingsMenu(params, manParams)
            print "Set up H2Bird"
            while True:
                c = None
                if( msvcrt.kbhit() ):
                    c = msvcrt.getch()
                if c == '9':
                    break
                else:
                    kbint.process(c)   

        if params.telemetry:
            # Construct filename
            # path     = '/home/duncan/Data/'
            path     = 'Data/'
            name     = 'trial'
            datetime = time.localtime()
            dt_str   = time.strftime('%Y.%m.%d_%H.%M.%S', datetime)
            root     = path + dt_str + '_' + name
            shared.dataFileName = root + '_imudata.txt'
            print "Data file:  ", shared.dataFileName
            print os.curdir
            if manParams.useFlag == True:
                duration = 1.0/manParams.strideFreq * (manParams.leadIn + 1 + manParams.leadOut)
                numSamples = int(ceil(1000 * duration))
            else:
                numSamples = int(ceil(1000 * (params.duration + shared.leadinTime + shared.leadoutTime) / 1000.0))
            eraseFlashMem(numSamples)

        # Trigger telemetry save, which starts as soon as it is received
        if params.telemetry:
        # Pause and wait to start run, including leadin time
            raw_input("Press enter to start run ...") 
            startTelemetrySave(numSamples)
        #Start robot
        if manParams.useFlag == True:
            runManeuver(params, manParams)
        else:
            xb_send(0, command.SET_SERVO, pack('f', -1.0))
            time.sleep(0.01)
            xb_send(0, command.SET_PHASE, pack('l', params.phase))
            time.sleep(0.01)
            xb_send(0, command.START_TIMED_RUN, pack('h',params.duration))
            while True:
                c = None
                if( msvcrt.kbhit() ):
                    c = msvcrt.getch()
                kbint.process(c)
                if c == '9':
                    xb_send(0, command.SET_SERVO, pack('f', 1.0))
                if c == '1':
                    break  
            time.sleep(params.duration / 1000.0)

        if params.telemetry and query_yes_no("Save Data?"):
            flashReadback(numSamples, params, manParams)

        repeatMenu(params)

    print "Done"

#Provide a try-except over the whole main function
# for clean exit. The Xbee module should have better
# provisions for handling a clean exit, but it doesn't.
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print "\nRecieved Ctrl+C, exiting."
        shared.xb.halt()
        shared.ser.close()
    except Exception as args:
        print "\nGeneral exception:",args
        print "Attemping to exit cleanly..."
        shared.xb.halt()
        shared.ser.close()
        sys.exit()
    except serial.serialutil.SerialException:
        shared.xb.halt()
        shared.ser.close()
