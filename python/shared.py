from vidStreamer import VideoStreamer

DEST_ADDR = '\x10\x23'
xb = []
ser = []

#Base station
BS_COMPORT = 'COM3'
# BS_COMPORT = '/dev/ttyUSB0'
#BS_BAUDRATE = 230400
BS_BAUDRATE = 57600
#XBee
# BS_COMPORT = 'COM2'
# BS_BAUDRATE = 57600

motor_gains_set = False
steering_gains_set = False
steering_rate_set = False
flash_erased = 0
pkts = 0
bytesIn = 0

last_packet_time = 0
readback_timeout = 2 #seconds

awake = True;

# Cross-module variable sharing; these need default values
imudata = []
dataFileName = ''
leadinTime = 0
leadoutTime = 0
angRateDeg = "NOT SET"  #This is only for writing the file header
angRate = "NOT SET"
motorGains = "NOT SET"
steeringGains = "NOT SET"
runtime = 0
numSamples = 0
moveq = "NOT SET"
curr_line_dat = []
curr_line_type = 0

streamer = None

robotQueried = False
maxQueries = 8