"""
Sends commands to Arduino Nano via Serial bus which then
are send them to nRF24L01+ 2.4G via SPI to control
the drone flight.

Commands:
	[arrows] = Pitch
	[W-S] = Throttle up/down
	[A-D] = Yaw
    
"""
import serial, time, msvcrt

def calibrate ( throttle, aileron, elevator, rudder):
		# First step both sticks Right-Down
		throttle=1000 # Throttle power	for stand on position 
		aileron=2000 # Left-Right pitch
		elevator=1000 # Up-Down pitch
		rudder=2000 # yaw
		command = "%i,%i,%i,%i" % (throttle, aileron, elevator, rudder)	 
		arduino.write(command + "\n")
		time.sleep(2)

		# Second step sticks to middle
		throttle=1000 # Throttle power	for stand on position 
		aileron=1500 # Left-Right pitch
		elevator=1500 # Up-Down pitch
		rudder=1500 # yaw
		command = "%i,%i,%i,%i" % (throttle, aileron, elevator, rudder)	 
		arduino.write(command + "\n")
		time.sleep(1)
		
		# Third step both sticks Left-Down
		throttle=1000 # Throttle power	for stand on position 
		aileron=1000 # Left-Right pitch
		elevator=1000 # Up-Down pitch
		rudder=1000 # yaw
		command = "%i,%i,%i,%i" % (throttle, aileron, elevator, rudder)	 
		arduino.write(command + "\n")
		time.sleep(1)
		
		return
		
def restriction ( value):
			if value < 1000:
				return 1000
			elif value > 2000:
				return 2000
			else:
				return value

def printHelp():
	print "\nx=========x\n"\
			"|  HELP:  |\n"\
			"x======================================x\n"\
			"|  C   - Calibration                   |\n"\
			"|  F   - Fligth straight               |\n"\
			"|  H   - List of functions             |\n"\
			"|  R   - Reset values                  |\n"\
            "|  ESC - Exit                          |\n"\
			"x======================================x\n"\
			"|  [W-S]    = Throttle up/down         |\n"\
			"|  [A-D]    = Yaw                      |\n"\
			"|  [arrows] = Pitch                    |\n"\
			"x======================================x\n"
				

				
throttle=1000 # Throttle power 
aileron=1500 # Left-Right pitch
elevator=1500 # Up-Down pitch
rudder=1500 # yaw

tg=25 # Throttle gain
ag=50 # Aileron gain
eg=50 # Elevator gain
rg=50 # Rudder gain

comPort = 'COM4'

try:
	arduino=serial.Serial(comPort, 115200, timeout=.01)
	time.sleep(1)
	while True:
		
		data = arduino.readline()
		if data:
			# Responses from Arduino are [DRONE]
			print "[DRONE]: "+data 
			
		if msvcrt.kbhit():
			key = ord(msvcrt.getch())
			if key == 27: #ESC
				print "[CONTROLLER] ESC exiting"
				break
			elif key == 99: #c
				print "[CONTROLLER]: C calibration"
				calibrate ( throttle, aileron, elevator, rudder)
			elif key == 102: #f
				print "[CONTROLLER]: F stand still"
				throttle=1725 # Throttle power	for stand on position 
				aileron=1500 # Left-Right pitch
				elevator=1500 # Up-Down pitch
				rudder=1500 # yaw
			elif key == 104: # h
				printHelp()
				continue
			elif key == 114: #r
				print "[CONTROLLER]: R reset"
				throttle=1000 # Throttle power 
				aileron=1500 # Left-Right pitch
				elevator=1500 # Up-Down pitch
				rudder=1500 # yaw
			elif key == 13: # Enter
				print "[CONTROLLER]: Enter"
			elif key == 119: # Less throttle
				throttle+=tg
			elif key == 97: #a
				rudder-=rg		 
			elif key == 115: #s
				throttle-=tg
			elif key == 100: #d
				rudder+=rg
			elif key == 224: # Special keys
				key = ord(msvcrt.getch())
				if key == 80: # Down pitch
					elevator -= eg
				elif key == 72: # Up pitch
					elevator += eg
				elif key == 77: # Right pitch
					aileron += ag
				elif key == 75: # Left pitch
					aileron -= ag			   
			throttle = restriction (throttle)	# limit values in range [1000,2000]
			aileron= restriction (aileron)
			elevator = restriction (elevator)
			rudder = restriction (rudder)
			command = "%i,%i,%i,%i" % (throttle, aileron, elevator, rudder)	 
			# Commands to Arudino are [CONTROLLER]
			print "[CONTROLLER]: " + command 
			arduino.write(command + "\n")

finally:
	# When script exists, reboot Arduino
	# Basically, close the serial connection the re-open it to reset
	arduino.close()
	arduino=serial.Serial(comPort, 115200, timeout=.01)
	# Then close it again so it's not used anymore.
	arduino.close()
	
