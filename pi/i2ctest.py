import smbus
import time
import RPi.GPIO as GPIO  

GPIO.setmode(GPIO.BOARD)  
  
GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04
myaddress = 0x03  # On this address we are listening for button presses
  
# now we'll define two threaded callback functions  
# these will run in another thread when our events are detected  
def my_callback(channel):  
    print "falling edge detected on 11"  
  
GPIO.add_event_detect(11, GPIO.FALLING, callback=my_callback, bouncetime=300)  
  
def writeI2C(value):
  #bus.write_byte(address, value)
  bus.write_i2c_block_data(address, value[0], value[1:])
#  bus.write_byte(address, value[1])

def readI2C():
  return bus.read_byte(address)

try:
  while True:
    cmd = raw_input("Enter command and timeout (1+1 char)")
    if len(cmd) == 2:
      cmd = [ord(x) for x in cmd]
      writeI2C(cmd)
      print "RPI: Hi Arduino, I sent you ", cmd
  
    # sleep one second
    time.sleep(1)

    interrupting = readI2C()
    print "Arduino: Hey RPI, I received 0x", interrupting, " = ", chr(interrupting)
    print

#    GPIO.wait_for_edge(24, GPIO.RISING)  
    pin = GPIO.input(11)
    print "Pin 11 is", pin
except KeyboardInterrupt:
  print "Goodbye"


GPIO.cleanup()           # clean up GPIO on normal exit  
