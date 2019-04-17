'''
Mohamed Elgohary
ECE4313 CompE DP1 Final Project

Flight Controller HAT Basic Library
'''
from RPi import GPIO
from digitalio import DigitalInOut
import time
import board
import busio
import serial
import smbus

import adafruit_gps
import FaBo9Axis_MPU9250
import adafruit_rfm9x






class FlightControllerHAT:

    def __init__(self):

        if self.VerifyHAT():
            self.gps = None
            self.imu = None
            self.altimeter = None
            self.radio = None

#            self.last_print = time.monotonic()


#            print("Initializing HAT...")

            self.InitGPS()
#            print("  GPS initialized!")

            self.InitIMU()
#            print("  IMU initialized!")

            self.InitAltimeter()
#            print("  Altimeter initialized!")

            self.InitRadio()
#            print("  Radio initialized!")

#            print("  Initialization complete!\n")


    def VerifyHAT(self):
#        print("Verifying HAT...")

        try:
            filename = open('/proc/device-tree/hat/product')
            product = 'Flight_Controller_HAT' # This is what I'm checking for.
            firstLine = filename.readline()
            firstLine = firstLine[:len(product)] # Slice firstLine so that its length is same as product's length.

            if (firstLine == product):
                # If they are the same return True.
#                print('  Flight Controller HAT is connected!\n')
                return True

            else:
#                print('  Flight Controller HAT is NOT connected.\n')
                return False

        except IOError:
#            print('  Flight Controller HAT is not connected.\n')
            return False


    def InitGPS(self):
        # Using the pyserial library for UART access
        uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=3000)

        # Create a GPS module instance.
        self.gps = adafruit_gps.GPS(uart, debug=False)

        # Initialize the GPS module by changing what data it sends and at what rate.
        # These are NMEA extensions for PMTK_314_SET_NMEA_OUTPUT and
        # PMTK_220_SET_NMEA_UPDATERATE but you can send anything from here to adjust
        # the GPS module behavior:
        #   https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf

        # Turn on the basic GGA and RMC info (what you typically want)
        self.gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        # Turn on just minimum info (RMC only, location):
        #gps.send_command(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        # Turn off everything:
        #gps.send_command(b'PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        # Tuen on everything (not all of it is parsed!)
        #gps.send_command(b'PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0')

        # Set update rate to once a second (1hz) which is what you typically want.
        #self.gps.send_command(b'PMTK220,1000')
        # Or decrease to once every two seconds by doubling the millisecond value.
        # Be sure to also increase your UART timeout above!
        #self.gps.send_command(b'PMTK220,2000')
        # You can also speed up the rate, but don't go too fast or else you can lose
        # data during parsing.  This would be twice a second (2hz, 500ms delay):
        self.gps.send_command(b'PMTK220,500')


    def InitIMU(self):
        self.imu = FaBo9Axis_MPU9250.MPU9250()


    def InitAltimeter(self):
        # Get I2C bus
        self.altimeter = smbus.SMBus(1)

        # MPL3115A2 address, 0x60(96)
        # Select control register, 0x26(38)
        #       0xB9(185)   Active mode, OSR = 128, Altimeter mode
        self.altimeter.write_byte_data(0x60, 0x26, 0xB9)
        # MPL3115A2 address, 0x60(96)
        # Select data configuration register, 0x13(19)
        #       0x07(07)    Data ready event enabled for altitude, pressure, temperature
        self.altimeter.write_byte_data(0x60, 0x13, 0x07)
        # MPL3115A2 address, 0x60(96)
        # Select control register, 0x26(38)
        #       0xB9(185)   Active mode, OSR = 128, Altimeter mode
        self.altimeter.write_byte_data(0x60, 0x26, 0xB9)


    def InitRadio(self):
        # Configure LoRa Radio -- Comment/Uncomment lines according to which pins you are using.
        CS = DigitalInOut(board.CE2_1)
        RESET = DigitalInOut(board.D26)
        spi = busio.SPI(board.SCK_1, MOSI=board.MOSI_1, MISO=board.MISO_1)
        self.radio = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915.0)
        self.radio.tx_power = 23
#        prev_packet = None


    def ReadAccel(self):
        accel = self.imu.readAccel()
#        print("ax = %.3f\tay = %.3f\taz = %.3f\t\n" % (accel['x'], accel['y'], accel['z']) )
        return accel


    def ReadGyro(self):
        gyro = self.imu.readGyro()
#        print("gx = %.3f\tgy = %.3f\tgz = %.3f\t\n" % (gyro['x'], gyro['y'], gyro['z']) )
        return gyro


    def ReadMag(self):
        mag = self.imu.readMagnet()
#        print("mx = %.3f\tmy = %.3f\tmz = %.3f\t\n" % (mag['x'], mag['y'], mag['z']) )
        return mag


    def ReadAltitude(self):
        # MPL3115A2 address, 0x60(96)
        # Read data back from 0x00(00), 6 bytes
        # status, tHeight MSB1, tHeight MSB, tHeight LSB, temp MSB, temp LSB
        data = self.altimeter.read_i2c_block_data(0x60, 0x00, 6)

        # Convert the data to 20-bits
        tHeight = ((data[1] * 65536) + (data[2] * 256) + (data[3] & 0xF0)) / 16
        temp = ((data[4] * 256) + (data[5] & 0xF0)) / 16
        altitude = tHeight / 16.0
        cTemp = temp / 16.0
        fTemp = cTemp * 1.8 + 32

#        print("Altitude : %.2f m" %altitude)
        return altitude


    def ReadGPSPosition(self):
        # Make sure to call gps.update() every loop iteration and at least twice
        # as fast as data comes from the GPS unit (usually every second).
        # This returns a bool that's true if it parsed new data (you can ignore it
        # though if you don't care and instead look at the has_fix property)

        if not self.gps.has_fix:
            # Try again if we don't have a fix yet.
#            print('Waiting for fix...\n')
            return (0, 0)

        else:

            self.gps.update()

            '''
            # We have a fix! (gps.has_fix is true)
            # Print out details about the fix like location, date, etc.
            print('=' * 40)  # Print a separator line.
            print('Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}'.format(
                self.gps.timestamp_utc.tm_mon,   # Grab parts of the time from the
                self.gps.timestamp_utc.tm_mday,  # struct_time object that holds
                self.gps.timestamp_utc.tm_year,  # the fix time.  Note you might
                self.gps.timestamp_utc.tm_hour,  # not get all data like year, day,
                self.gps.timestamp_utc.tm_min,   # month!
                self.gps.timestamp_utc.tm_sec))

            print('Latitude: {0:.6f} degrees'.format(self.gps.latitude))
            print('Longitude: {0:.6f} degrees'.format(self.gps.longitude))
            print('Fix quality: {}'.format(self.gps.fix_quality))

            # Some attributes beyond latitude, longitude and timestamp are optional
            # and might not be present.  Check if they're None before trying to use!
            if self.gps.satellites is not None:
                print('# satellites: {}'.format(self.gps.satellites))
            if self.gps.altitude_m is not None:
                print('Altitude: {} meters'.format(self.gps.altitude_m))
            if self.gps.track_angle_deg is not None:
                print('Speed: {} knots'.format(self.gps.speed_knots))
            if self.gps.track_angle_deg is not None:
                print('Track angle: {} degrees'.format(self.gps.track_angle_deg))
            if self.gps.horizontal_dilution is not None:
                print('Horizontal dilution: {}'.format(self.gps.horizontal_dilution))
            if self.gps.height_geoid is not None:
                print('Height geo ID: {} meters'.format(self.gps.height_geoid))

            print('\n\n')
            '''

            return (self.gps.latitude, self.gps.longitude)



    def TransmitPacket(self, data):
        packet = bytes(data, "utf-8")
        self.radio.send(packet)








if __name__ == '__main__':


    try:
        hat = FlightControllerHAT()

        for i in range(10):
            position = hat.ReadGPSPosition() # position[0] is latitude, position [1] is longitude.
            height = hat.ReadAltitude()
            accelero = hat.ReadAccel() # accelero['x'] is ax, accelero['y'] is ay, and so on.
            gyroscope = hat.ReadGyro() # Same format as accelero.
            magneto = hat.ReadMag() # Same format as accelero.

            dataString = str(position[0]) + ', ' + str(position[1]) + ', ' + \
                         str(height) + ', ' + \
                         str(accelero['x']) + ', ' + str(accelero['y']) + ', ' + str(accelero['z']) + ', ' + \
                         str(gyroscope['x']) + ', ' + str(gyroscope['y']) + ', ' + str(gyroscope['z']) + ', ' + \
                         str(magneto['x']) + ', ' + str(magneto['y']) + ', ' + str(magneto['z'])

            print(dataString)
            hat.TransmitPacket(dataString)

            time.sleep(0.1)

        GPIO.cleanup()

    except KeyboardInterrupt:
        GPIO.cleanup()
