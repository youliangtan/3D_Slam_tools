import roslib; roslib.load_manifest('razor_imu_9dof')
import rospy

import serial
import string
import math

from time import time, sleep
from sensor_msgs.msg import Imu
from razor_imu_9dof.msg import RazorImu


import tf

grad2rad = 3.141592/180.0

rospy.init_node("node")
pub = rospy.Publisher('imu', Imu)
pubRaw = rospy.Publisher('imuRaw', RazorImu)
#print pubRaw

#epub = rospy.Publisher('imuRaw', Imu)

imuMsg = Imu()
imuRawMsg = RazorImu()
imuMsg.orientation_covariance = [999999 , 0 , 0,  0, 9999999, 0,  0, 0, 999999]
imuMsg.angular_velocity_covariance = [9999, 0 , 0,  0 , 99999, 0,  0 , 0 , 0.02]
imuMsg.linear_acceleration_covariance = [0.2 , 0 , 0,  0 , 0.2, 0,  0 , 0 , 0.2]

default_port='/dev/ttyUSB0'
port = rospy.get_param('device', default_port)
# Check your COM port and baud rate
ser = serial.Serial(port=port,baudrate=57600, timeout=1)

#f = open("Serial"+str(time())+".txt", 'w')

roll=0
pitch=0
yaw=0
rospy.sleep(5) # Sleep for 8 seconds to wait for the board to boot then only write command.
ser.write('#ox' + chr(13)) # To start display angle and sensor reading in text 
while 1:
    line = ser.readline()
    line = line.replace("#YPR=","")
    line = line.replace("#YPRAMG=","")   # Delete "#YPR="
    #f.write(line)                     # Write to the output log file
    words = string.split(line,",")    # Fields split
    #wort5 = float(words[5]) 
    #print wort3, wort4, wort5

    if len(words) > 2:
        try:
            yaw = float(words[0])*grad2rad
            pitch = -float(words[1])*grad2rad
            roll = -float(words[2])*grad2rad

            # Publish message
            imuMsg.linear_acceleration.x = float(words[3]) # tripe axis accelerator meter
            imuMsg.linear_acceleration.y = float(words[4])
            imuMsg.linear_acceleration.z = float(words[5])
        #print imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y ,imuMsg.linear_acceleration.z

        #epub.publish(imuMsg)

            imuMsg.angular_velocity.x = float(words[9]) #gyro
            imuMsg.angular_velocity.y = float(words[10]) 
            imuMsg.angular_velocity.z = float(words[11])
        except Exception as e:
            print e

        q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        imuMsg.orientation.x = q[0] #magnetometer
        imuMsg.orientation.y = q[1]
        imuMsg.orientation.z = q[2]
        imuMsg.orientation.w = q[3]
    #time.sleep( 2 )
    #print imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w
        imuMsg.header.stamp= rospy.Time.now()
        imuMsg.header.frame_id = 'base_link'
        pub.publish(imuMsg)
    print imuMsg #Gibt alle Sensorwerte in einer Art Tabelle aus


        # Publish Raw message from Razor board
        imuRawMsg.yaw = yaw
        imuRawMsg.pitch = pitch
        imuRawMsg.roll = roll
        pubRaw.publish(imuRawMsg)
    #print imuRawMsg


ser.close
#f.close