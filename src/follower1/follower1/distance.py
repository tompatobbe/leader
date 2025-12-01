#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import time
from sensor_msgs.msg import Range

# --- Configuration ---
GPIO_TRIGGER = 23
GPIO_ECHO = 24
MIN_RANGE = 0.02 # 2cm in meters
MAX_RANGE = 4.0  # 4m in meters
FOV = 0.26       # Field of view in radians (approx 15 degrees)

def measure_distance():
    # Ensure trigger is low
    GPIO.output(GPIO_TRIGGER, False)
    time.sleep(0.000002)

    # Send 10us pulse to trigger
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    start_time = time.time()
    stop_time = time.time()

    # Save StartTime
    # This loop waits for the Echo pin to go HIGH
    while GPIO.input(GPIO_ECHO) == 0:
        start_time = time.time()
        # Safety break to prevent infinite loops if sensor fails
        if start_time - stop_time > 0.1: 
            return -1

    # Save StopTime
    # This loop waits for the Echo pin to go LOW
    while GPIO.input(GPIO_ECHO) == 1:
        stop_time = time.time()
        # Safety break
        if stop_time - start_time > 0.1:
            return -1

    # Time difference between start and arrival
    time_elapsed = stop_time - start_time

    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (time_elapsed * 343) / 2

    return distance

def sonar_publisher():
    # Initialize the node
    rospy.init_node('hcsr04_sensor', anonymous=True)
    
    # Create publisher
    # We use the standard 'Range' message type for distance sensors
    pub = rospy.Publisher('/sonar_dist', Range, queue_size=10)
    
    # Set the loop rate (10 Hz is standard for these sensors)
    rate = rospy.Rate(10) 

    # GPIO Setup
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
    GPIO.setup(GPIO_ECHO, GPIO.IN)

    try:
        rospy.loginfo("Ultrasonic Sensor Node Started...")
        
        while not rospy.is_shutdown():
            dist = measure_distance()

            # Create the message object
            msg = Range()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "ultrasonic_link"
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = FOV 
            msg.min_range = MIN_RANGE
            msg.max_range = MAX_RANGE
            msg.range = dist

            # Only publish valid readings
            if dist > 0:
                pub.publish(msg)
                rospy.logdebug("Distance: %.2f m" % dist)
            else:
                rospy.logwarn("Sensor read timeout")

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        # Clean up GPIO on exit
        GPIO.cleanup()
        rospy.loginfo("GPIO Cleaned up.")

if __name__ == '__main__':
    sonar_publisher()