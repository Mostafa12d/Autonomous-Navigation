#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from rclpy.parameter import Parameter
import serial
import utm
import re
from gps_msgs.msg import GPSmsg

class GPSNode(Node):
   def __init__(self):
      super().__init__('gps_node')
      #SENSOR_NAME = "GPS"
      self.declare_parameter('port', '/dev/ttyUSB0')
      self.declare_parameter('baudrate', 4800)
      self.declare_parameter('sampling_rate', 10.0)

      serial_port = self.get_parameter('port').value
      serial_baud = self.get_parameter('baudrate').value
      sampling_rate = self.get_parameter('sampling_rate').value

      try:
            self.ser = serial.Serial(serial_port, serial_baud, timeout=1)
            self.get_logger().info('GPS sensor connected to port: %s' % serial_port)
      except serial.SerialException as err:
            self.get_logger().error('GPS sensor connection error: %s' % err)
            return

      self.publisher_ = self.create_publisher(GPSmsg, 'gps', 10)
      self.timer = self.create_timer(1/sampling_rate, self.timer_callback)

      # re allows us to search for a regular expression in a string
      self.gpgga = re.compile(r'^\$GPGGA.*')
      
   def parse_gps_data(self, line):
      data = line.split(',')
      time = data[1]
      secs, nsecs = self.time_parser(time)
      try:
         latitude = float(data[2])
      except:
         latitude = 0.0

      lat_direction = data[3]
      decimal_latitude = self.direction_decimal(lat_direction, latitude)
      try:
         longitude = float(data[4])
      except:
         longitude = 0.0

      long_direction = data[5]
      decimal_longitude = self.direction_decimal(long_direction, longitude)
      try:
         altitude = float(data[9])
      except:
         altitude = 0.0

      return [secs, nsecs, decimal_latitude, decimal_longitude, altitude]
      
   def time_parser(self, time):
      hours = int(time[:2])
      minutes = int(time[2:4])
      seconds = int(time[4:6])
      fracofsec = float("0" + time[6:]) if len(time) > 6 else 0.0
      print(hours)
      total_seconds = (hours * 3600) + (minutes * 60) + seconds + fracofsec
      # print(total_seconds)
      secs = int(total_seconds)
      nsecs = int((total_seconds - secs) * 1e9)

      return secs, nsecs
      #return hours, minutes, seconds, fracofsec
      
   def direction_decimal(self, direction, value):
      degrees = int(value / 100)
      minutes = value - (degrees * 100)
      decimal_degrees = degrees + minutes / 60.0
      if direction == 'S' or direction == 'W':
         decimal_degrees *= -1
      return decimal_degrees
     
   def timer_callback(self):
      line = self.ser.readline().decode('ascii')
      if self.gpgga.match(line):
         gps_data = self.parse_gps_data(line)
         self.publish_gps_data(gps_data)
   def publish_gps_data(self, gps_data):
      msg = GPSmsg()
      msg.header = Header()
      msg.header.stamp = self.get_clock().now().to_msg()
      msg.header.frame_id = 'GPS1_FRAME'

      utm_result = utm.from_latlon(gps_data[2], gps_data[3])
      msg.latitude = gps_data[2]
      msg.longitude = gps_data[3]
      msg.altitude = gps_data[4]
      msg.utm_easting = utm_result[0]
      msg.utm_northing = utm_result[1]
      try:
         msg.zone = utm_result[2]
      except:
         msg.zone = 0
      try:
         msg.letter = str(utm_result[3])
      except:
         msg.letter = 'X'

      self.publisher_.publish(msg)
      self.get_logger().info('GPS: Latitude: %f, Longitude: %f, Altitude: %f, UTM Easting: %f, UTM Northing: %f, Letter: %s, Zone: %s' % (msg.latitude, msg.longitude, msg.altitude, msg.utm_easting, msg.utm_northing, msg.letter, msg.zone))

def main(args=None):
   rclpy.init(args=args)
   gps_driver = GPSNode()
   rclpy.spin(gps_driver)

   gps_driver.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()
