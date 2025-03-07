import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Header
from rclpy.parameter import Parameter
import serial
import utm
import re
from imu_message.msg import IMUmsg
#removed from my message file to match the message that was collected in the 5hrs dataset by my labmate
#string raw_data
class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        serial_port = self.get_parameter('port').value
        serial_baud = self.get_parameter('baudrate').value
        cmd = "$VNWRG,07,40\r\n"

        try:
            self.ser = serial.Serial(serial_port, serial_baud, timeout=1)
            self.get_logger().info('IMU sensor connected to port: %s' % serial_port)
        except serial.SerialException as err:
            self.get_logger().error('IMU sensor connection error: %s' % err)
            return
        
        self.ser.write(cmd.encode('ascii'))
        
        frequencey = 40
        self.publisher_ = self.create_publisher(IMUmsg, '/imu', 10)
        self.timer = self.create_timer(1.0/frequencey, self.timer_callback)
        
        #imu is VNYMR
        #self.imu_pattern = re.compile(r'\$VNYMR,[^*]*\*..')

    def calculate_quaternion(self, roll, pitch, yaw):
        roll = np.radians(roll)
        pitch = np.radians(pitch)
        yaw = np.radians(yaw)
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return qx, qy, qz, qw
    
    def tesla_to_gauss(self, tesla):
        return tesla * 10000
    
    def parse_imu_data(self, line):
    # IMU data $VNYMR,-165.972,-037.299,+001.254,+00.2860,+00.0735,+00.7465,-05.953,-00.172,-07.799,+00.001606,-00.000007,-00.000762*64
        # imu_match = self.imu_pattern.search(line)
        # if not imu_match:
        #     return None

        # imu_sentence = imu_match.group(0)
        # data = line.split(',')
        if line.startswith('$VNYMR'):
            data = line.split(',')
            try:
                roll = float(data[3]) if data[3] else 0.0
                pitch = float(data[2]) if data[2] else 0.0
                yaw = float(data[1]) if data[1] else 0.0
                compass_x = float(data[4]) if data[4] else 0.0
                compass_y = float(data[5]) if data[5] else 0.0
                compass_z = float(data[6]) if data[6] else 0.0
                accel_x = float(data[7]) if data[7] else 0.0
                accel_y = float(data[8]) if data[8] else 0.0
                accel_z = float(data[9]) if data[9] else 0.0
                gyro_x = float(data[10]) if data[10] else 0.0
                gyro_y = float(data[11]) if data[11] else 0.0
                gyro_z = float(data[12][0:9]) if data[12][0:9] else 0.0

                qx, qy, qz, qw = self.calculate_quaternion(roll, pitch, yaw)
                compass_x = self.tesla_to_gauss(compass_x)
                compass_y = self.tesla_to_gauss(compass_y)
                compass_z = self.tesla_to_gauss(compass_z)

            except (IndexError, ValueError) as e:
                self.get_logger().error('IMU data parsing error: %s' % e)
                return None


        return [roll, pitch, yaw, compass_x, compass_y, compass_z, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, qx, qy, qz, qw]
    
    def publish_imu_data(self, imu_data):
        msg = IMUmsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'IMU1_Frame'
        msg.imu.orientation.x = imu_data[12]
        msg.imu.orientation.y = imu_data[13]
        msg.imu.orientation.z = imu_data[14]
        msg.imu.orientation.w = imu_data[15]
        msg.imu.linear_acceleration.x = imu_data[6]
        msg.imu.linear_acceleration.y = imu_data[7]
        msg.imu.linear_acceleration.z = imu_data[8]
        msg.imu.angular_velocity.x = imu_data[9]
        msg.imu.angular_velocity.y = imu_data[10]
        msg.imu.angular_velocity.z = imu_data[11]
        msg.mag_field.magnetic_field.x = imu_data[3]
        msg.mag_field.magnetic_field.y = imu_data[4]
        msg.mag_field.magnetic_field.z = imu_data[5]
        #commented to match the 5 hours rosbag dataset
        #msg.raw_data = str(imu_data)
        self.publisher_.publish(msg)

        # self.get_logger().info('Published IMU data %s', imu_data)
    
    
    def timer_callback(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line.startswith('$VNYMR'):
                imu_data = self.parse_imu_data(line)
                self.publish_imu_data(imu_data)
        except Exception as e:
            self.get_logger().error(f"Error reading/publishing data: {e}")

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
