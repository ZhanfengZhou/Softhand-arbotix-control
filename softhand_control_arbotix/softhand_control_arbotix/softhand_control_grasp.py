import rclpy
import numpy
import time
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException

from std_msgs.msg import Float64


class Softhand_Publisher(Node):

    def __init__(self):
        super().__init__('softhand_publisher')    #node name
        self.publisher1_ = self.create_publisher(Float64, 'dynamixel1/command', 10)    #topic name
        self.publisher2_ = self.create_publisher(Float64, 'dynamixel2/command', 10)
        self.publisher3_ = self.create_publisher(Float64, 'dynamixel3/command', 10)
        self.publisher4_ = self.create_publisher(Float64, 'dynamixel4/command', 10)
        self.publisher5_ = self.create_publisher(Float64, 'dynamixel5/command', 10)
        self.publisher6_ = self.create_publisher(Float64, 'dynamixel6/command', 10)
        self.publisher7_ = self.create_publisher(Float64, 'dynamixel7/command', 10)
        self.publisher8_ = self.create_publisher(Float64, 'dynamixel8/command', 10)
        self.publisher9_ = self.create_publisher(Float64, 'dynamixel9/command', 10)
        self.publisher10_ = self.create_publisher(Float64, 'dynamixel10/command', 10)
        self.publisher_array = [self.publisher1_, self.publisher2_, self.publisher3_, self.publisher4_,
                    self.publisher5_, self.publisher6_, self.publisher7_, self.publisher8_,
                    self.publisher9_, self.publisher10_]
        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.declare_parameter('bend_angle_array')    #in degrees
        self.declare_parameter('wave_angle_array')

    def timer_callback(self):
        bend_angle_array = self.get_parameter('bend_angle_array').value
        bend_angle_array_new = [x-150 for x in bend_angle_array]    # bending starts from -150
        wave_angle_array = self.get_parameter('wave_angle_array').value
        bend_angle_array_r = [x/180*numpy.pi for x in bend_angle_array_new]    #change to radians
        wave_angle_array_r = [x/180*numpy.pi for x in wave_angle_array]
        
        fingerbend_2motorID = [3, 4, 5, 6, 1]
        fingerwave_2motorID = [2, 8, 7, 10, 9]
        
        for i in range(5):
            msg = Float64()
            msg.data = bend_angle_array_r[i]
            motorID = fingerbend_2motorID[i]
            topic_name = 'dynamixel%s/command' % str(motorID)
            self.publisher_array[motorID-1].publish(msg)
            self.get_logger().info('Publishing to {}: {}'.format(topic_name, 
                str(bend_angle_array[i]) ))
        
        time.sleep(1)
        
        for i in range(5):
            msg = Float64()
            msg.data = wave_angle_array_r[i]
            motorID = fingerwave_2motorID[i]
            topic_name = 'dynamixel%s/command' % str(motorID)
            self.publisher_array[motorID-1].publish(msg)   
            self.get_logger().info('Publishing to {}: {}'.format(topic_name, 
                str(wave_angle_array[i]) ))
        
def main(args=None):
    rclpy.init(args=args)

    softhand_publisher = Softhand_Publisher()

    rclpy.spin(softhand_publisher)

    softhand_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
