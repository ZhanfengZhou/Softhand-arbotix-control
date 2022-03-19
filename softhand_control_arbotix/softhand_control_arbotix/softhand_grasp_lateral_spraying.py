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
        
        self.declare_parameter('bend_angle_array_point0')    #in degrees
        self.declare_parameter('bend_angle_array_point1')
        self.declare_parameter('bend_angle_array_point2')
        self.declare_parameter('wave_angle_array_point0')
        self.declare_parameter('wave_angle_array_point1')
        self.declare_parameter('grasp_time1') 
        self.declare_parameter('grasp_time2') 
        self.declare_parameter('release_time1') 
        self.declare_parameter('release_time2') 
        
        self.publish_motors_angle_steps()
        
    def publish_motors_angle_steps(self):
    
        # get and process parameter:
        self.grasp_time1 = self.get_parameter('grasp_time1').value
        self.grasp_time2 = self.get_parameter('grasp_time2').value
        self.release_time1 = self.get_parameter('release_time1').value
        self.release_time2 = self.get_parameter('release_time2').value
        
        self.bend_angle_array_point0 = self.get_parameter('bend_angle_array_point0').value
        self.bend_angle_array_point1 = self.get_parameter('bend_angle_array_point1').value
        self.bend_angle_array_point2 = self.get_parameter('bend_angle_array_point2').value
        self.wave_angle_array_point0 = self.get_parameter('wave_angle_array_point0').value
        self.wave_angle_array_point1 = self.get_parameter('wave_angle_array_point1').value
        
        self.bend_angle_array_r_point0 = self.process_angle_param(self.bend_angle_array_point0, 'bend')
        self.bend_angle_array_r_point1 = self.process_angle_param(self.bend_angle_array_point1, 'bend')
        self.bend_angle_array_r_point2 = self.process_angle_param(self.bend_angle_array_point2, 'bend')
        self.wave_angle_array_r_point0 = self.process_angle_param(self.wave_angle_array_point0, 'wave')
        self.wave_angle_array_r_point1 = self.process_angle_param(self.wave_angle_array_point1, 'wave')
        
        #step 1: 
        self.get_logger().info('Moving soft hand! initializing ... \n ... ... \n ... ...')
        time.sleep(2)  # wait to start, or the first step will not work!!
        self.get_logger().info('Fingers moving to first point... ...')
        self.publishing_bend_angle(self.bend_angle_array_r_point0)
        self.publishing_wave_angle(self.wave_angle_array_r_point0)
        
        
        time.sleep(self.grasp_time1)
        #step 2:
        self.get_logger().info('Fingers moving to second point... ...')
        self.publishing_bend_angle(self.bend_angle_array_r_point1)
        self.publishing_wave_angle(self.wave_angle_array_r_point1)
        
        time.sleep(self.grasp_time2)
        #step 2:
        self.get_logger().info('Fingers moving to second point... ...')
        self.publishing_bend_angle(self.bend_angle_array_r_point2)
        self.publishing_wave_angle(self.wave_angle_array_r_point1)
        
        time.sleep(self.release_time1)
        #step 3:
        self.get_logger().info('Fingers moving to second point... ...')
        self.publishing_bend_angle(self.bend_angle_array_r_point1)
        self.publishing_wave_angle(self.wave_angle_array_r_point1)
        
        time.sleep(self.release_time1)
        #step 3:
        self.get_logger().info('Fingers moving to second point... ...')
        self.publishing_bend_angle(self.bend_angle_array_r_point2)
        self.publishing_wave_angle(self.wave_angle_array_r_point1)
        
        time.sleep(self.release_time1)
        #step 3:
        self.get_logger().info('Fingers moving to second point... ...')
        self.publishing_bend_angle(self.bend_angle_array_r_point1)
        self.publishing_wave_angle(self.wave_angle_array_r_point1)
        
        #step 4:
        timer_period = self.release_time2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        
        #step 3.1:
        self.get_logger().info('Fingers moving to fifth point... ...')
        self.publishing_bend_angle(self.bend_angle_array_r_point0)
        self.publishing_wave_angle(self.wave_angle_array_r_point0)

        
    def process_angle_param(self, angle_array, move_types):
        #process angle array to radians
        angle_array_r = None
        if move_types == 'bend':
            angle_array = [x-150 for x in angle_array]
            angle_array_r = [x/180*numpy.pi for x in angle_array]
        elif move_types == 'wave':
            angle_array_r = [x/180*numpy.pi for x in angle_array]
        else:
            self.get_logger().error('process_angle_param(): Wrong argument input!')
        return angle_array_r
        
    def publishing_bend_angle(self, bend_angle_array_r_point):
                
        fingerbend_2motorID = [3, 4, 5, 6, 1]
        for i in range(5):
            msg = Float64()
            msg.data = bend_angle_array_r_point[i]
            motorID = fingerbend_2motorID[i]
            topic_name = 'dynamixel%s/command' % str(motorID)
            self.publisher_array[motorID-1].publish(msg)
            self.get_logger().info('Publishing to {}: {}'.format(topic_name, str(msg.data) ))
            
    def publishing_wave_angle(self, wave_angle_array_r_point):
        
        fingerwave_2motorID = [2, 8, 7, 10, 9]
        for i in range(5):
            msg = Float64()
            msg.data = wave_angle_array_r_point[i]
            motorID = fingerwave_2motorID[i]
            topic_name = 'dynamixel%s/command' % str(motorID)
            self.publisher_array[motorID-1].publish(msg)   
            self.get_logger().info('Publishing to {}: {}'.format(topic_name, str(msg.data) ))

        
def main(args=None):
    rclpy.init(args=args)

    softhand_publisher = Softhand_Publisher()

    rclpy.spin(softhand_publisher)

    softhand_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
