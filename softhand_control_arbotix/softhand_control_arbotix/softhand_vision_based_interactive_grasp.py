import rclpy
import numpy
import time
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException

from std_msgs.msg import Float64
from example_interfaces.srv import SetBool

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class Softhand_Publisher(Node):

    def __init__(self):
        super().__init__('softhand_publisher')    #node name
        self.publisher1_ = self.create_publisher(Float64, 'dynamixel1/command', 10, callback_group=MutuallyExclusiveCallbackGroup())    #topic name
        self.publisher2_ = self.create_publisher(Float64, 'dynamixel2/command', 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.publisher3_ = self.create_publisher(Float64, 'dynamixel3/command', 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.publisher4_ = self.create_publisher(Float64, 'dynamixel4/command', 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.publisher5_ = self.create_publisher(Float64, 'dynamixel5/command', 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.publisher6_ = self.create_publisher(Float64, 'dynamixel6/command', 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.publisher7_ = self.create_publisher(Float64, 'dynamixel7/command', 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.publisher8_ = self.create_publisher(Float64, 'dynamixel8/command', 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.publisher9_ = self.create_publisher(Float64, 'dynamixel9/command', 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.publisher10_ = self.create_publisher(Float64, 'dynamixel10/command', 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.publisher_array = [self.publisher1_, self.publisher2_, self.publisher3_, self.publisher4_,
                    self.publisher5_, self.publisher6_, self.publisher7_, self.publisher8_,
                    self.publisher9_, self.publisher10_]
        
        self.declare_parameter('bend_angle_array_point0')    #in degrees
        self.declare_parameter('bend_angle_array_point1')
        self.declare_parameter('bend_angle_array_point_shaking')
        self.declare_parameter('wave_angle_array_point0')
        self.declare_parameter('wave_angle_array_point1')
        
        self.publish_motors_angle_steps()

        self.service = self.create_service(SetBool, 'Grasp_object', self.service_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.service_shakehand = self.create_service(SetBool, 'shake_hand', self.service_shakehand_callback, callback_group=MutuallyExclusiveCallbackGroup())
        
    def publish_motors_angle_steps(self):
    
        # get and process parameter:
        self.bend_angle_array_point0 = self.get_parameter('bend_angle_array_point0').value
        self.bend_angle_array_point1 = self.get_parameter('bend_angle_array_point1').value
        self.bend_angle_array_point_shaking = self.get_parameter('bend_angle_array_point_shaking').value
        self.wave_angle_array_point0 = self.get_parameter('wave_angle_array_point0').value
        self.wave_angle_array_point1 = self.get_parameter('wave_angle_array_point1').value
        
        self.bend_angle_array_r_point0 = self.process_angle_param(self.bend_angle_array_point0, 'bend')
        self.bend_angle_array_r_point1 = self.process_angle_param(self.bend_angle_array_point1, 'bend')
        self.bend_angle_array_r_point_shaking = self.process_angle_param(self.bend_angle_array_point_shaking, 'bend')
        self.wave_angle_array_r_point0 = self.process_angle_param(self.wave_angle_array_point0, 'wave')
        self.wave_angle_array_r_point1 = self.process_angle_param(self.wave_angle_array_point1, 'wave')
        
        #original point: 
        self.get_logger().info('Moving soft hand! initializing ... \n ... ... \n ... ...')
        time.sleep(2)  # wait to start, or the first step will not work!!
        self.get_logger().info('Fingers moving to original point... ...')
        self.publishing_bend_angle(self.bend_angle_array_r_point0)


    def service_callback(self, request, response):
        response.success = request.data  #  if False: release object; else if True, grasp object.

        if response.success :
            response.message = 'Start grasping'
            self.get_logger().info(f'I got {response.success}, {response.message}')
            
            self.get_logger().info('Fingers moving to the grasping point... ...')
            self.publishing_bend_angle(self.bend_angle_array_r_point1)

        else:
            response.message = 'Start Releasing'
            self.get_logger().info(f'I got {response.success}, {response.message}')

            self.get_logger().info('Fingers moving back to releasing point... ...')
            self.publishing_bend_angle(self.bend_angle_array_r_point0)

        return response
        
    def service_shakehand_callback(self, request, response):
        response.success = request.data  #  if False: release object; else if True, grasp object.

        if response.success :
            response.message = 'Start grasping'
            self.get_logger().info(f'I got {response.success}, {response.message}')
            
            self.get_logger().info('Fingers moving to the grasping point... ...')
            self.publishing_bend_angle(self.bend_angle_array_r_point_shaking)

        else:
            response.message = 'Start Releasing'
            self.get_logger().info(f'I got {response.success}, {response.message}')

            self.get_logger().info('Fingers moving back to releasing point... ...')
            self.publishing_bend_angle(self.bend_angle_array_r_point0)

        return response
        
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


    # I am using a MultiThreadedExecutor here as I want all the callbacks to run on a different thread each 
    executor = MultiThreadedExecutor()
    try:
        executor.add_node(softhand_publisher)
        executor.spin()
    except KeyboardInterrupt:
        pass

    softhand_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
