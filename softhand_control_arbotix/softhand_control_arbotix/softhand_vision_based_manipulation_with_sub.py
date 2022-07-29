from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import rclpy
import numpy
import time
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float64


class Softhand_Manipulation(Node):

    def __init__(self):
        super().__init__('softhand_manipulation')    #node name
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

        self.marker_angle_list = []
        self.marker_angle_filtered = -1   #initial marker angle
        
        self.subscription = self.create_subscription(
            Float64,
            'topic',
            self.listener_callback,
            10, callback_group=MutuallyExclusiveCallbackGroup())
        self.subscription  # prevent unused variable warning

        self.declare_parameter('bend_angle_array_point0')    #in degrees
        self.declare_parameter('bend_angle_array_point1')
        self.declare_parameter('wave_angle_array_point0')
        self.declare_parameter('wave_angle_array_point1')
        self.declare_parameter('wave_angle_array_point2')
        self.declare_parameter('grasp_time') 
        self.declare_parameter('release_time') 
        self.declare_parameter('manipulation_step_time') 
        self.declare_parameter('manipulation_steps') 
        self.declare_parameter('bend_step_angle') 
        self.declare_parameter('rotation_angle') 
        
        self.publish_motors_angle_steps()

    def listener_callback(self, msg):
        marker_angle =  msg.data
        
        if len(self.marker_angle_list) < 10:
            self.marker_angle_list.append(marker_angle)
        else:
            self.marker_angle_list = []

        if len(self.marker_angle_list) == 10:

            self.marker_angle_filtered = self.marker_angle_filter(self.marker_angle_list)
            self.get_logger().info(f'the filtered marker angle is {self.marker_angle_filtered}') 
    
    def marker_angle_filter(self, marker_angle_list):
        angle_sum = sum(marker_angle_list)
        angle_filtered = angle_sum / len(marker_angle_list)
        return angle_filtered

        
    def publish_motors_angle_steps(self):
    
        # get and process parameter:
        self.grasp_time = self.get_parameter('grasp_time').value
        self.release_time = self.get_parameter('release_time').value
        self.manipulation_step_time = self.get_parameter('manipulation_step_time').value
        self.manipulation_steps = self.get_parameter('manipulation_steps').value
        self.bend_step_angle = self.get_parameter('bend_step_angle').value
        self.rotation_angle = self.get_parameter('rotation_angle').value
        
        self.bend_angle_array_point0 = self.get_parameter('bend_angle_array_point0').value
        self.bend_angle_array_point1 = self.get_parameter('bend_angle_array_point1').value
        self.wave_angle_array_point0 = self.get_parameter('wave_angle_array_point0').value
        self.wave_angle_array_point1 = self.get_parameter('wave_angle_array_point1').value
        self.wave_angle_array_point2 = self.get_parameter('wave_angle_array_point2').value
        
        self.bend_angle_array_r_point0 = self.process_angle_param(self.bend_angle_array_point0, 'bend')
        self.bend_angle_array_r_point1 = self.process_angle_param(self.bend_angle_array_point1, 'bend')
        self.wave_angle_array_r_point0 = self.process_angle_param(self.wave_angle_array_point0, 'wave')
        self.wave_angle_array_r_point1 = self.process_angle_param(self.wave_angle_array_point1, 'wave')
        self.wave_angle_array_r_point2 = self.process_angle_param(self.wave_angle_array_point2, 'wave')
        
        #step 1: 
        self.get_logger().info('Moving soft hand! initializing ... \n ... ... \n ... ...')
        time.sleep(2)  # wait to start, or the first step will not work!!
        self.get_logger().info('Fingers moving to original point... ...')
        self.publishing_bend_angle(self.bend_angle_array_r_point0)
        self.publishing_wave_angle(self.wave_angle_array_r_point0)
        
        #step 2:
        time.sleep(2)
        self.get_logger().info('Grasping')
        self.publishing_bend_angle(self.bend_angle_array_r_point1)
        
        time.sleep(5)
        #step 3:
        timer_period = self.manipulation_step_time
        #timer_period = self.grasp_time
        
        self.wave_angle_array_r = self.wave_angle_array_r_point0
        self.bend_angle_array_r = self.bend_angle_array_r_point1
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.i = 0


        
    def timer_callback(self):
        if self.i == 0:
            self.marker_angle_initial = self.marker_angle_filtered
            # self.marker_angle_desired = self.marker_angle_filtered + self.rotation_angle
            self.marker_angle_desired = self.rotation_angle
            self.marker_angle_desired_back = -10
            

        direction = -1
            
        if self.i  < self.manipulation_steps:

            self.get_logger().info(f'Publisher timer: {self.i}, initial angle is {self.marker_angle_initial},  desired angle is {self.marker_angle_desired}  receiving marker angle as : {self.marker_angle_filtered}')
            marker_angle_error = self.marker_angle_desired - self.marker_angle_filtered

            if (marker_angle_error >= 1):
                direction = 0    #rotate to right
                self.get_logger().info('Rotating objects to right!')
                self.wave_angle_accumulate(direction)
                self.bend_angle_accumulate(direction)

                self.publishing_wave_angle(self.wave_angle_array_r)
                self.publishing_bend_angle(self.bend_angle_array_r)
            elif (marker_angle_error <= -1):
                direction = 1    #rotate to left
                self.get_logger().info('Rotating objects to left!')
                self.wave_angle_accumulate(direction)
                self.bend_angle_accumulate(direction)

                self.publishing_wave_angle(self.wave_angle_array_r)
                self.publishing_bend_angle(self.bend_angle_array_r)
            else:
                direction = -1
                self.get_logger().info('Error is 0 now!!!!!!!!!!Stop!!!!!!!!!!!!')
        elif (self.manipulation_steps + 10) <= self.i < 4*self.manipulation_steps :
            self.get_logger().info('Rotating back to left!')
            direction = 1    #rotate to left

            self.get_logger().info(f'Publisher timer: {self.i}, initial angle is {self.marker_angle_initial},  desired angle is {self.marker_angle_desired_back}  receiving marker angle as : {self.marker_angle_filtered}')
            marker_angle_error = self.marker_angle_desired_back - self.marker_angle_filtered
            if (marker_angle_error <= -1):
                self.get_logger().info('Rotating objects to left!')
                self.wave_angle_accumulate(direction)
                self.bend_angle_accumulate(direction)

                self.publishing_wave_angle(self.wave_angle_array_r)
                self.publishing_bend_angle(self.bend_angle_array_r)
            else:
                direction = -1
                self.get_logger().info('Error is 0 now!!!!!!!!!!Stop!!!!!!!!!!!!')


        elif self.i  >= 4*self.manipulation_steps:
            self.get_logger().info('Rotating limits reached')
        else:
            self.get_logger().error('Rotation stopped or wrong')
        
        self.i = self.i + 1

    def wave_angle_accumulate(self, direction):
        
        if direction == 0:    #rotate to right
            self.wave_angle_array_r[0] = self.wave_angle_array_r[0] + (self.wave_angle_array_r_point1[0] - self.wave_angle_array_r_point0[0]) / self.manipulation_steps
            self.wave_angle_array_r[1] = self.wave_angle_array_r[1] + (self.wave_angle_array_r_point1[1] - self.wave_angle_array_r_point0[1]) / self.manipulation_steps
            self.wave_angle_array_r[2] = self.wave_angle_array_r[2] + (self.wave_angle_array_r_point1[2] - self.wave_angle_array_r_point0[2]) / self.manipulation_steps
            self.wave_angle_array_r[3] = self.wave_angle_array_r[3] + (self.wave_angle_array_r_point1[3] - self.wave_angle_array_r_point0[3]) / self.manipulation_steps
            self.wave_angle_array_r[4] = self.wave_angle_array_r[4] + (self.wave_angle_array_r_point1[4] - self.wave_angle_array_r_point0[4]) / self.manipulation_steps
        elif direction == 1:   #rotate to left
            self.wave_angle_array_r[0] = self.wave_angle_array_r[0] + (self.wave_angle_array_r_point2[0] - self.wave_angle_array_r_point0[0]) / self.manipulation_steps
            self.wave_angle_array_r[1] = self.wave_angle_array_r[1] + (self.wave_angle_array_r_point2[1] - self.wave_angle_array_r_point0[1]) / self.manipulation_steps
            self.wave_angle_array_r[2] = self.wave_angle_array_r[2] + (self.wave_angle_array_r_point2[2] - self.wave_angle_array_r_point0[2]) / self.manipulation_steps
            self.wave_angle_array_r[3] = self.wave_angle_array_r[3] + (self.wave_angle_array_r_point2[3] - self.wave_angle_array_r_point0[3]) / self.manipulation_steps
            self.wave_angle_array_r[4] = self.wave_angle_array_r[4] + (self.wave_angle_array_r_point2[4] - self.wave_angle_array_r_point0[4]) / self.manipulation_steps
        else:
            self.get_logger().error('wave_angle_accumulate: No rotating direction or wave angle wrong ')
        
    def bend_angle_accumulate(self, direction):
        
        if direction == 0: #rotate to right
            self.bend_angle_array_r[1] -= self.bend_step_angle/180*numpy.pi
            self.bend_angle_array_r[3] += self.bend_step_angle/180*numpy.pi
        elif direction == 1:
            self.bend_angle_array_r[1] += self.bend_step_angle/180*numpy.pi
            self.bend_angle_array_r[3] -= self.bend_step_angle/180*numpy.pi
        else:
            self.get_logger().error('bend_angle_accumulate: No rotating direction or bend angle wrong ')

        
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

    softhand_manipulation_node = Softhand_Manipulation()

    executor = MultiThreadedExecutor()
    executor.add_node(softhand_manipulation_node)
    executor.spin()

    softhand_manipulation_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
