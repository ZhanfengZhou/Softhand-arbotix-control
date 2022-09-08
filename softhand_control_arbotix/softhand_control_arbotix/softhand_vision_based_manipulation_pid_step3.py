from fileinput import filename
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import rclpy
import numpy
import time
import termcolor
import csv
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.executors import MultiThreadedExecutor

from threading import Thread

from std_msgs.msg import Float64
from arbotix_msgs.srv import Enable, SetSpeed, Relax


class Softhand_Manipulation(Node):

    def __init__(self):
        super().__init__('softhand_manipulation')    #node name


        # motor position publisher
        self.publisher1_ = self.create_publisher(Float64, 'dynamixel1/command', 10)    
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
        
        # motor speed client
        self.client1 = self.create_client_i(1)    #service client name
        self.client2 = self.create_client_i(2)
        self.client3 = self.create_client_i(3)
        self.client4 = self.create_client_i(4)
        self.client5 = self.create_client_i(5)
        self.client6 = self.create_client_i(6)
        self.client7 = self.create_client_i(7)
        self.client8 = self.create_client_i(8)
        self.client9 = self.create_client_i(9)
        self.client10 = self.create_client_i(10)
        self.client_array = [self.client1, self.client2, self.client3, self.client4, self.client5,
                self.client6, self.client7, self.client8, self.client9, self.client10]

        self.marker_angle_list = []
        self.marker_angle_time_list = []
        self.marker_angle_feedback = -1   #initial marker angle
        
        self.subscription = self.create_subscription(
            Float64,
            'topic',
            self.camera_callback,
            10, callback_group=MutuallyExclusiveCallbackGroup())
        self.subscription  # prevent unused variable warning

        self.declare_parameter('bend_angle_array_point0')    #in degrees
        self.declare_parameter('bend_angle_array_limit')
        self.declare_parameter('wave_angle_array_point0')
        self.declare_parameter('wave_angle_array_limit1')
        self.declare_parameter('wave_angle_array_limit2')

        self.declare_parameter('bend_speed_array_preset') 
        self.declare_parameter('wave_speed_array_preset')

        self.declare_parameter('K_P') 
        self.declare_parameter('K_I') 
        self.declare_parameter('K_D') 

        self.object_grasped = False
        self.marker_feedback_update_times = 0
        self.speed_preset = False
        self.PID_controller_ready = False
        self.marker_angle_desired = 12

        self.motors_parameter_set()

    def create_client_i(self, client_num):
        srv_name = 'dynamixel%s/set_speed' % str(client_num)
        client = self.create_client(SetSpeed, srv_name, callback_group=MutuallyExclusiveCallbackGroup())
        
        # check if service is connected.
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(termcolor.colored(f'service {client_num} not available, waiting again...', 'blue' ))

        self.get_logger().info(termcolor.colored(f'server {client_num} is connected to client {client_num}', 'blue' ))
        
        return client

    def send_request(self):

        bend_speed_array_preset = self.get_parameter('bend_speed_array_preset').value    #get preset speed degrees/sec
        wave_speed_array_preset = self.get_parameter('wave_speed_array_preset').value
        bend_speed_array_preset_r = [x/180*numpy.pi for x in bend_speed_array_preset]    #in radians/sec
        wave_speed_array_preset_r = [x/180*numpy.pi for x in wave_speed_array_preset]
        
        self.get_logger().info(termcolor.colored(f'MotorSpeed Client Start working: sending preset speed to motors ... ...', 'blue'))

        self.sending_bend_speed(bend_speed_array_preset_r)
        self.sending_wave_speed(wave_speed_array_preset_r)

        self.speed_preset = True
        self.wave_speed = 1

        self.get_logger().info(termcolor.colored(f'MotorSpeed Client: waiting softhand to grasp object ... ...', 'blue'))

        while not self.object_grasped:
            pass        

        self.get_logger().info(termcolor.colored(f'MotorSpeed Client: object grasped, start PID speed controller for motors ... ...', 'blue'))

        wave_speed_array = [1.0, 1.0, 1.0, 1.0, 1.0]  # degrees
        wave_speed_array_r = [speed/180*numpy.pi for speed in wave_speed_array]
        self.sending_wave_speed(wave_speed_array_r)

        self.K_P = self.get_parameter('K_P').value
        self.K_I = self.get_parameter('K_I').value
        self.K_D = self.get_parameter('K_D').value     
        
        self.last_error = self.marker_angle_error
        self.PID_P = 0.0
        self.PID_I = 0.0
        self.PID_D = 0.0

        last_time = time.time()
        # PID should be updated at a regular interval
        sample_time = 0.05  # second

        PID_update_times = 0
        # wave speed is based on the PID controller
        while True:

            # pid speed controller for wave motors

            # Update PID only when the time interval is larger than the sample time interval
            current_time = time.time()
            delta_time = current_time - last_time

            if (delta_time >= sample_time):

                self.PID_controller_ready = True
                self.wave_speed = self.PID_update(self.marker_angle_error, delta_time)

                if abs(self.wave_speed) > 15:
                    self.wave_speed = 1
                    raise Exception(f'MotorSpeed Client: Finger wave speed is too fast, K_p is too large!')

                if (self.wave_speed < 0.1) and (self.wave_speed >= 0) :
                    self.wave_speed = 0.1
                elif (self.wave_speed > -0.1) and (self.wave_speed < 0):
                    self.wave_speed = -0.1
                
                wave_speed_r = float(abs(self.wave_speed)/180*numpy.pi)
                wave_speed_array_r = [wave_speed_r, wave_speed_r, wave_speed_r, wave_speed_r, wave_speed_r]

                self.sending_wave_speed(wave_speed_array_r)

                last_time = current_time

                if (PID_update_times % 10 == 0):
                    self.get_logger().info(termcolor.colored(f'MotorSpeed Client: PID speed controller updating for the motors, speed: {self.wave_speed}, times: {PID_update_times}', 'blue'))
                
                PID_update_times = PID_update_times + 1


    def PID_update(self, error, delta_time):

        delta_error = error - self.last_error

        '''Integral windup, also known as integrator windup or reset windup, refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change) and the integral terms accumulates a significant error during the 
        rise (windup), thus overshooting and continuing to increase as this accumulated error is unwound
        (offset by errors in the other direction). The specific problem is the excess overshooting.'''

        windup_guard = 2

        self.PID_P = self.K_P * error

        self.PID_I = self.PID_I + self.K_I * error * delta_time
        if self.PID_I <= -windup_guard:
            self.PID_I = -windup_guard
        elif self.PID_I >= windup_guard:
            self.PID_I = windup_guard

        self.PID_D = self.K_D * (delta_error / delta_time)

        self.last_error = error

        output = self.PID_P + self.PID_I + self.PID_D

        return output


    def sending_bend_speed(self, bend_speed_array_r):
        
        fingerbend_2motorID = [3, 4, 5, 6, 1]
        
        for i in range(5):
            req = SetSpeed.Request()
            req.speed = bend_speed_array_r[i]
            motorID = fingerbend_2motorID[i]
            srv_name = 'dynamixel%s/set_speed' % str(motorID)
            self.future = self.client_array[motorID-1].call_async(req)
            #self.get_logger().info('Sending motor speed to server {}: {}'.format(srv_name, str(req.speed) ))
            
    def sending_wave_speed(self, wave_speed_array_r):
        
        fingerwave_2motorID = [2, 8, 7, 10, 9]
        
        for i in range(5):
            req = SetSpeed.Request()
            req.speed = wave_speed_array_r[i]
            motorID = fingerwave_2motorID[i]
            srv_name = 'dynamixel%s/set_speed' % str(motorID)
            self.future = self.client_array[motorID-1].call_async(req) 
            #self.get_logger().info('Sending motor speed to server {}: {}'.format(srv_name, str(req.speed) ))
    

    def camera_callback(self, msg):

        marker_angle =  msg.data
        
        if len(self.marker_angle_list) < 5:
            self.marker_angle_list.append(marker_angle)
            self.marker_angle_time_list.append(time.time())
        else:
            self.marker_angle_list = []
            self.marker_angle_time_list = []
        
        filename = '/home/zhanfeng/camera_ws/src/Realsense_python/vision_based_control/marker_angle_feedback/marker_angle.csv'

        if len(self.marker_angle_list) == 5:

            self.marker_angle_feedback = self.marker_angle_filter(self.marker_angle_list)
            self.marker_angle_error = self.marker_angle_desired - self.marker_angle_feedback
            
            # save marker angle feedback to csv file
            marker_angle_saved_list = []
            for i, marker_angle in enumerate(self.marker_angle_list):
                marker_angle_saved = [str(self.marker_angle_time_list[i]) , str(marker_angle)]
                marker_angle_saved_list.append(marker_angle_saved)
            #marker_angle_saved = [[str(marker_angle)] for marker_angle in self.marker_angle_list]

            with open(filename, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(marker_angle_saved_list)
            
            self.marker_feedback_update_times = self.marker_feedback_update_times + 1

        
            if (self.marker_feedback_update_times % 10 == 1):
                self.get_logger().info(termcolor.colored(f'The marker feedback angle is {self.marker_angle_feedback}, and the desired marker angle is {self.marker_angle_desired}', 'yellow'))
                self.get_logger().info(termcolor.colored(f'The marker angle error is {self.marker_angle_error}, update times: {self.marker_feedback_update_times}', 'yellow')) 


    def marker_angle_filter(self, marker_angle_list):

        angle_sum = sum(marker_angle_list)
        angle_filtered = angle_sum / len(marker_angle_list)
        return angle_filtered

        
    def motors_parameter_set(self):
    
        # get and process parameter:
        
        self.bend_angle_array_point0 = self.get_parameter('bend_angle_array_point0').value
        self.bend_angle_array_limit = self.get_parameter('bend_angle_array_limit').value
        self.wave_angle_array_point0 = self.get_parameter('wave_angle_array_point0').value
        self.wave_angle_array_limit1 = self.get_parameter('wave_angle_array_limit1').value
        self.wave_angle_array_limit2 = self.get_parameter('wave_angle_array_limit2').value
        
        self.bend_angle_array_r_point0 = self.process_angle_param(self.bend_angle_array_point0, 'bend')
        self.bend_angle_array_r_limit = self.process_angle_param(self.bend_angle_array_limit, 'bend')
        self.wave_angle_array_r_point0 = self.process_angle_param(self.wave_angle_array_point0, 'wave')
        self.wave_angle_array_r_limit1 = self.process_angle_param(self.wave_angle_array_limit1, 'wave')
        self.wave_angle_array_r_limit2 = self.process_angle_param(self.wave_angle_array_limit2, 'wave')
        

    def finger_move(self):

        #step 1: Initialization 
        self.get_logger().info(termcolor.colored('Motor Publisher: initializing motor... ...', 'green'))
        
        while not self.speed_preset:
            pass   

        #self.get_logger().info(termcolor.colored('Motor Publisher: Speed initialized, fingers moving to original point... ...', 'green'))
        #self.publishing_bend_angle(self.bend_angle_array_r_point0)
        #self.publishing_wave_angle(self.wave_angle_array_r_point0)
        #time.sleep(5)
        
        #step 2: Grasp object
        
        self.publishing_bend_angle(self.bend_angle_array_r_limit)
        #self.get_logger().info(termcolor.colored('Motor Publisher: Fingers bending to grasp the object', 'green'))
        time.sleep(3)

        self.object_grasped = True
        
        #step 3: Rotate object
        while not self.PID_controller_ready:
            pass

        self.get_logger().info(termcolor.colored('Motor Publisher: Rotating objects', 'green'))
        while True:

            if self.wave_speed > 0:
                self.get_logger().info(termcolor.colored('Rotating objects to right!', 'green'))
                self.publishing_wave_angle(self.wave_angle_array_r_limit1)

            elif self.wave_speed < 0:
                self.get_logger().info(termcolor.colored('Rotating objects to left!', 'green'))
                self.publishing_wave_angle(self.wave_angle_array_r_limit2)

            else:
                self.get_logger().info('Error is 0 now!!!!!!!!!!Stop!!!!!!!!!!!!')

            time.sleep(0.05)


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
            #self.get_logger().info('Publishing to {}: {}'.format(topic_name, str(msg.data) ))
            
    def publishing_wave_angle(self, wave_angle_array_r_point):
        
        fingerwave_2motorID = [2, 8, 7, 10, 9]
        for i in range(5):
            msg = Float64()
            msg.data = wave_angle_array_r_point[i]
            motorID = fingerwave_2motorID[i]
            topic_name = 'dynamixel%s/command' % str(motorID)
            self.publisher_array[motorID-1].publish(msg)   
            #self.get_logger().info('Publishing to {}: {}'.format(topic_name, str(msg.data) ))

        
def main(args=None):
    rclpy.init(args=args)

    softhand_manipulation_node = Softhand_Manipulation()

    # This creates a parallel thread of execution that will execute the `send_request` method of the client node. 
    # This is because I want the send request to run concurrently with the callbacks of the node.
    thread1 = Thread(target=softhand_manipulation_node.send_request)
    thread1.start()

    thread2 = Thread(target=softhand_manipulation_node.finger_move)
    thread2.start()

    # I am using a MultiThreadedExecutor here as I want all the callbacks to run on a different thread each 
    executor = MultiThreadedExecutor()
    executor.add_node(softhand_manipulation_node)
    executor.spin()

    softhand_manipulation_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
