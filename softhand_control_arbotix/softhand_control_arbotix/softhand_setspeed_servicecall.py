from arbotix_msgs.srv import Enable, SetSpeed, Relax

import rclpy
import numpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException

class Softhand_SetSpeed_ClientAsync(Node):

    def __init__(self):
        super().__init__('softhand_setspeed_client_async')    #node name
        
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
        
        self.declare_parameter('bend_speed_array')    #in degrees/sec
        self.declare_parameter('wave_speed_array')    #in degrees/sec
            
    def create_client_i(self, client_num):
        srv_name = 'dynamixel%s/set_speed' % str(client_num)
        client = self.create_client(SetSpeed, srv_name)
        
        # check if service is connected.
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service {} not available, waiting again...'.format(client_num) )
            
        self.get_logger().info('server {} is connected to client {}'.format(client_num, client_num) )
        
        return client

    def send_request(self):
        bend_speed_array = self.get_parameter('bend_speed_array').value    #get parameter in degrees/sec
        wave_speed_array = self.get_parameter('wave_speed_array').value
        bend_speed_array_r = [x/180*numpy.pi for x in bend_speed_array]    #in radians/sec
        wave_speed_array_r = [x/180*numpy.pi for x in wave_speed_array]
        
        self.get_logger().info('Sending speed info to motors... ...')
        
        self.sending_bend_speed(bend_speed_array_r)
        self.sending_wave_speed(wave_speed_array_r)
        
        self.get_logger().info('Finish sending speed to motors.')
        
        
    def sending_bend_speed(self, bend_speed_array_r):     
        fingerbend_2motorID = [3, 4, 5, 6, 1]
        
        for i in range(5):
            req = SetSpeed.Request()
            req.speed = bend_speed_array_r[i]
            motorID = fingerbend_2motorID[i]
            srv_name = 'dynamixel%s/set_speed' % str(motorID)
            self.future = self.client_array[motorID-1].call_async(req)
            self.get_logger().info('Sending to server {}: {}'.format(srv_name, str(req.speed) ))
            
    def sending_wave_speed(self, wave_speed_array_r):
        fingerwave_2motorID = [2, 8, 7, 10, 9]
        
        for i in range(5):
            req = SetSpeed.Request()
            req.speed = wave_speed_array_r[i]
            motorID = fingerwave_2motorID[i]
            srv_name = 'dynamixel%s/set_speed' % str(motorID)
            self.future = self.client_array[motorID-1].call_async(req) 
            self.get_logger().info('Sending to server {}: {}'.format(srv_name, str(req.speed) ))


def main(args=None):
    rclpy.init(args=args)

    softhand_setspeed_client = Softhand_SetSpeed_ClientAsync()
    softhand_setspeed_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(softhand_setspeed_client)    #spin the node once
        if softhand_setspeed_client.future.done():
            try:
                response = softhand_setspeed_client.future.result()
            except Exception as e:
                softhand_setspeed_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                softhand_setspeed_client.get_logger().info(
                    'service call success, motor speed set.' )
            break

    softhand_setspeed_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
