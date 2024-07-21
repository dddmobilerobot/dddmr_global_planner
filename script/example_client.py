#! /usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from dddmr_sys_core.action import GetPlan


class ExampleGetPlan(Node):

    def __init__(self):
        super().__init__('example_get_plan_client')
        self.get_logger().info("Starting example get plan client")
        self._action_client = ActionClient(self, GetPlan, 'get_plan')

    def send_goal(self):

        self.get_logger().info("Sending Goal")

        goal_msg = GetPlan.Goal()

        goal_msg.goal.header.frame_id = "map"
        goal_msg.goal.pose.position.x = -93.1
        goal_msg.goal.pose.position.y = -49.3
        goal_msg.goal.pose.position.z = 5.42
        goal_msg.goal.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.get_logger().info("Goal sent. Waiting response")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        
        if(future.result().status == 4):
            self.get_logger().info('Goal status: Succeed')
        elif(future.result().status == 6): 
            self.get_logger().info('Goal status: ABORTED')
        else:
            self.get_logger().info('Goal status: ' + str(future.result().status))
        
        result = future.result().result
        #TODO check result().status also
        #self.get_logger().info(result)
        
        rclpy.shutdown()
        

def main(args=None):
    rclpy.init(args=args)
        
    action_client = ExampleGetPlan()
    
    
    #future = action_client.send_goal(False) #Get status without opening
    future = action_client.send_goal()  #Send an 'open' command, and then get status
        
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()