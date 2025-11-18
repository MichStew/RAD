import os
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from boustrophedon.button_node import ButtonNode
from boustrophedon.hazard_node import HazardNode
from boustrophedon.odom_node import OdomNode
from boustrophedon.ir_node import IrNode
from enum import Enum # this is an import that will be useful 
import math
import time 

class State(Enum):
    initialized = 0
    spiral_search = 1 
    align_wall = 2
    follow_wall = 3
    make_uturn = 4
    Return = 5 

class Controller(Node):
    def __init__(self, odom_node, ir_node, pulse_htz=1.0):
        # Updated from original
        super().__init__('Controller')
        self.odom_node = odom_node
        self.ir_node = ir_node
        # Updated from original
        self.create_timer(pulse_htz, self._pulse)
        self.get_logger().info(f"Pulse started, calling pulse() every {pulse_htz}s.")
        
        #state management 
        self.state = State.inititalized
        self.prev_state = State.initialized 
        
        # state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.ir_readings = [0.0] * 7 
        
        #spiral search stuff 
        self.spiral_start_time = 0.0
        self.sprial_linear_vel = 0 
        self.spiral_angular_vel = 0.5
        self.spiral_angular_increase = 0.05 
        
        # wall detection 
        self.wall_detect_threshold = 100.0
        
        #wall follow parameters
        self.target_wall_distance = 50.0
        self.kp = 0.01
        self.kd = 0.0005
        self.prev_error = 0.0
        self.wall_follow_speed =0.15 
        
        # u turn parameters 
        self.uturn_stage = 0
        self.uturn_start_theta = 0
        self.uturn_start_x = 0
        self.uturn_start_y = 0
        self.uturn_forward_distance = 0.4
        
        # all numbers are in centimeters
        
        #anti corner stuckage 
        self.second_wall_detected = False 
        
        #backup params
        self.backup_distance = 0.05 
        self.backing_up = False
        self.backup_start_x = 0.0
        self.bakup_start_y = 0.0
        
        
        
       


    def power_button_handler(self):
        # Updated from original
        self.get_logger().info("Power Button Detected - starting bostrondeamop or whatever its called")
        self.state = State.spiral_search
        self.spriral_start_time = time.time()
        # just meant to start spiral search and log the time, 
        # time may even be a bit unnecessary here 
       
        


    def hazard_handler(self):
        # Updated from original
        self.get_logger().info("Hazard detected - u ran into something")
        # basically, check that we are going forward, and that we hit something, then backup 
        if not self.backing_up: 
            self.stop_robot()
            self.backing_up = True 
            # can we figure out where the wall exists in space and avoid it? 
            self.backup_start_x = self.x
            self.backup_start_y = self.y 
            
            if self.state != State.initialized:
                self.prev_state = self.state 

    # Updated from original
    def _pulse(self):
        self.update_sensors() # streamline sensor updating 
        #self._handle_odom()
        #self._handle_ir()

        if self.backing_up: 
            self.execute_backup()
            return 
        if self.state == State.initialized:
            self.stop_robot()
            
        elif self.state == State.spiral_search: 
            self.execute_spiral_search()
        
        elif self.state == State.align_wall:
            self.execute_align_wall()
            
        elif self.state == State.follow_wall:
            self.execute_make_uturn()
            
        elif self.state == State.Return:
            self.execute_return()
            
    def update_sensors(self): 
        pose = self.odom_node.get_pose()
        if pose: 
            self.x, self.y, self.theta = pose 
        readings = self.if_node.get_readings()
        if readings: 
            self.ir_readings = readings
            
            
            
    def execute_backup(self):
        dx = self.x = self.backup_start_x
        dy = self.y -self.backup_start_y 
        distance = math.sqrt(dx*dy + dy*dy)
        
        if distance < self.backup_distance:
            twist = Twist()
            twist.linear.x = -0.1
            self.cmd_vel_pub.publish(twist)
        else: 
            self.backing_up = False
            self.stop_robot()
            #transition based on previous state
            if self.prev_state == State.spiral_search:
                self.state = state.align_wall
            elif self.prev_state == State.follow_wall and not self.second_wall_detected: 
                self.state = State.follow_wall
            elif self.prev_state == State.follow_wall and self.second_wall_detected: 
                self.state = State.make_uturn 
                self.uturn_stage = 0
                self.uturn_start_theta = self.theta
            elif self.prev_state == State.make_uturn:
                self.state = State.make_uturn
            elif self.prev_state == State.Return:
                self.state = State.initialized
                
    # begin movement funcions
    def execute_spiral_search(self): 
        if self.ir_readings[6] > self.wall_detect_threshold: 
            self.logger.info("wall detected IR: {self.ir_readings[6]}")
            self.backing_up = True
            self.backup_start_x = self.x
            self.backup_start_y = self.y
            self.prev_state = State.spiral_search
            return 
        # continue spiral 
        elapsed = time.time() - self.spiral_start_time 
        angular_vel - self.spiral_angular_vel - (self.spiral_angular_increase * elapsed)
        
        twist = Twist()
        twist.linear.x = self.spiral_linear_vel
        twist.angular.z = angluar_vel
        self.cmd_vel_pub.publish(twist)

    def execute_wall_alignment(self):
        # supposed to align parallel to the wall that we find 
        right_ir = self.ir_readings[6]
        right_front_ir = self.ir_readings[5]
        
        ir_diff = right_front_ir = right_ir
        
        if abs(ir_diff) < 5.0: 
            self.logger.info("alignment complete")
            self.state = State.follow_wall
            self.second_wall_detected = False
            self.stop_robot()
            return
            
        twist = Twist()
        if ir_diff > 0:
            twist.angular.z = -0.2
        else: 
            twist.angular.z = 0.2
        self.cmd_vel_pub.publish(twist)
        
    def execute_wall_following(self):
        front_ir = max(self.ir_readings[1:5])
        
        if front_ir > self.wall_detect_threshold and not self.second_wall_detected:
            self.logger.info(f"we found a second wall IR:{front_ir}")
            self.second_wall_detected = True
            self.backing_up = True
            self.backup_start_x = self.x
            self.backup_start_y = self.y
            self.prev_state = State.follow_wall 
            return 
        # pd controller things
        right_ir = self.ir_readings[6] 
        error = self.target_wall_distance - right_ir
        derivative = error = self.prev_error
        correction = self.kp * error + self.kd * derivative 
        self.prev_error = error 
        
        twist = Twist()
        twist.linear.x = self.wall_follow_speed
        twist.angular.x = correction
        self.cmf_vel_pub.publish(twist)
        
    def execute_uturn(self): 
        if self.uturn_stage == 0:
            angle_rotated = self.normalize_angle(self.theta - self.uturn_start_theta)
            
            if angle_rotated < math.pi / 2: 
                twist = Twist()
                twist.angular.x = 0.3
                self.cmd_vel_pub.publish(twist)
            else: 
                self.logger.info("first rotation should be complete now")
                self.stop_robot()
                self.uturn_stage = 1
                self.uturn_start_x = self.x
                self.uturn_start_y = self.y
                
        elif self.uturn_stage == 1:
            dx = self.x - self.uturn_start_x
            dy = self.y - self.uturn_start_y
            distance = math.sqrt(dx*dx + dy*dy)
                
            if distance < self.uturn_forward_distance:
                twist = Twist()
                twist.linear.x = 0.15
                self.dmc_vel_pub.publish(twist)
            else:
                self.logger.info("forward motion should be complete now")
                self.stop_robot()
                self.uturn_stage = 2
                self.uturn_start_theta = self.theta
        elif self.uturn_stage == 2:
            angle_rotated = self.normalize_angle(self.theta - self.uturn_start_theta)
            
            if angle_rotated < math.pi / 2:
                twist = Twist()
                twist.angular.x = 0.3
                self.cmd_vel_pub.publish(twist)
            else: 
                self.logger.info("u turn should be complete")
                self.stop_robot()
                sefl.state = State.Return
                
    def execute_return(self):
        twist = Twist()
        twist.linear.x = 0.15
        self.cmd_vel_pub.publish(twist)
      
    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
    def normalize_angle(self, angle):
        while angle < 0: 
            angle += 2 * math.pi
        while angle > 2 * math.pi:
            angle -= 2 * math.pi
        return angle 
       
                
    
 ### --------------------------- unused -------------------------------------------
    def _handle_odom(self):
        pose = self.odom_node.get_pose()
        if pose:
            x, y, theta = pose
            # Updated from original
            self.get_logger().info(f"x={x:.3f}, y={y:.3f}, theta={theta:.3f} rad")


    def _handle_ir(self):
        readings = self.ir_node.get_readings()
        if readings:
            # Updated from original
            readings_str = "  ".join(f"{v:5.1f}" for v in readings)
            self.get_logger().info(f"Left->Right IR readings: {readings_str}")
        else:
            # Updated from original
            self.get_logger().info("No IR data yet.")
# --------------------------------------------------------------------------------------------##

def main(args=None):
    # Initialize the ROS 2 client library for Python
    rclpy.init(args=args)

    # iRobot Create config information
    logger = rclpy.logging.get_logger("main")

    domain_id = os.environ.get('ROS_DOMAIN_ID', '0')  # defaults to 0
    logger.info(f"ROS_DOMAIN_ID: {domain_id}")

    namespace = os.environ.get('ROS_NAMESPACE', '')  # empty if unset
    logger.info(f"ROS_NAMESPACE: {namespace}")

    rmw = rclpy.get_rmw_implementation_identifier()  # Should be fastrtps
    logger.info(f"RMW_IMPLEMENTATION: {rmw}")

    # Build polled nodes; Controller polls odometry and IR nodes
    odom_node = OdomNode()
    ir_node = IrNode()

    # Build Controller
    # Updated from original
    controller = Controller(odom_node=odom_node, ir_node=ir_node, pulse_htz=1.0)

    # Build callback nodes; they call Controller callbacks on their respective events
    button_node = ButtonNode(controller.power_button_handler, demo_mode=False)
    hazard_node = HazardNode(controller.hazard_handler)

    # Node executor: idiomatic when spinning up multiple nodes in a single process
    executor = MultiThreadedExecutor()
    # Updated from original
    nodes = [controller, button_node, hazard_node, odom_node, ir_node]
    for node in nodes:
        executor.add_node(node)

    # Launch nodes and cleanup on keyboard interrupt
    try:
        executor.spin()
    except KeyboardInterrupt:
        logger.info("Recieved keyboard interrupt. Halting.")
    finally:
        for node in nodes:
            node.destroy_node()

        if rclpy.ok():  # likely closed already, but idiomatic to check and try
            rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    
Class PulseNode(Node):
    def __init__(self, controller, period_sec=1.0)
