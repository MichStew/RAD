import os
import rclpy
# import all the provided node code 
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from boustrophedon.button_node import ButtonNode
from boustrophedon.hazard_node import HazardNode
from boustrophedon.odom_node import OdomNode
from boustrophedon.ir_node import IrNode
import math
import time
from enum import Enum

# we enumerate our states for easy use 
class State(Enum):
    INITIALIZED = 0
    SPIRAL_SEARCH = 1
    ALIGN_WALL = 2
    FOLLOW_WALL = 3
    MAKE_UTURN = 4
    RETURN = 5


class Controller:
    def __init__(self, odom_node, ir_node, cmd_vel_publisher):
        self.logger = rclpy.logging.get_logger("Controller")
        self.odom_node = odom_node
        self.ir_node = ir_node
        self.cmd_vel_pub = cmd_vel_publisher
        
        # State management
        self.state = State.INITIALIZED
        self.prev_state = State.INITIALIZED
        
        # Robot state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.ir_readings = [0.0] * 7
        
        # Spiral search parameters
        self.spiral_start_time = 0.0
        self.spiral_linear_vel = 0.1  # m/s
        self.spiral_angular_vel = 0.5  # rad/s
        self.spiral_angular_increase = 0.05
        
        # Wall detection threshold
        self.wall_detect_threshold = 100.0
        
        # Wall following parameters
        self.target_wall_distance = 50.0
        self.kp = 0.01
        self.kd = 0.005
        self.prev_error = 0.0
        self.wall_follow_speed = 0.15
        self.max_linear_speed = 0.35
        self.max_angular_speed = 0.8
        
        # U-turn parameters
        self.uturn_stage = 0
        self.uturn_start_theta = 0.0
        self.uturn_start_x = 0.0
        self.uturn_start_y = 0.0
        self.uturn_forward_distance = 0.4  # 40 cm
        
        # Second wall detection
        self.second_wall_detected = False
        
        # Backup parameters
        self.backup_distance = 0.05  # 5 cm
        self.backing_up = False
        self.backup_start_x = 0.0
        self.backup_start_y = 0.0
        
        self.logger.info("Controller initialized and ready")

    def power_button_handler(self):
        if self.state == State.INITIALIZED:
            self.logger.info("Power button pressed - Starting spiral search")
            self.state = State.SPIRAL_SEARCH
            self.spiral_start_time = time.time()
        else:
            self.logger.info("Power button pressed - Resetting to initialized")
            self.state = State.INITIALIZED
            self.stop_robot()

    def hazard_handler(self):
        if not self.backing_up:
            self.logger.info(f"Hazard detected in state {self.state}")
            self.stop_robot()
            
            # Start backup maneuver
            self.backing_up = True
            self.backup_start_x = self.x
            self.backup_start_y = self.y
            
            # Store previous state
            if self.state != State.INITIALIZED:
                self.prev_state = self.state

            # Determine what type of collision we saw
            if self.state == State.FOLLOW_WALL:
                self.logger.info("Second wall hit - preparing for U-turn")
                self.second_wall_detected = True
            else:
                self.second_wall_detected = False

    def pulse(self):
        # Update sensor readings
        self.update_sensors()
        
        # Handle backup if in progress
        if self.backing_up:
            self.execute_backup()
            return
        
        # State machine
        if self.state == State.INITIALIZED:
            self.stop_robot()
        
        elif self.state == State.SPIRAL_SEARCH:
            self.execute_spiral_search()
        
        elif self.state == State.ALIGN_WALL:
            self.execute_wall_alignment()
        
        elif self.state == State.FOLLOW_WALL:
            self.execute_wall_following()
        
        elif self.state == State.MAKE_UTURN:
            self.execute_uturn()
        
        elif self.state == State.RETURN:
            self.execute_return()

    def update_sensors(self):
        pose = self.odom_node.get_pose()
        if pose:
            self.x, self.y, self.theta = pose
        
        readings = self.ir_node.get_readings()
        if readings:
            self.ir_readings = readings

    def execute_backup(self):
        dx = self.x - self.backup_start_x
        dy = self.y - self.backup_start_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < self.backup_distance:
            twist = Twist()
            twist.linear.x = -0.1
            self.cmd_vel_pub.publish(twist)
        else:
            self.backing_up = False
            self.stop_robot()
            self.logger.info("Backup complete")
            
            # Transition based on previous state
            if self.prev_state == State.SPIRAL_SEARCH:
                self.state = State.ALIGN_WALL
            elif self.prev_state == State.FOLLOW_WALL and not self.second_wall_detected:
                self.state = State.FOLLOW_WALL
            elif self.prev_state == State.FOLLOW_WALL and self.second_wall_detected:
                self.state = State.MAKE_UTURN
                self.uturn_stage = 0
                self.uturn_start_theta = self.theta
            elif self.prev_state == State.MAKE_UTURN:
                self.state = State.MAKE_UTURN
            elif self.prev_state == State.RETURN:
                self.state = State.INITIALIZED

    def execute_spiral_search(self):
        # Continue spiral
        elapsed = max(0.0, time.time() - self.spiral_start_time)
        angular_vel = self.spiral_angular_vel - (self.spiral_angular_increase * elapsed)
        angular_vel = max(-self.max_angular_speed, min(angular_vel, self.max_angular_speed))
        linear_vel = self.spiral_linear_vel + (self.spiral_linear_vel * elapsed)
        linear_vel = min(linear_vel, self.max_linear_speed)

        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)

    def execute_wall_alignment(self):
        """Align parallel to wall"""
        right_ir = self.ir_readings[6]
        right_front_ir = self.ir_readings[5]
        
        ir_diff = right_front_ir - right_ir
        
        if abs(ir_diff) < 5.0:
            self.logger.info("Alignment complete")
            self.state = State.FOLLOW_WALL
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
        # PD controller
        right_ir = self.ir_readings[6]
        error = self.target_wall_distance - right_ir
        derivative = error - self.prev_error
        correction = self.kp * error + self.kd * derivative
        self.prev_error = error
        
        twist = Twist()
        twist.linear.x = min(self.wall_follow_speed, self.max_linear_speed)
        twist.angular.z = max(-self.max_angular_speed, min(correction, self.max_angular_speed))
        self.cmd_vel_pub.publish(twist)

    def execute_uturn(self):
        if self.uturn_stage == 0:
            angle_rotated = self.normalize_angle(self.theta - self.uturn_start_theta)
            
            if angle_rotated < math.pi / 2:
                twist = Twist()
                twist.angular.z = min(0.3, self.max_angular_speed)
                self.cmd_vel_pub.publish(twist)
            else:
                self.logger.info("First rotation complete")
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
                twist.linear.x = min(0.15, self.max_linear_speed)
                self.cmd_vel_pub.publish(twist)
            else:
                self.logger.info("Forward motion complete")
                self.stop_robot()
                self.uturn_stage = 2
                self.uturn_start_theta = self.theta
        
        elif self.uturn_stage == 2:
            angle_rotated = self.normalize_angle(self.theta - self.uturn_start_theta)
            
            if angle_rotated < math.pi / 2:
                twist = Twist()
                twist.angular.z = min(0.3, self.max_angular_speed)
                self.cmd_vel_pub.publish(twist)
            else:
                self.logger.info("U-turn complete - Starting return")
                self.stop_robot()
                self.state = State.RETURN

    def execute_return(self):
        twist = Twist()
        twist.linear.x = min(0.15, self.max_linear_speed)
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


class PulseNode(Node):

    def __init__(self, controller, period_sec=1.0):
        super().__init__('pulse_node')
        self.controller = controller
        self.create_timer(period_sec, self._on_timer)
        self.get_logger().info(f"Pulse started, calling pulse() every {period_sec}s.")

    def _on_timer(self):
        self.controller.pulse()


def main(args=None):
    # Initialize the ROS 2 client library for Python
    rclpy.init(args=args)
    
    # iRobot Create config information
    logger = rclpy.logging.get_logger("main")
    domain_id = os.environ.get('ROS_DOMAIN_ID', '0')
    logger.info(f"ROS_DOMAIN_ID: {domain_id}")
    namespace = os.environ.get('ROS_NAMESPACE', '')
    logger.info(f"ROS_NAMESPACE: {namespace}")
    rmw = rclpy.get_rmw_implementation_identifier()
    logger.info(f"RMW_IMPLEMENTATION: {rmw}")
    
    # Create cmd_vel publisher node
    cmd_vel_node = Node('cmd_vel_publisher')
    cmd_vel_pub = cmd_vel_node.create_publisher(Twist, '/cmd_vel', 10)
    
    # Controller depends on odometry and IR
    odom_node = OdomNode()
    ir_node = IrNode()
    
    # Create Controller
    controller = Controller(odom_node=odom_node, ir_node=ir_node, cmd_vel_publisher=cmd_vel_pub)
    
    # Build callback nodes
    button_node = ButtonNode(controller.power_button_handler, demo_mode=False)
    hazard_node = HazardNode(controller.hazard_handler)
    
    # Node executor
    executor = MultiThreadedExecutor()
    nodes = [button_node, hazard_node, odom_node, ir_node, cmd_vel_node]
    for node in nodes:
        executor.add_node(node)
    
    # Pulse node for control loop
    pulse_node = PulseNode(controller, period_sec=0.1)
    executor.add_node(pulse_node)
    
    # Launch nodes and cleanup on keyboard interrupt
    try:
        executor.spin()
    except KeyboardInterrupt:
        logger.info("Received keyboard interrupt. Halting.")
    finally:
        for node in nodes:
            node.destroy_node()
        pulse_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
