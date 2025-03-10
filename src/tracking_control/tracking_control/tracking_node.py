import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import TransformException, Buffer, TransformListener
import numpy as np
import math

## Functions for quaternion and rotation matrix conversion
## The code is adapted from the general_robotics_toolbox package
## Code reference: https://github.com/rpiRobotics/rpi_general_robotics_toolbox_py
def hat(k):
    """
    Returns a 3 x 3 cross product matrix for a 3 x 1 vector

             [  0 -k3  k2]
     khat =  [ k3   0 -k1]
             [-k2  k1   0]

    :param   k: 3 x 1 vector
    :return: the 3 x 3 cross product matrix
    """
    khat = np.zeros((3,3))
    khat[0,1] = -k[2]
    khat[0,2] = k[1]
    khat[1,0] = k[2]
    khat[1,2] = -k[0]
    khat[2,0] = -k[1]
    khat[2,1] = k[0]
    return khat

def q2R(q):
    """
    Converts a quaternion into a 3 x 3 rotation matrix using the Euler-Rodrigues formula.
    
    :param   q: 4 x 1 vector representation of a quaternion q = [q0, q1, q2, q3]
    :return: the 3x3 rotation matrix    
    """
    I = np.identity(3)
    qhat = hat(q[1:4])
    qhat2 = qhat.dot(qhat)
    return I + 2*q[0]*qhat + 2*qhat2

def euler_from_quaternion(q):
    w = q[0]
    x = q[1]
    y = q[2]
    z = q[3]
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(2*(w*y - z*x))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return [roll, pitch, yaw]

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        self.get_logger().info('Tracking Node Started')
        
        # Current object and goal pose (in world frame)
        self.obs_pose = None
        self.goal_pose = None
        
        # ROS parameters
        self.declare_parameter('world_frame_id', 'odom')

        # Create a transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create publisher for the control command
        self.pub_control_cmd = self.create_publisher(Twist, '/track_cmd_vel', 10)
        # Create subscribers to the detected poses
        self.sub_detected_goal_pose = self.create_subscription(
            PoseStamped, 'detected_color_object_pose', self.detected_obs_pose_callback, 10)
        self.sub_detected_obs_pose = self.create_subscription(
            PoseStamped, 'detected_color_goal_pose', self.detected_goal_pose_callback, 10)

        # Create timer, running at 100Hz
        self.timer = self.create_timer(0.01, self.timer_update)
    
    def detected_goal_pose_callback(self, msg):
        #self.get_logger().info('Received Detected Object Pose')
        
        odom_id = self.get_parameter('world_frame_id').get_parameter_value().string_value
        center_points = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        # TODO: Filtering
        # You can decide to filter the detected object pose here
        # For example, you can filter the pose based on the distance from the camera
        # or the height of the object
        # if np.linalg.norm(center_points) > 3 or center_points[2] > 0.7:
        #     return
        
        try:
            # Transform the center point from the camera frame to the world frame
            transform = self.tf_buffer.lookup_transform(odom_id,msg.header.frame_id,rclpy.time.Time(),rclpy.duration.Duration(seconds=0.1))
            t_R = q2R(np.array([transform.transform.rotation.w,transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z]))
            cp_world = t_R@center_points+np.array([transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z])
        except TransformException as e:
            self.get_logger().error('Transform Error: {}'.format(e))
            return
        
        # Get the detected object pose in the world frame
        self.goal_pose = cp_world

    def detected_obs_pose_callback(self, msg):
        #self.get_logger().info('Received Detected Object Pose')
        
        odom_id = self.get_parameter('world_frame_id').get_parameter_value().string_value
        center_points = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        # TODO: Filtering
        # You can decide to filter the detected object pose here
        # For example, you can filter the pose based on the distance from the camera
        # or the height of the object
        # if np.linalg.norm(center_points) > 3 or center_points[2] > 0.7:
        #     return
        
        try:
            # Transform the center point from the camera frame to the world frame
            transform = self.tf_buffer.lookup_transform(odom_id,msg.header.frame_id,rclpy.time.Time(),rclpy.duration.Duration(seconds=0.1))
            t_R = q2R(np.array([transform.transform.rotation.w,transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z]))
            cp_world = t_R@center_points+np.array([transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z])
        except TransformException as e:
            self.get_logger().error('Transform Error: {}'.format(e))
            return
        
        # Get the detected object pose in the world frame
        self.obs_pose = cp_world
        
    def get_current_poses(self):

        odom_id = self.get_parameter('world_frame_id').get_parameter_value().string_value
        try:
            transform = self.tf_buffer.lookup_transform('base_footprint', odom_id, rclpy.time.Time())
            robot_trans = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z])
            robot_R = q2R([
                transform.transform.rotation.w,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z])
            # Transform poses into the robot frame
            obstacle_pose = None
            goal_pose = None
            if self.obs_pose is not None:
                obstacle_pose = robot_R @ self.obs_pose + robot_trans
            if self.goal_pose is not None:
                goal_pose = robot_R @ self.goal_pose + robot_trans
        except TransformException as e:
            self.get_logger().error('Transform error: ' + str(e))
            return None, None
        
        return obstacle_pose, goal_pose
    
    def timer_update(self):
        
        if self.goal_pose is None:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.pub_control_cmd.publish(cmd_vel)
            return
        
        current_obs_pose, current_goal_pose = self.get_current_poses()
        if current_goal_pose is None:
            return
        
        cmd_vel = self.controller(current_obs_pose, current_goal_pose)
        self.pub_control_cmd.publish(cmd_vel)
    
    def controller(self, current_obs_pose, current_goal_pose):
        cmd_vel = Twist()

        goal_vec = current_goal_pose[:2]
        dist_to_goal = np.linalg.norm(goal_vec)
        
        if dist_to_goal < 0.3:
            return cmd_vel  
        
        k_attract = 1.0 
        k_repls = 0.5
        k_linear = 0.2
        k_ang = 1.0
        d_obs_thres = 0.5

        F_attract = k_attract * goal_vec
        
        F_repls = np.array([0.0, 0.0])
        if current_obs_pose is not None:
            obs_vec = current_obs_pose[:2]
            d_obs = np.linalg.norm(obs_vec)
            if d_obs < d_obs_thres and d_obs > 1e-6:
                F_repls = k_repls * (1.0/d_obs - 1.0/d_obs_thres) * (1.0/(d_obs**2)) * (-obs_vec/ d_obs)
        
        F_total = F_attract + F_repls
        
        desired_angle = math.atan2(F_total[1], F_total[0])
        angle_error = desired_angle
        
        lin_sped = k_linear * np.linalg.norm(F_total)
        lin_sped *= math.cos(angle_error)
        lin_sped = max(0.0, lin_sped)
        lin_sped = min(lin_sped, 0.5)
        
        ang_sped = k_ang * angle_error
        ang_sped = max(-0.5, min(ang_sped, 0.5))
        
        cmd_vel.linear.x = lin_sped
        cmd_vel.angular.z = ang_sped
        
        return cmd_vel

def main(args=None):
    rclpy.init(args=args)
    tracking_node = TrackingNode()
    rclpy.spin(tracking_node)
    tracking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
