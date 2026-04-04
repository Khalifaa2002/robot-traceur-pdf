"""
ros_trajectory_node.py
Nœud ROS 2 qui publie la trajectoire générée
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
import numpy as np
import os
from math import cos, sin

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        # Crée les publishers
        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)
        self.waypoints_publisher = self.create_publisher(PoseStamped, '/waypoint', 10)
        
        # Timer pour publier périodiquement
        self.timer = self.create_timer(2.0, self.publish_trajectory)
        
        # Charge la trajectoire
        self.trajectory = self.load_trajectory()
        
        self.get_logger().info('🤖 Nœud Robot Traceur de Plan PDF démarré!')
        if self.trajectory is not None:
            self.get_logger().info(f'✅ Trajectoire chargée: {len(self.trajectory)} points')
        else:
            self.get_logger().error('❌ Erreur lors du chargement de la trajectoire')
    
    def load_trajectory(self):
        """Charge la trajectoire depuis le fichier NPY"""
        trajectory_file = os.path.join(
            os.path.dirname(__file__),
            '../data/trajectory.npy'
        )
        
        if os.path.exists(trajectory_file):
            trajectory = np.load(trajectory_file)
            self.get_logger().info(f"📂 Trajectoire chargée depuis: {trajectory_file}")
            return trajectory
        else:
            self.get_logger().error(f"❌ Fichier de trajectoire non trouvé: {trajectory_file}")
            return None
    
    def angle_to_quaternion(self, angle):
        """Convertit un angle (rad) en quaternion"""
        # Pour une rotation 2D autour de Z
        half_angle = angle / 2.0
        qx = 0.0
        qy = 0.0
        qz = sin(half_angle)
        qw = cos(half_angle)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
    
    def publish_trajectory(self):
        """Publie la trajectoire complète"""
        if self.trajectory is None:
            return
        
        # Crée un message Path
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Ajoute chaque point de la trajectoire
        for i, point in enumerate(self.trajectory):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.seq = i
            
            # Position
            pose_stamped.pose.position = Point(
                x=float(point[0]),
                y=float(point[1]),
                z=0.0
            )
            
            # Orientation (angle en 2D)
            theta = float(point[2])
            pose_stamped.pose.orientation = self.angle_to_quaternion(theta)
            
            path_msg.poses.append(pose_stamped)
        
        # Publie
        self.path_publisher.publish(path_msg)
        self.get_logger().info(f"📤 Trajectoire publiée: {len(path_msg.poses)} poses")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()