"""
Simulateur visuel du robot
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.robot_base import RobotSimulator
from src.localizer import Localizer
from src.trajectory_follower import TrajectoryFollower


class RobotVisualizer:
    """Visualise la simulation du robot"""
    
    def __init__(self, trajectory: np.ndarray = None):
        self.trajectory = trajectory
        self.robot_path = []
        self.errors = []
        self.time_steps = []
        self.time = 0
        
        # Crée la figure
        self.fig, (self.ax_main, self.ax_error) = plt.subplots(1, 2, figsize=(14, 6))
        
        # Configuration
        self.ax_main.set_xlim(-0.5, 1.5)
        self.ax_main.set_ylim(-0.5, 1.5)
        self.ax_main.set_aspect('equal')
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.set_xlabel("X (m)")
        self.ax_main.set_ylabel("Y (m)")
        self.ax_main.set_title("Robot Trajectory")
        
        # Affiche trajectoire cible
        if trajectory is not None:
            self.ax_main.plot(trajectory[:, 0], trajectory[:, 1], 'b--',
                            linewidth=2, label='Target', alpha=0.7)
            self.ax_main.plot(trajectory[0, 0], trajectory[0, 1], 'go',
                            markersize=10, label='Start')
        
        # Lignes pour le chemin réel
        self.real_path_line, = self.ax_main.plot([], [], 'r-', linewidth=2, label='Actual')
        self.robot_pos, = self.ax_main.plot([], [], 'ro', markersize=8)
        
        self.ax_main.legend()
        
        # Graphe erreur
        self.error_line, = self.ax_error.plot([], [], 'r-', linewidth=2)
        self.ax_error.set_xlabel("Time (s)")
        self.ax_error.set_ylabel("Error (m)")
        self.ax_error.grid(True)
    
    def add_point(self, x, y, theta, error=0):
        """Ajoute un point"""
        self.robot_path.append([x, y, theta])
        self.errors.append(error)
        self.time_steps.append(self.time)
        self.time += 0.01
    
    def update_plot(self):
        """Met à jour la visualisation"""
        if len(self.robot_path) == 0:
            return
        
        path = np.array(self.robot_path)
        
        # Chemin réel
        self.real_path_line.set_data(path[:, 0], path[:, 1])
        
        # Position du robot
        self.robot_pos.set_data([path[-1, 0]], [path[-1, 1]])
        
        # Erreurs
        if self.errors:
            self.error_line.set_data(self.time_steps, self.errors)
            self.ax_error.set_xlim(0, max(self.time_steps) + 1)
            self.ax_error.set_ylim(0, max(self.errors) + 0.1)
        
        self.fig.canvas.draw_idle()
    
    def show(self):
        """Affiche"""
        plt.show()
    
    def save(self, filename):
        """Sauvegarde"""
        self.fig.savefig(filename, dpi=150)
        print(f"📊 Saved: {filename}")


def run_full_simulation():
    """Simule le robot complet"""
    
    print("\n🎮 FULL ROBOT SIMULATION")
    print("="*60)
    
    # Crée les composants
    robot = RobotSimulator()
    localizer = Localizer()
    follower = TrajectoryFollower(robot, localizer)
    
    robot.connect()
    
    # Crée une trajectoire carrée
    trajectory = np.array([
        [0.0, 0.0, 0.0],
        [0.25, 0.0, 0.0],
        [0.5, 0.0, 0.0],
        [0.75, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 0.25, 1.57],
        [1.0, 0.5, 1.57],
        [1.0, 0.75, 1.57],
        [1.0, 1.0, 1.57],
        [0.75, 1.0, 3.14],
        [0.5, 1.0, 3.14],
        [0.25, 1.0, 3.14],
        [0.0, 1.0, 3.14],
        [0.0, 0.75, -1.57],
        [0.0, 0.5, -1.57],
        [0.0, 0.25, -1.57],
        [0.0, 0.0, 0.0],
    ])
    
    follower.load_trajectory(trajectory)
    
    # Crée le visualizer
    visualizer = RobotVisualizer(trajectory)
    
    print(f"📋 Trajectory: {len(trajectory)} waypoints")
    print("🚀 Starting simulation...\n")
    
    # Simulation
    step = 0
    max_steps = 500
    
    while not follower.trajectory_complete and step < max_steps:
        
        waypoint = follower.get_current_waypoint()
        if waypoint is None:
            break
        
        x_target, y_target, theta_target = waypoint
        
        # Simule le robot
        robot.update()
        x, y, theta = robot.get_pose()
        
        # Erreur
        dx = x_target - x
        dy = y_target - y
        dist_error = np.sqrt(dx**2 + dy**2)
        
        # PID
        angle_target = np.arctan2(dy, dx)
        angle_error = angle_target - theta
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        
        v_cmd = follower.pid_linear.update(dist_error, dt=0.01)
        omega_cmd = follower.pid_angular.update(angle_error, dt=0.01)
        
        # Moteurs
        v_left = v_cmd - (localizer.config.WHEEL_BASE / 2) * omega_cmd
        v_right = v_cmd + (localizer.config.WHEEL_BASE / 2) * omega_cmd
        
        max_v = max(abs(v_left), abs(v_right), 1e-6)
        if max_v > 1.0:
            v_left /= max_v
            v_right /= max_v
        
        robot.set_motor_speed(v_left, v_right)
        
        # Enregistre
        visualizer.add_point(x, y, theta, dist_error)
        
        # Waypoint atteint?
        if (dist_error < follower.config.LINEAR_TOLERANCE and
            abs(angle_error) < follower.config.ANGULAR_TOLERANCE):
            follower.advance_waypoint()
            print(f"✅ Waypoint {follower.current_waypoint_idx-1} reached")
        
        if step % 50 == 0:
            print(f"   Step {step:3d}: pos=({x:.2f}, {y:.2f}), error={dist_error:.3f}m")
        
        step += 1
    
    robot.disconnect()
    
    # Résultats
    print("\n" + "="*60)
    print("📊 SIMULATION RESULTS")
    print("="*60)
    print(f"Steps executed: {step}")
    print(f"Waypoints reached: {follower.current_waypoint_idx}/{len(trajectory)}")
    
    if visualizer.errors:
        errors = np.array(visualizer.errors)
        print(f"Error mean: {errors.mean():.4f}m ({errors.mean()*100:.2f}cm)")
        print(f"Error max: {errors.max():.4f}m ({errors.max()*100:.2f}cm)")
    
    print("="*60 + "\n")
    
    # Visualise
    visualizer.update_plot()
    visualizer.save("simulation_result.png")
    visualizer.show()


if __name__ == "__main__":
    run_full_simulation()
