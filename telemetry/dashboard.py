"""
telemetry/dashboard.py
======================
Real-time dashboard and performance visualization for robot trajectory following.

Features:
  - Real-time trajectory plotting (target vs actual)
  - Error evolution graphs
  - Speed/command evolution
  - Performance metrics (RMS error, success rate, latency)
  - Support for both matplotlib (static) and Plotly Dash (interactive)

Usage:
  from telemetry.dashboard import Dashboard, plot_static_results
  
  # After mission.follow() completes:
  dashboard = Dashboard(mission_name="pure_pursuit_test")
  dashboard.plot_from_telemetry(telemetry_obj, trajectory_data)
  
  # Or static plot:
  plot_static_results(robot_x, robot_y, target_x, target_y, errors)
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from typing import Optional, List, Tuple
from pathlib import Path
import json


class Dashboard:
    """Interactive dashboard for mission analysis."""
    
    def __init__(self, mission_name: str = "mission", output_dir: str = "results"):
        self.mission_name = mission_name
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True, parents=True)
        
        self.robot_path = None
        self.target_path = None
        self.errors = []
        self.velocities = []
        self.angular_velocities = []
        self.timestamps = []
        
    def load_from_telemetry(self, telemetry_obj):
        """Load data from MissionTelemetry object."""
        if hasattr(telemetry_obj, 'poses'):
            poses = telemetry_obj.poses
            if len(poses) > 0:
                self.robot_path = np.array(poses)
        
        if hasattr(telemetry_obj, 'errors'):
            self.errors = telemetry_obj.errors
            
        if hasattr(telemetry_obj, 'velocities'):
            self.velocities = telemetry_obj.velocities
            
        if hasattr(telemetry_obj, 'timestamps'):
            self.timestamps = telemetry_obj.timestamps
    
    def load_from_arrays(self, robot_path: np.ndarray, target_path: np.ndarray, 
                        errors: List[float], velocities: List[float] = None):
        """Manually load trajectory data."""
        self.robot_path = robot_path
        self.target_path = target_path
        self.errors = errors
        self.velocities = velocities if velocities is not None else []
    
    def plot_trajectories(self, show_grid: bool = True, show_error_heatmap: bool = False):
        """Plot target vs actual trajectory."""
        fig, ax = plt.subplots(figsize=(12, 10))
        
        # Target trajectory
        if self.target_path is not None:
            ax.plot(self.target_path[:, 0], self.target_path[:, 1], 
                   'g--', linewidth=2, label='Target trajectory', alpha=0.7)
            ax.plot(self.target_path[0, 0], self.target_path[0, 1], 
                   'go', markersize=10, label='Start')
            ax.plot(self.target_path[-1, 0], self.target_path[-1, 1], 
                   'gs', markersize=10, label='Goal')
        
        # Robot actual trajectory
        if self.robot_path is not None:
            if show_error_heatmap and len(self.errors) == len(self.robot_path):
                # Color-coded by error magnitude
                scatter = ax.scatter(self.robot_path[:, 0], self.robot_path[:, 1],
                                   c=self.errors, cmap='RdYlGn_r', s=20, alpha=0.6,
                                   label='Robot path (colored by error)')
                cbar = plt.colorbar(scatter, ax=ax, label='Error [m]')
            else:
                ax.plot(self.robot_path[:, 0], self.robot_path[:, 1], 
                       'b-', linewidth=1.5, label='Robot path', alpha=0.8)
            
            ax.plot(self.robot_path[0, 0], self.robot_path[0, 1], 
                   'bo', markersize=8, alpha=0.6)
            ax.plot(self.robot_path[-1, 0], self.robot_path[-1, 1], 
                   'bs', markersize=8, alpha=0.6)
        
        ax.set_xlabel('X position [m]', fontsize=12)
        ax.set_ylabel('Y position [m]', fontsize=12)
        ax.set_title(f'Robot Trajectory - {self.mission_name}', fontsize=14, fontweight='bold')
        ax.legend(loc='best', fontsize=10)
        ax.grid(show_grid, alpha=0.3)
        ax.axis('equal')
        
        plt.tight_layout()
        filepath = self.output_dir / f"{self.mission_name}_trajectory.png"
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        print(f"✅ Saved: {filepath}")
        
        return fig
    
    def plot_errors(self):
        """Plot tracking error over time."""
        if not self.errors:
            print("⚠️  No error data to plot")
            return None
        
        fig, ax = plt.subplots(figsize=(12, 6))
        
        time_steps = np.arange(len(self.errors))
        ax.plot(time_steps, self.errors, 'r-', linewidth=1.5, label='Distance error')
        ax.fill_between(time_steps, 0, self.errors, alpha=0.3, color='red')
        
        # Add statistics
        mean_error = np.mean(self.errors)
        rms_error = np.sqrt(np.mean(np.array(self.errors)**2))
        max_error = np.max(self.errors)
        
        ax.axhline(mean_error, color='orange', linestyle='--', label=f'Mean: {mean_error:.4f} m')
        ax.axhline(rms_error, color='purple', linestyle='--', label=f'RMS: {rms_error:.4f} m')
        
        ax.set_xlabel('Time step', fontsize=12)
        ax.set_ylabel('Tracking error [m]', fontsize=12)
        ax.set_title(f'Tracking Error Evolution - {self.mission_name}\nMax: {max_error:.4f} m', 
                    fontsize=14, fontweight='bold')
        ax.legend(loc='best', fontsize=10)
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        filepath = self.output_dir / f"{self.mission_name}_errors.png"
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        print(f"✅ Saved: {filepath}")
        
        return fig
    
    def plot_velocities(self):
        """Plot linear and angular velocity commands over time."""
        if not self.velocities:
            print("⚠️  No velocity data to plot")
            return None
        
        velocities_array = np.array(self.velocities)
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
        
        time_steps = np.arange(len(self.velocities))
        
        if velocities_array.shape[1] >= 1:
            ax1.plot(time_steps, velocities_array[:, 0], 'b-', linewidth=1.5, label='v_cmd')
            ax1.fill_between(time_steps, 0, velocities_array[:, 0], alpha=0.3, color='blue')
            ax1.set_ylabel('Linear velocity [m/s]', fontsize=11)
            ax1.set_title(f'Velocity Commands - {self.mission_name}', fontsize=12, fontweight='bold')
            ax1.legend(loc='best')
            ax1.grid(True, alpha=0.3)
        
        if velocities_array.shape[1] >= 2:
            ax2.plot(time_steps, velocities_array[:, 1], 'g-', linewidth=1.5, label='ω_cmd')
            ax2.fill_between(time_steps, 0, velocities_array[:, 1], alpha=0.3, color='green')
            ax2.set_ylabel('Angular velocity [rad/s]', fontsize=11)
        
        ax2.set_xlabel('Time step', fontsize=11)
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        filepath = self.output_dir / f"{self.mission_name}_velocities.png"
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        print(f"✅ Saved: {filepath}")
        
        return fig
    
    def generate_report(self, metrics: dict = None):
        """Generate a summary report."""
        report = {
            "mission": self.mission_name,
            "statistics": {}
        }
        
        if self.errors:
            errors_array = np.array(self.errors)
            report["statistics"]["error"] = {
                "mean_m": float(np.mean(errors_array)),
                "rms_m": float(np.sqrt(np.mean(errors_array**2))),
                "max_m": float(np.max(errors_array)),
                "min_m": float(np.min(errors_array)),
                "std_m": float(np.std(errors_array))
            }
        
        if metrics:
            report["metrics"] = metrics
        
        # Save as JSON
        filepath = self.output_dir / f"{self.mission_name}_report.json"
        with open(filepath, 'w') as f:
            json.dump(report, f, indent=2)
        
        print(f"✅ Saved: {filepath}")
        print("\n📊 MISSION REPORT:")
        print("=" * 60)
        for key, val in report.items():
            if isinstance(val, dict):
                print(f"\n{key}:")
                for k, v in val.items():
                    if isinstance(v, dict):
                        print(f"  {k}:")
                        for k2, v2 in v.items():
                            print(f"    {k2}: {v2}")
                    else:
                        print(f"  {k}: {v}")
            else:
                print(f"{key}: {val}")
        print("=" * 60)
        
        return report
    
    def show_all(self):
        """Generate all plots."""
        self.plot_trajectories(show_error_heatmap=True)
        self.plot_errors()
        self.plot_velocities()
        plt.show()


def plot_static_results(robot_x: np.ndarray, robot_y: np.ndarray,
                       target_x: np.ndarray, target_y: np.ndarray,
                       errors: List[float],
                       title: str = "Robot Trajectory",
                       output_file: Optional[str] = None):
    """
    Simple static plot function for quick visualization.
    
    Args:
        robot_x, robot_y: Robot path coordinates
        target_x, target_y: Target path coordinates
        errors: Tracking errors
        title: Plot title
        output_file: Save to file if specified
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    
    # Trajectory
    ax = axes[0, 0]
    ax.plot(target_x, target_y, 'g--', linewidth=2, label='Target', alpha=0.7)
    ax.plot(robot_x, robot_y, 'b-', linewidth=1.5, label='Robot', alpha=0.8)
    ax.plot(robot_x[0], robot_y[0], 'go', markersize=10, label='Start')
    ax.plot(robot_x[-1], robot_y[-1], 'rs', markersize=10, label='End')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Trajectory')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # Error
    ax = axes[0, 1]
    ax.plot(errors, 'r-', linewidth=1.5)
    ax.fill_between(range(len(errors)), 0, errors, alpha=0.3, color='red')
    rms = np.sqrt(np.mean(np.array(errors)**2))
    ax.axhline(rms, color='purple', linestyle='--', label=f'RMS: {rms:.4f}m')
    ax.set_xlabel('Time step')
    ax.set_ylabel('Error [m]')
    ax.set_title('Tracking Error')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Error histogram
    ax = axes[1, 0]
    ax.hist(errors, bins=30, color='orange', alpha=0.7, edgecolor='black')
    ax.set_xlabel('Error [m]')
    ax.set_ylabel('Frequency')
    ax.set_title('Error Distribution')
    ax.grid(True, alpha=0.3, axis='y')
    
    # Statistics
    ax = axes[1, 1]
    ax.axis('off')
    stats_text = f"""
    STATISTICS
    {'='*40}
    Mean Error:     {np.mean(errors):.4f} m
    RMS Error:      {rms:.4f} m
    Max Error:      {np.max(errors):.4f} m
    Min Error:      {np.min(errors):.4f} m
    Std Dev:        {np.std(errors):.4f} m
    
    Path Length:    {len(robot_x)} points
    Total Distance: {np.sum(np.sqrt(np.diff(robot_x)**2 + np.diff(robot_y)**2)):.2f} m
    """
    ax.text(0.1, 0.5, stats_text, fontsize=11, family='monospace',
           verticalalignment='center', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.suptitle(title, fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"✅ Saved: {output_file}")
    
    return fig
