"""
microcontroller_interface.py
Interface de communication avec le microcontrôleur (encodeurs + IMU)
"""

import serial
import json
import time
import numpy as np
from typing import Optional, Dict, Tuple
import threading

class MicrocontrollerInterface:
    def __init__(self, port='COM3', baudrate=115200, timeout=1.0):
        """
        Initialise la connexion avec le microcontrôleur
        
        Args:
            port: Port COM (ex: 'COM3' sur Windows, '/dev/ttyUSB0' sur Linux)
            baudrate: Vitesse de communication (par défaut 115200)
            timeout: Timeout de lecture (secondes)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.is_connected = False
        self.current_state = {
            'x': 0.0,      # Position X (mètres)
            'y': 0.0,      # Position Y (mètres)
            'theta': 0.0,  # Angle (radians)
            'v': 0.0,      # Vitesse linéaire (m/s)
            'omega': 0.0,  # Vitesse angulaire (rad/s)
            'encoder_left': 0,   # Compteur encodeur gauche
            'encoder_right': 0,  # Compteur encodeur droit
            'battery': 0.0       # Tension batterie (V)
        }
        self.lock = threading.Lock()
    
    def connect(self) -> bool:
        """Établit la connexion avec le microcontrôleur"""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(1)  # Attendre que l'Arduino redémarre
            self.is_connected = True
            print(f"✅ Connecté au microcontrôleur sur {self.port}")
            return True
        except serial.SerialException as e:
            print(f"❌ Erreur de connexion: {e}")
            print(f"   Vérifiez que le port {self.port} est correct")
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Ferme la connexion"""
        if self.serial is not None:
            self.serial.close()
            self.is_connected = False
            print("✅ Déconnecté du microcontrôleur")
    
    def send_command(self, command: str) -> bool:
        """
        Envoie une commande au microcontrôleur
        
        Args:
            command: Commande à envoyer (ex: "MOVE 0.5 1.57" pour v=0.5 m/s, theta=1.57 rad)
            
        Returns:
            True si succès, False sinon
        """
        if not self.is_connected:
            print("❌ Non connecté au microcontrôleur")
            return False
        
        try:
            # Ajoute le caractère de fin de ligne
            command_bytes = (command + "\n").encode()
            self.serial.write(command_bytes)
            print(f"📤 Commande envoyée: {command}")
            return True
        except Exception as e:
            print(f"❌ Erreur lors de l'envoi: {e}")
            return False
    
    def send_waypoint(self, x: float, y: float, theta: float, v: float = 0.5) -> bool:
        """
        Envoie un waypoint au microcontrôleur
        
        Args:
            x, y: Coordonnées cible (mètres)
            theta: Orientation cible (radians)
            v: Vitesse (m/s)
            
        Returns:
            True si succès
        """
        # Formate la commande JSON
        command = json.dumps({
            'cmd': 'GOTO',
            'x': round(x, 4),
            'y': round(y, 4),
            'theta': round(theta, 4),
            'v': round(v, 4)
        })
        
        return self.send_command(command)
    
    def send_motor_command(self, v: float, omega: float) -> bool:
        """
        Envoie une commande de moteur direct (contrôle bas niveau)
        
        Args:
            v: Vitesse linéaire (m/s)
            omega: Vitesse angulaire (rad/s)
            
        Returns:
            True si succès
        """
        command = json.dumps({
            'cmd': 'MOTOR',
            'v': round(v, 4),
            'omega': round(omega, 4)
        })
        
        return self.send_command(command)
    
    def read_state(self) -> Optional[Dict]:
        """
        Lit l'état du robot depuis le microcontrôleur
        
        Returns:
            Dictionnaire avec {x, y, theta, v, omega, encoder_left, encoder_right, battery}
        """
        if not self.is_connected:
            return None
        
        try:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode().strip()
                
                if line.startswith('{'):
                    # Reçoit un JSON
                    state = json.loads(line)
                    
                    with self.lock:
                        self.current_state.update(state)
                    
                    return state
        except json.JSONDecodeError:
            pass
        except Exception as e:
            print(f"❌ Erreur lors de la lecture: {e}")
        
        return None
    
    def get_current_state(self) -> Dict:
        """Retourne l'état courant du robot"""
        with self.lock:
            return self.current_state.copy()
    
    def start_continuous_reading(self):
        """Démarre une lecture continue en thread"""
        def read_loop():
            while self.is_connected:
                self.read_state()
                time.sleep(0.01)  # 100 Hz
        
        thread = threading.Thread(target=read_loop, daemon=True)
        thread.start()
        print("📡 Lecture continue démarrée")

class TrajectoryFollower:
    """Classe pour suivre une trajectoire avec correction"""
    
    def __init__(self, microcontroller: MicrocontrollerInterface):
        self.mc = microcontroller
        self.trajectory = None
        self.current_waypoint_idx = 0
        self.reached_tolerance = 0.05  # 5 cm
    
    def load_trajectory(self, trajectory_npy_path: str) -> bool:
        """Charge la trajectoire depuis un fichier NPY"""
        try:
            self.trajectory = np.load(trajectory_npy_path)
            print(f"✅ Trajectoire chargée: {len(self.trajectory)} points")
            return True
        except Exception as e:
            print(f"❌ Erreur lors du chargement: {e}")
            return False
    
    def get_next_waypoint(self) -> Optional[Tuple[float, float, float]]:
        """Retourne le prochain waypoint (x, y, theta)"""
        if self.trajectory is None or self.current_waypoint_idx >= len(self.trajectory):
            return None
        
        waypoint = self.trajectory[self.current_waypoint_idx]
        return (waypoint[0], waypoint[1], waypoint[2])
    
    def check_waypoint_reached(self, state: Dict) -> bool:
        """Vérifie si le waypoint actuel est atteint"""
        if self.trajectory is None or self.current_waypoint_idx >= len(self.trajectory):
            return False
        
        waypoint = self.trajectory[self.current_waypoint_idx]
        dx = state['x'] - waypoint[0]
        dy = state['y'] - waypoint[1]
        distance = np.sqrt(dx**2 + dy**2)
        
        if distance < self.reached_tolerance:
            print(f"✅ Waypoint {self.current_waypoint_idx} atteint!")
            self.current_waypoint_idx += 1
            return True
        
        return False
    
    def follow_trajectory(self, velocity: float = 0.5):
        """
        Suit la trajectoire avec correction en temps réel
        
        Args:
            velocity: Vitesse de suivi (m/s)
        """
        if self.trajectory is None:
            print("❌ Aucune trajectoire chargée")
            return
        
        print(f"🤖 Début du suivi de trajectoire ({len(self.trajectory)} points)")
        
        self.current_waypoint_idx = 0
        
        while self.current_waypoint_idx < len(self.trajectory):
            # Récupère le waypoint actuel
            waypoint = self.get_next_waypoint()
            if waypoint is None:
                break
            
            x_target, y_target, theta_target = waypoint
            
            # Envoie le waypoint au microcontrôleur
            self.mc.send_waypoint(x_target, y_target, theta_target, velocity)
            
            # Attend d'avoir atteint le waypoint
            time.sleep(0.1)
            state = self.mc.get_current_state()
            
            # Affiche la distance au waypoint
            dx = state['x'] - x_target
            dy = state['y'] - y_target
            distance = np.sqrt(dx**2 + dy**2)
            
            if self.current_waypoint_idx % 10 == 0:
                print(f"  Point {self.current_waypoint_idx}: distance = {distance:.3f} m, "
                      f"pos = ({state['x']:.2f}, {state['y']:.2f})")
            
            # Vérifie si le waypoint est atteint
            self.check_waypoint_reached(state)
        
        print("✅ Trajectoire complète!")

def main():
    """Exemple d'utilisation"""
    print("🤖 Interface Microcontrôleur - Robot Traceur de Plan PDF")
    print("=" * 60)
    
    # Crée l'interface avec le microcontrôleur
    mc = MicrocontrollerInterface(port='COM3', baudrate=115200)
    
    # Se connecte
    if not mc.connect():
        print("❌ Impossible de se connecter. Vous pouvez continuer en mode test.")
    else:
        # Démarre la lecture continue
        mc.start_continuous_reading()
        
        # Crée le suivi de trajectoire
        follower = TrajectoryFollower(mc)
        
        # Charge la trajectoire
        trajectory_path = os.path.join(
            os.path.dirname(__file__),
            '../data/trajectory.npy'
        )
        
        if follower.load_trajectory(trajectory_path):
            # Suit la trajectoire
            follower.follow_trajectory(velocity=0.5)
        
        # Déconnecte
        mc.disconnect()

if __name__ == "__main__":
    import os
    main()