/*
 * Code pour STM32/Arduino - Robot Traceur
 * Gère: encodeurs, IMU, moteurs, communication UART
 */

#include <Wire.h>
#include <ArduinoJson.h>  // Bibliothèque pour JSON

// === PINS ===
#define MOTOR_LEFT_PWM 9
#define MOTOR_RIGHT_PWM 10
#define MOTOR_LEFT_DIR 8
#define MOTOR_RIGHT_DIR 7
#define ENCODER_LEFT 2
#define ENCODER_RIGHT 3

// === VARIABLES GLOBALES ===
volatile int encoder_left = 0;
volatile int encoder_right = 0;
float x = 0.0, y = 0.0, theta = 0.0;
float v_cmd = 0.0, omega_cmd = 0.0;

// === CONSTANTS ===
const float WHEEL_DIAMETER = 0.065;  // 6.5 cm
const float WHEEL_BASE = 0.15;       // 15 cm entre les roues
const float PPR = 20.0;              // Pulses per revolution de l'encodeur
const float COUNTS_PER_METER = PPR / (M_PI * WHEEL_DIAMETER);

// === SETUP ===
void setup() {
  Serial.begin(115200);
  
  // Initialise les pins
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(ENCODER_LEFT, INPUT);
  pinMode(ENCODER_RIGHT, INPUT);
  
  // Attache les interruptions pour les encodeurs
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), isr_encoder_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), isr_encoder_right, CHANGE);
  
  Serial.println("{\"msg\":\"🤖 Robot Traceur démarré\"}");
}

// === ISRs (Interrupt Service Routines) ===
void isr_encoder_left() {
  encoder_left++;
}

void isr_encoder_right() {
  encoder_right++;
}

// === ODOMÉTRIE ===
void update_odometry() {
  // Convertit les pulses en distance
  float dist_left = encoder_left / COUNTS_PER_METER;
  float dist_right = encoder_right / COUNTS_PER_METER;
  float dist_avg = (dist_left + dist_right) / 2.0;
  float delta_theta = (dist_right - dist_left) / WHEEL_BASE;
  
  // Met à jour la position
  x += dist_avg * cos(theta);
  y += dist_avg * sin(theta);
  theta += delta_theta;
  
  // Réinitialise les encodeurs
  encoder_left = 0;
  encoder_right = 0;
}

// === COMMANDE MOTEUR ===
void set_motor_speed(float left_speed, float right_speed) {
  // left_speed et right_speed en m/s (limité à [-1, 1])
  left_speed = constrain(left_speed, -1.0, 1.0);
  right_speed = constrain(right_speed, -1.0, 1.0);
  
  // Convertit en PWM (0-255)
  int left_pwm = (int)(left_speed * 255.0);
  int right_pwm = (int)(right_speed * 255.0);
  
  // Définit les directions
  digitalWrite(MOTOR_LEFT_DIR, left_speed >= 0 ? HIGH : LOW);
  digitalWrite(MOTOR_RIGHT_DIR, right_speed >= 0 ? HIGH : LOW);
  
  // Applique les PWM
  analogWrite(MOTOR_LEFT_PWM, abs(left_pwm));
  analogWrite(MOTOR_RIGHT_PWM, abs(right_pwm));
}

// === CINÉMATIQUE ===
void cmd_to_motor_speeds(float v, float omega, float &left_speed, float &right_speed) {
  // Convertit (v, omega) en vitesses des roues gauche et droite
  // v: vitesse linéaire (m/s)
  // omega: vitesse angulaire (rad/s)
  
  float v_left = v - (WHEEL_BASE / 2.0) * omega;
  float v_right = v + (WHEEL_BASE / 2.0) * omega;
  
  left_speed = v_left / 0.5;    // Normalize (assumez vitesse max = 0.5 m/s)
  right_speed = v_right / 0.5;
}

// === PARSING COMMANDES ===
void parse_command(String cmd) {
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, cmd);
  
  if (error) {
    Serial.println("{\"error\":\"JSON parse error\"}");
    return;
  }
  
  String command = doc["cmd"];
  
  if (command == "MOTOR") {
    // Commande directe moteur
    v_cmd = doc["v"];
    omega_cmd = doc["omega"];
    
    float left_speed, right_speed;
    cmd_to_motor_speeds(v_cmd, omega_cmd, left_speed, right_speed);
    set_motor_speed(left_speed, right_speed);
    
    Serial.println("{\"status\":\"motor\"}");
    
  } else if (command == "GOTO") {
    // Waypoint (simplifié: on envoi juste "reçu")
    Serial.println("{\"status\":\"waypoint_received\"}");
  }
  
  else if (command == "STOP") {
    set_motor_speed(0, 0);
    Serial.println("{\"status\":\"stopped\"}");
  }
}

// === LECTURE SERIE ===
void read_serial() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      parse_command(cmd);
    }
  }
}

// === ENVOI ETAT ===
void send_state() {
  StaticJsonDocument<256> doc;
  doc["x"] = x;
  doc["y"] = y;
  doc["theta"] = theta;
  doc["v"] = v_cmd;
  doc["omega"] = omega_cmd;
  doc["encoder_left"] = encoder_left;
  doc["encoder_right"] = encoder_right;
  doc["battery"] = analogRead(A0) * 5.0 / 1023.0;  // Lecture ADC batterie
  
  serializeJson(doc, Serial);
  Serial.println();
}

// === LOOP PRINCIPALE ===
void loop() {
  // Lit les commandes série
  read_serial();
  
  // Met à jour l'odométrie
  update_odometry();
  
  // Envoie l'état (à ~10 Hz)
  static unsigned long last_send = 0;
  if (millis() - last_send > 100) {
    send_state();
    last_send = millis();
  }
  
  delay(10);  // 100 Hz control loop
}