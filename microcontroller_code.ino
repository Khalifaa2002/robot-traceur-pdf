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
#define ENCODER_LEFT_A 2
#define ENCODER_LEFT_B 4
#define ENCODER_RIGHT_A 3
#define ENCODER_RIGHT_B 5

// === VARIABLES GLOBALES ===
volatile long encoder_left = 0;
volatile long encoder_right = 0;
long last_encoder_left = 0;
long last_encoder_right = 0;
float x = 0.0, y = 0.0, theta = 0.0;
float v_cmd = 0.0, omega_cmd = 0.0;
unsigned long last_cmd_time = 0;

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
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  
  // Attache les interruptions pour les encodeurs (RISING pour simplifier, x2 au lieu de x4 si CHANGE)
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), isr_encoder_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), isr_encoder_right, CHANGE);
  
  Serial.println("{\"msg\":\"🤖 Robot Traceur démarré\"}");
}

// === ISRs (Interrupt Service Routines) ===
void isr_encoder_left() {
  if (digitalRead(ENCODER_LEFT_A) == digitalRead(ENCODER_LEFT_B)) {
    encoder_left--;
  } else {
    encoder_left++;
  }
}

void isr_encoder_right() {
  if (digitalRead(ENCODER_RIGHT_A) == digitalRead(ENCODER_RIGHT_B)) {
    encoder_right--;
  } else {
    encoder_right++;
  }
}

// === ODOMÉTRIE ===
void update_odometry() {
  long curr_left, curr_right;
  
  // Safe atomic read of volatile variables
  noInterrupts();
  curr_left = encoder_left;
  curr_right = encoder_right;
  interrupts();
  
  long delta_left_ticks = curr_left - last_encoder_left;
  long delta_right_ticks = curr_right - last_encoder_right;
  last_encoder_left = curr_left;
  last_encoder_right = curr_right;
  
  // Convertit les pulses en distance
  float dist_left = delta_left_ticks / COUNTS_PER_METER;
  float dist_right = delta_right_ticks / COUNTS_PER_METER;
  float dist_avg = (dist_left + dist_right) / 2.0;
  float delta_theta = (dist_right - dist_left) / WHEEL_BASE;
  
  // Met à jour la position
  x += dist_avg * cos(theta);
  y += dist_avg * sin(theta);
  theta += delta_theta;
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
    last_cmd_time = millis(); // Reset watchdog
    
    Serial.println("{\"status\":\"motor\"}");
    
  } else if (command == "GOTO") {
    // Waypoint (simplifié: on envoi juste "reçu")
    last_cmd_time = millis(); // Reset watchdog
    Serial.println("{\"status\":\"waypoint_received\"}");
  }
  
  else if (command == "STOP") {
    set_motor_speed(0, 0);
    last_cmd_time = millis(); // Reset watchdog
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
  // Safe read for JSON payload
  long copy_left, copy_right;
  noInterrupts();
  copy_left = encoder_left;
  copy_right = encoder_right;
  interrupts();
  
  doc["encoder_left"] = copy_left;
  doc["encoder_right"] = copy_right;
  
  // Voltage divider (assuming R1=10k, R2=4.7k -> Factor ~ 3.12)
  doc["battery"] = analogRead(A0) * (5.0 / 1023.0) * 3.12;
  
  serializeJson(doc, Serial);
  Serial.println();
}

// === LOOP PRINCIPALE ===
void loop() {
  unsigned long current_millis = millis();
  
  // Lit les commandes série en non-bloquant
  read_serial();
  
  // Watchdog de sécurité (500ms sans msg)
  if (current_millis - last_cmd_time > 500) {
    set_motor_speed(0, 0); // Arrêt des moteurs
  }
  
  static unsigned long last_odo = 0;
  if (current_millis - last_odo > 10) { // 100 Hz Odométrie
    update_odometry();
    last_odo = current_millis;
  }
  
  // Envoie l'état (à ~10 Hz)
  static unsigned long last_send = 0;
  if (current_millis - last_send > 100) {
    send_state();
    last_send = current_millis;
  }
}

// ✅ FIXED: [BUG 3: Race condition ISR lock, BUG 4: millis loop, Watchdog, Bidirectional Encoders, Battery Divider, Incremental Odometry Fix]