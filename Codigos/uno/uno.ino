#include <Arduino.h>
#include <Ticker.h>

// Pines de los encoders
#define Encoder_MotorA 5
#define Encoder_MotorB 18
#define Encoder_A 32
#define Encoder_B 33

#define Ts 0.05

#define IN1 26
#define IN2 27

volatile long encoderPos_Motor = 0;
volatile int lastEncoded_Motor = 0;
volatile int encoderValue = 0;
volatile int lastEncoded = 0;

const int MotorPasos = 900;
const int EncoderPasos = 2400;

unsigned long lastCalcTime = 0;
long lastEncoderPos_Motor = 0;
int lastEncoderValue = 0;

// Struct para el motor
struct MotorData {
  long position;
  float velocity;
} motor;

// Struct para el péndulo
struct PendulumData {
  int encoderPosition;
  float encoderAngle; // Ángulo en grados
  float encoderVelocity;
} pendulum;

// Variables
const int outMax = 255;

float Kp = 0.4;
float Ki = 0.45;
float Kd = 0.02;

volatile float error[3] = {0, 0, 0};
volatile float PID[3] = {0, 0, 0};
float integralMax = 10.0; // Límite máximo para el término integral
float integralMin = -10.0; // Límite mínimo para el término integral
float Ajuste_angulo = 2.0;

float setpoint = 0.0;
volatile float output = 0;
volatile float output_norm = 0;

Ticker timer;

void IRAM_ATTR handleEncoder() {
  int MSB = digitalRead(Encoder_A); // LSB primero
  int LSB = digitalRead(Encoder_B); // MSB segundo

  int encoded = (MSB << 1) | LSB; // Concatena los valores de los pines
  int sum = (lastEncoded << 2) | encoded; // Combina los valores actuales y anteriores

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderValue++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderValue--;
  }

  lastEncoded = encoded; // Actualiza el último valor codificado
}

void IRAM_ATTR handleEncoder_Motor() {
  int MSB = digitalRead(Encoder_MotorA); // LSB primero
  int LSB = digitalRead(Encoder_MotorB); // MSB segundo

  int encoded = (MSB << 1) | LSB; // Concatena los valores de los pines
  int sum = (lastEncoded_Motor << 2) | encoded; // Combina los valores actuales y anteriores

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderPos_Motor++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderPos_Motor--;
  }

  lastEncoded_Motor = encoded; // Actualiza el último valor codificado
}

void calculate() {
  // Calcular la velocidad del motor
  long pulses = encoderPos_Motor - lastEncoderPos_Motor;
  motor.velocity = (pulses * 60.0) / (MotorPasos * Ts);
  motor.position = encoderPos_Motor;
  lastEncoderPos_Motor = encoderPos_Motor;

  // Calcular la velocidad y el ángulo del encoder del péndulo
  pendulum.encoderPosition = encoderValue;
  long pulses_encoder = encoderValue - lastEncoderValue;
  pendulum.encoderVelocity = (pulses_encoder * 60.0) / (EncoderPasos * Ts);
  pendulum.encoderAngle = (encoderValue / (float)EncoderPasos) * 360.0; // Convertir a grados
  lastEncoderValue = encoderValue;
  if (abs(pendulum.encoderAngle) >= 20) {
    motorStop();
    setpoint=0;
  }
  else{Controlador();}
  //Controlador();
}

void Controlador() {
  
  if(pendulum.encoderAngle < setpoint){
    setpoint = setpoint + (Ajuste_angulo*Ts);
  }
  else{
    setpoint = setpoint - (Ajuste_angulo*Ts);
  }
  error[0] = setpoint - pendulum.encoderAngle;
  PID[2] = error[0];                              //Proporcional
  PID[1] = (PID[1] + (error[0] * Ts));            //Integral
  if (PID[1] > integralMax) {
    PID[1] = integralMax;
  } else if (PID[1] < integralMin) {
    PID[1] = integralMin;
  }
  PID[0] = (error[0] - error[1]) / Ts;            //Derivativo
  output = Kp * PID[2] + Ki * PID[1] + Kd * PID[0];
  error[1] = error[0];
  output_norm = output;
  if (abs(output) > 1) {
    output_norm = output > 0 ? 1.0 : -1.0;
  }
  //if (abs(pendulum.encoderAngle) >= 20) {
    //motorStop();
  //}
  //else {
    pwmOut(int(output * 255));
  //
}

void pwmOut(int out) {
  if (abs(out) > outMax) {
    out = out > 0 ? outMax : -outMax;
  }
  if (out > 0) {
    analogWrite(IN1, abs(out));
    analogWrite(IN2, 0);
  } else if (out < 0) {
    analogWrite(IN1, 0);
    analogWrite(IN2, abs(out));
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
  }
}

void motorStop() {
  analogWrite(IN1, 255);
  analogWrite(IN2, 255);
}

void processSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.startsWith("Kp=")) {
      Kp = input.substring(3).toFloat();
      Serial.print("Kp set to: ");
      Serial.println(Kp,5);
    } else if (input.startsWith("Ki=")) {
      Ki = input.substring(3).toFloat();
      Serial.print("Ki set to: ");
      Serial.println(Ki,5);
    } else if (input.startsWith("Kd=")) {
      Kd = input.substring(3).toFloat();
      Serial.print("Kd set to: ");
      Serial.println(Kd,5);
    } else if (input.startsWith("FA=")) {
      Ajuste_angulo = input.substring(3).toFloat();
      Serial.print("AngleFixed set to: ");
      Serial.println(Ajuste_angulo,5);
    } else {
      Serial.println("Invalid command. Use Kp=, Ki=, Kd= to set values.");
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(Encoder_MotorA, INPUT_PULLUP);
  pinMode(Encoder_MotorB, INPUT_PULLUP);

  pinMode(Encoder_A, INPUT_PULLUP);
  pinMode(Encoder_B, INPUT_PULLUP);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(Encoder_A), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_B), handleEncoder, CHANGE);

  attachInterrupt(digitalPinToInterrupt(Encoder_MotorA), handleEncoder_Motor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_MotorB), handleEncoder_Motor, CHANGE);

  // Iniciar el temporizador para calcular cada 50 ms
  timer.attach(Ts, calculate);

  Serial.println("Enter Kp, Ki, Kd values in the format: Kp=0.4, Ki=0.01, Kd=0");
}

void loop() {
  processSerialInput();

  // Imprimir los valores en el monitor serie
  // Serial.print("SetPoint: "); Serial.print(setpoint);
  // Serial.print("  Angulo: "); Serial.print(pendulum.encoderAngle);
  // Serial.print("  Error: "); Serial.print(error[0]);
  // Serial.print("  Controlador: "); Serial.print(output);
  // Serial.print("  Controlador Normalizado: "); Serial.print(output_norm);
  // Serial.print("  Entrada Motor: "); Serial.print(output_norm * 255);
  // Serial.print("  Velocidad Motor: "); Serial.println(motor.velocity);
  Serial.print(setpoint);Serial.print(",");Serial.print(pendulum.encoderAngle);Serial.print(",");Serial.print(-20);Serial.print(",");Serial.print(20);Serial.print(",");Serial.println(output);
  //delay(5); // Ajusta el delay según sea necesario para evitar sobrecarga del puerto serie
}
