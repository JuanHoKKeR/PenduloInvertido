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

float Kp = 10; 
float Ki = 0; 
float Kd = 0; 


volatile float error[3] = {0, 0, 0};
volatile float PID[3]={0, 0, 0};

float setpoint = 20.0;
volatile float output = 0;

Ticker timer;

// PWM Channel configuration
const int pwmFreq = 1000;
const int pwmResolution = 8;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;

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
  motor.velocity = (pulses * 60.0) / (MotorPasos*Ts); // Reemplaza 
  motor.position = encoderPos_Motor;
  lastEncoderPos_Motor = encoderPos_Motor;

  // Calcular la velocidad y el ángulo del encoder del péndulo
  pendulum.encoderPosition = encoderValue;
  long pulses_encoder = encoderValue - lastEncoderValue;
  pendulum.encoderVelocity = (pulses_encoder * 60.0) / (EncoderPasos*Ts);
  pendulum.encoderAngle = (encoderValue / (float)EncoderPasos) * 360.0; // Convertir a grados
  lastEncoderValue = encoderValue;
  //if (pendulum.encoderAngle > 45 || pendulum.encoderAngle < 5) {motorStop();}
  //else{Controlador();}
}

void Controlador() {
  error[0] = setpoint - pendulum.encoderAngle;
  PID[2] = error[0]; 
  PID[1] = (PID[1]+(error[0]*Ts));
  PID[0] = (error[0]-error[1])/Ts;
  output = Kp*PID[2]+Ki*PID[1]+Kd*PID[0]; //Kp*P + Ki*I + Kd*D
  error[1]=error[0];

  if (abs(output) > outMax) {
    output = output > 0 ? outMax : -outMax;
  }
  pwmOut(int(output));
}

void pwmOut(int out) {
  
  if (abs(out) > outMax) {
    out = out > 0 ? outMax : -outMax;
  }
  if (out > 0) {
    ledcWrite(pwmChannel1, abs(out));
    ledcWrite(pwmChannel2, 0);
  }
  else if (out < 0) {
    ledcWrite(pwmChannel1, 0);
    ledcWrite(pwmChannel2, abs(out));
  }
  else {
    ledcWrite(pwmChannel1, 0);
    ledcWrite(pwmChannel2, 0);
  }
  //Serial.println(out);
}

void motorStop() {
  
  ledcWrite(pwmChannel1, 255);
  ledcWrite(pwmChannel2, 255);

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

  // Configuración de los canales PWM
  ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
  ledcSetup(pwmChannel2, pwmFreq, pwmResolution);

  // Adjuntar los canales PWM a los pines
  ledcAttachPin(IN1, pwmChannel1);
  ledcAttachPin(IN2, pwmChannel2);

  // Iniciar el temporizador para calcular cada 50 ms
  timer.attach(Ts, calculate);
}

void loop() {
  // Imprimir los valores en el monitor serie
  Serial.print("SetPoint: ");Serial.print(setpoint);
  Serial.print("  Angulo: "); Serial.print(pendulum.encoderAngle);
  Serial.print("  Error: "); Serial.print(error[0]);
  Serial.print("  Controlador: "); Serial.println(output);
  //pwmOut(-200);
  for(int i=0;i<=255;i++){
    pwmOut(i);
    Serial.print((i*100)/255); Serial.println("%");
    delay(80);
  }
}
