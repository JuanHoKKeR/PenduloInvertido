#include "Wire.h"
#include "MPU6050.h"

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  Serial.println("Inicializando MPU6050...");
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 conectado correctamente");
  } else {
    Serial.println("Error al conectar el MPU6050");
  }

  Serial.println("Calibrando MPU6050...");
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  // Leer los sesgos calibrados
  Serial.print("Acelerómetro abias: ");
  Serial.print(mpu.getXAccelOffset()); Serial.print(" ");
  Serial.print(mpu.getYAccelOffset()); Serial.print(" ");
  Serial.println(mpu.getZAccelOffset());
  
  Serial.print("Giroscopio gbias: ");
  Serial.print(mpu.getXGyroOffset()); Serial.print(" ");
  Serial.print(mpu.getYGyroOffset()); Serial.print(" ");
  Serial.println(mpu.getZGyroOffset());
}

void loop() {
  // Nada en el bucle
}
