#include "Wire.h"
#include "MPU6050.h"

MPU6050 mpu;

// Valores de sesgos obtenidos de la calibración
int16_t axBias = -1938; // Sustituye con el valor real obtenido
int16_t ayBias = 445;  // Sustituye con el valor real obtenido
int16_t azBias = 1254;  // Sustituye con el valor real obtenido

int16_t gxBias = -86; // Sustituye con el valor real obtenido
int16_t gyBias = 17; // Sustituye con el valor real obtenido
int16_t gzBias = -15; // Sustituye con el valor real obtenido

unsigned long lastGyroTime = 0;
float gyroAngle = NAN;
float filteredAngle = NAN;

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

  // Aplicar los sesgos al sensor
  mpu.setXAccelOffset(axBias);
  mpu.setYAccelOffset(ayBias);
  mpu.setZAccelOffset(azBias);
  mpu.setXGyroOffset(gxBias);
  mpu.setYGyroOffset(gyBias);
  mpu.setZGyroOffset(gzBias);

  lastGyroTime = millis();
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  // Leer los valores de aceleración y giroscopio
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convertir los datos del acelerómetro a Gs
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  // Calcular el ángulo de inclinación (roll y pitch) en grados
  float accAngle = atan2(ay_g, az_g) * 57.2958;

  // Obtener el tiempo transcurrido
  unsigned long currentTime = millis();
  float timeDelta = (currentTime - lastGyroTime) / 1000.0; // en segundos
  lastGyroTime = currentTime;

  // Calcular el cambio de ángulo del giroscopio
  float gyroAngleDelta = gz / 131.0 * timeDelta; // 131.0 es la sensibilidad para 250dps
  if (isnan(gyroAngle)) gyroAngle = accAngle;
  gyroAngle += gyroAngleDelta;

  // Filtro complementario
  if (isnan(filteredAngle)) filteredAngle = accAngle;
  filteredAngle = 0.999 * (filteredAngle + gyroAngleDelta) + 0.001 * accAngle;

  // Imprimir los valores en el monitor serie
  Serial.print("accAngle: "); Serial.print(accAngle);
  Serial.print(" gyroAngle: "); Serial.print(gyroAngle);
  Serial.print(" filteredAngle: "); Serial.println(filteredAngle);

  // Esperar un poco antes de la siguiente lectura
  delay(10);
}
