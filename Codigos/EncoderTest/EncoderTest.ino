// Pines de conexión del encoder
const int pinA = 4; // Canal A del encoder
const int pinB = 5; // Canal B del encoder

volatile int encoderValue = 0; // Valor del encoder
volatile int lastEncoded = 0;

void IRAM_ATTR handleEncoder() {
  int MSB = digitalRead(pinA); // LSB primero
  int LSB = digitalRead(pinB); // MSB segundo

  int encoded = (MSB << 1) | LSB; // Concatena los valores de los pines
  int sum = (lastEncoded << 2) | encoded; // Combina los valores actuales y anteriores

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderValue++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderValue--;
  }

  lastEncoded = encoded; // Actualiza el último valor codificado
}

void setup() {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinA), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), handleEncoder, CHANGE);

  Serial.begin(115200);
}

void loop() {
  // Imprime el valor del encoder
  Serial.println(encoderValue);
  delay(100);
}
