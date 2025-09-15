#define EncoderPinA 4
#define EncoderPinB 13
#define ForwardPin 18
#define BackwardPin 19
#define EnablePin 32
#define EncoderPinAB 27
#define EncoderPinBB 14
#define ForwardPinB 5
#define BackwardPinB 17
#define EnablePinB 16
#define StandbyPin 23

volatile long Encodervalue = 0;   // Contador para Motor A
volatile long EncodervalueB = 0;  // Contador para Motor B

int angleError = 0
int oldAngleError = 0
float distanceError = 0
float oldDistanceError = 0
float kPw = (SOME VALUE)
float kDw = (SOME VALUE)
float kPx = (SOME VALUE)
float kDx = (SOME VALUE)

// Declaración anticipada de las funciones
void updateEncoderA();
void updateEncoderB();

void setup() {
  Serial.begin(115200);
  
  // Configuración Motor A
  pinMode(EncoderPinA, INPUT_PULLUP);
  pinMode(EncoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EncoderPinA), updateEncoderA, RISING);
  
  // Configuración Motor B
  pinMode(EncoderPinAB, INPUT_PULLUP);
  pinMode(EncoderPinBB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EncoderPinAB), updateEncoderB, RISING);
}

void loop() {
  static unsigned long lastPrint = 0;
  
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    
    // Lectura segura de los contadores
    long motorA, motorB;
    noInterrupts();
    motorA = Encodervalue;
    motorB = EncodervalueB;
    interrupts();
    
    Serial.print("Motor A: ");
    Serial.print(motorA);
    Serial.print(" pulsos | Motor B: ");
    Serial.print(motorB);
    Serial.println(" pulsos");
  }
}

// Función de interrupción para Motor A
void updateEncoderA() {
  if (digitalRead(EncoderPinB) == LOW) {
    Encodervalue++;  // Dirección positiva
  } else {
    Encodervalue--;  // Dirección negativa
  }
}

// Función de interrupción para Motor B
void updateEncoderB() {
  if (digitalRead(EncoderPinBB) == LOW) {
    EncodervalueB--;  // Dirección positiva
  } else {
    EncodervalueB++;  // Dirección negativa
  }
}

void updatePID(){
	angleError = goalAngle - (EncodervalueA-EncodervalueB);
	angleCorrection = kPw * angleError + kDw * (angleError - oldAngleError)
	oldAngleError = angleError

	distanceError = goalDistance - ((EncodervalueA+EncodervalueB)/2);
	distanceCorrection = kPx * distanceError + kDx * (distanceError - oldDistanceError)
	oldDistanceError = distanceError

	digitalWrite(ForwardPin,HIGH);
  digitalWrite(BackwardPin,LOW);
  ledcWrite(EnablePin, distanceCorrection-angleCorrection);
  Serial.println(Encodervalue);
  digitalWrite(ForwardPinB,HIGH);
  digitalWrite(BackwardPinB,LOW);
  ledcWrite(EnablePinB, distanceCorrection+angleCorrection);
}
