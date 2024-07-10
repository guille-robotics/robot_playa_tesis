////Medición de Voltaje ////
double Sensibilidad = 0.100;  // Sensibilidad en Voltios/Amperio para sensor de 5A
int pinCorriente = A0;
int pinVoltaje = A1;
float factorDivisor = 5.22;  // Factor del divisor de voltaje

////// Motor Derecho ////
const int C1R =3; //3;  // Entrada de la señal A del encoder C1 (cable Morado)
const int C2R =2; //2;  // Entrada de la señal B del encoder C2 (cable Azul)

////// Motor Izquierdo ////
const int C1L = 18;//19;  // Entrada de la señal A del encoder C1 (cable Azul)
const int C2L = 19;//18;  // Entrada de la señal B del encoder C2 (cable Morado)

// Encoder Derecho
volatile int nR = 0;
volatile int antR = 0;
volatile int actR = 0;

// Encoder Izquierdo
volatile int nL = 0;
volatile int antL = 0;
volatile int actL = 0;

unsigned long lastTime = 0;    // Tiempo anterior
unsigned long sampleTime = 100; // Tiempo de muestreo

/////////////////////// VELOCIDAD ANGULAR //////////////////////
double wR = 0.0;  // Velocidad angular Rueda Derecha en rad/s.
double wL = 0.0;  // Velocidad angular Rueda Izquierda en rad/s.

/////////////////////// Valores //////////////////////
double constValue = 0.20; //(1000*2*pi)/R, R= 25910

//////////////////////// ROBOT /////////////////////////
double uMeas  = 0;
double wMeas  = 0;
double xp = 0.0, yp = 0.0;
double x = 0.0, y = 0.0;
double phi = 0.0;


const double R = 0.24; // radio de la llanta
const double d = 0.70; // Distancia entre llantas


void setup() {
  // Configuracion de los pines
  Serial.begin(9600);
  pinMode(C1R, INPUT_PULLUP);
  pinMode(C2R, INPUT_PULLUP);

  pinMode(C1L, INPUT_PULLUP);
  pinMode(C2L, INPUT_PULLUP);

  // Interrupciones
  attachInterrupt(digitalPinToInterrupt(C1R), encoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2R), encoderR, CHANGE);

  attachInterrupt(digitalPinToInterrupt(C1L), encoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2L), encoderL, CHANGE);

  lastTime = millis();
}

void loop() {
  /// Medicion de Corriente y Voltaje ///
  double I = get_corriente(200);  // Obtener corriente promedio de 200 muestras
  double V = get_voltaje(200);    // Obtener voltaje promedio de 200 muestras

  if (millis() - lastTime >= sampleTime) { // Se actualiza cada sampleTime (milisegundos)
    computeWR();
    computeWL(); 
    velocityRobot(wR, wL);

    phi = phi+0.1*wMeas;
    xp = uMeas*cos(phi);
    yp = uMeas*sin(phi);
    
    x = x + 0.1*xp;
    y = y + 0.1*yp; 
      
    // Enviar las posiciones y ángulo a través de la comunicación serial en formato CSV
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.print(phi);
    Serial.print(",");
    Serial.print(uMeas);
    Serial.print(",");
    Serial.print(wMeas);
    Serial.print(",");
    Serial.print(I);
    Serial.print(",");
    Serial.println(V);

    lastTime = millis();
  }
}

/// Funciones //
void encoderR() { // Funcion Encoder Rueda Derecha
  antR = actR;
  if (digitalRead(C2R)) bitSet(actR, 0);
  else bitClear(actR, 0);
  if (digitalRead(C1R)) bitSet(actR, 1);
  else bitClear(actR, 1);

  if (antR == 2 && actR == 0) nR--;
  if (antR == 0 && actR == 1) nR--;
  if (antR == 3 && actR == 2) nR--;
  if (antR == 1 && actR == 3) nR--;

  if (antR == 1 && actR == 0) nR++;
  if (antR == 3 && actR == 1) nR++;
  if (antR == 0 && actR == 2) nR++;
  if (antR == 2 && actR == 3) nR++;
}

void encoderL() {
  antL = actL;
  if (digitalRead(C2L)) bitSet(actL, 0);
  else bitClear(actL, 0);
  if (digitalRead(C1L)) bitSet(actL, 1);
  else bitClear(actL, 1);

  if (antL == 2 && actL == 0) nL++;
  if (antL == 0 && actL == 1) nL++;
  if (antL == 3 && actL == 2) nL++;
  if (antL == 1 && actL == 3) nL++;

  if (antL == 1 && actL == 0) nL--;
  if (antL == 3 && actL == 1) nL--;
  if (antL == 0 && actL == 2) nL--;
  if (antL == 2 && actL == 3) nL--;
}

void computeWR(void) {
  wR = (constValue * nR) / (millis() - lastTime); // Calculamos velocidad Rueda Derecha rad/s
  nR = 0;  // Reiniciamos los pulsos.
}

void computeWL(void) {
  wL = (constValue * nL) / (millis() - lastTime); // Calculamos velocidad Rueda Izquierda rad/s
  nL = 0;  // Reiniciamos los pulsos.
}


void velocityRobot(double w1, double w2)
{
  uMeas = (R*(w1+w2))/2.0; // w1=wR w2=Wl
  wMeas = (R*(w1-w2))/d;
}

float get_corriente(int n_muestras) {
  float voltajeSensor;
  float corriente = 0;
  for (int i = 0; i < n_muestras; i++) {
    voltajeSensor = analogRead(pinCorriente) * (5.0 / 1023.0);  // Lectura del sensor
    corriente = corriente + (voltajeSensor - 2.5) / Sensibilidad;  // Ecuación para obtener la corriente
  }
  corriente = corriente / n_muestras;
  return corriente;
}

float get_voltaje(int n_muestras) {
  float voltajeSensor;
  float voltaje = 0;
  for (int i = 0; i < n_muestras; i++) {
    voltajeSensor = analogRead(pinVoltaje) * (5.0 / 1023.0);  // Lectura del sensor
    voltaje = voltaje + voltajeSensor * factorDivisor;  // Ajuste del voltaje según el factor del divisor
  }
  voltaje = voltaje / n_muestras;
  return voltaje;
}
