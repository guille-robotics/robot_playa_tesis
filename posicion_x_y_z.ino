////// Motor Derecho ////
const int C1R = 3;  // Entrada de la señal A del encoder C1 (cable amarillo)
const int C2R = 2;  // Entrada de la señal B del encoder C2 (cable verde)

////// Motor Izquierdo ////
const int C1L = 19;  // Entrada de la señal A del encoder C1 (cable amarillo)
const int C2L = 18;  // Entrada de la señal B del encoder C2 (cable verde)

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
    Serial.print(Umeas);
    Serial.print(",");
    Serial.println(Wmeas);

    lastTime = millis();
  }
}

void encoderR() {
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
