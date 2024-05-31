const int C1R = 3;  // Entrada de la señal A del encoder C1 (cable amarillo)
const int C2R = 2;  // Entrada de la señal B del encoder C2 (cable verde)

// Encoder
volatile int nR = 0;
volatile int antR = 0;
volatile int actR = 0;

unsigned long lastTime = 0;    // Tiempo anterior
unsigned long sampleTime = 100; // Tiempo de muestreo


/////////////////////// VELOCIDAD ANGULAR //////////////////////
double w = 0.0;  // Velocidad angular en rad/s.

/////////////////////// Valores //////////////////////
double constValue = 0.20; //(1000*2*pi)/R, R= 25910
void setup() {
  // Configuracion de los pines
  Serial.begin(9600);
  pinMode(C1R, INPUT_PULLUP);
  pinMode(C2R, INPUT_PULLUP);
  // Interrupciones
  attachInterrupt(digitalPinToInterrupt(C1R), encoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2R), encoderR, CHANGE);
  lastTime = millis();
}

void loop() {
  if (millis() - lastTime >= sampleTime) { // Se actualiza cada sampleTime (milisegundos)
    //lastTime = millis();
    computeW(); 
    Serial.print("Velocidad Angular: ");
    Serial.println(w);
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

void computeW(void)
{
  w =(constValue*nR)/(millis()-lastTime); // Calculamos velocidad rad/s
  lastTime = millis(); // Almacenamos el tiempo actual.
  nR = 0;  // Reiniciamos los pulsos.
}


 
  
