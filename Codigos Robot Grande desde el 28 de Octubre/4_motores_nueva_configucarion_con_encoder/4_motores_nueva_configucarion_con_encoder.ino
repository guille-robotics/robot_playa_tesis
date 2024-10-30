volatile unsigned long lastInterruptTimeDerecho = 0;
volatile unsigned long lastInterruptTimeIzquierdo = 0;
///////////////////// COMUNICACION SERIAL ////////////////
String inputString = "";
bool stringComplete = false;
const char separator = ',';
const int dataLength = 4;
double data[dataLength];

////// Motor Derecho ////
const int C1R = 3;  // Entrada de la señal A del encoder C1 (cable amarillo)
const int C2R = 2;  // Entrada de la señal B del encoder C2 (cable verde)

////// Motor Izquierdo ////
const int C1L = 19;  // Entrada de la señal A del encoder C1 (cable amarillo)
const int C2L = 18;  // Entrada de la señal B del encoder C2 (cable verde)

// Encoder Derecho
volatile long nR = 0;
volatile int antR = 0;
volatile int actR = 0;

// Encoder Izquierdo
volatile long nL = 0;
volatile int antL = 0;
volatile int actL = 0;

int cv2_Derecho=0;
int cv1_Izquierdo=0;

int cv4_Derecho=0;
int cv3_Izquierdo=0;

int cv_Derecho=0;
int cv_Izquierdo=0;
/////// 2 y 4 Derecho   1 y 3 Izquierdo/////////
//////PINES DRIVER Y ENCODERS /////
// Pines Motor 1
int R1_PWM=5;
int L1_PWM=6;

// Pines Motor 2
int R2_PWM=11;
int L2_PWM=12;

// Pines Motor 3
int R3_PWM=7;
int L3_PWM=8;

// Pines Motor 4
int R4_PWM=9;
int L4_PWM=10;

/// Variables para Lectura ///
unsigned long lastTime = 0;  // Tiempo anterior
unsigned long sampleTime = 100;  // Tiempo de muestreo

/////////////////////// VELOCIDAD ANGULAR //////////////////////
double wR = 0.0;  // Velocidad angular Rueda Derecha en rad/s.
double wL = 0.0;  // Velocidad angular Rueda Izquierda en rad/s.

/////////////////////// Valores //////////////////////
double constValue = 0.20; //(1000*2*pi)/R, R= 25910

void setup()
{
  Serial.begin(9600);

  // Pines Motor 1
  pinMode(R1_PWM,OUTPUT);
  pinMode(L1_PWM,OUTPUT);

  // Pines Motor 2
  pinMode(R2_PWM,OUTPUT);
  pinMode(L2_PWM,OUTPUT);
  
  // Pines Motor 3
  pinMode(R3_PWM,OUTPUT);
  pinMode(L3_PWM,OUTPUT);
  
  // Pines Motor 4
  pinMode(R4_PWM,OUTPUT);
  pinMode(L4_PWM,OUTPUT);  

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
  if (stringComplete) 
  {
    for (int i = 0; i < dataLength ; i++)
    {
      int index = inputString.indexOf(separator);
      data[i] = inputString.substring(0, index).toFloat();
      inputString = inputString.substring(index + 1);
     }
     cv_Derecho = data[0];
     cv_Izquierdo = data[1];
     /*cv2_Derecho = data[0];
     cv4_Derecho = data[1];
     cv1_Izquierdo = data[2];
     cv3_Izquierdo = data[3];*/

     inputString = "";
     stringComplete = false;
  }

  //Serial.print("Cuentas: ");Serial.println(n);
  if (millis() - lastTime >= sampleTime){  // Se actualiza cada sampleTime (milisegundos)
    lastTime = millis();

    if (cv_Derecho > 0) adelante(L2_PWM,R2_PWM,(cv_Derecho/8.2)); else atras(L2_PWM,R2_PWM,abs(cv_Derecho/8.2));
    if (cv_Derecho > 0) adelante(L4_PWM,R4_PWM,cv_Derecho); else atras(L4_PWM,R4_PWM,abs(cv_Derecho));

    if (cv_Izquierdo > 0) atras(L1_PWM,R1_PWM,(cv_Izquierdo/8.2)); else adelante(L1_PWM,R1_PWM,abs(cv_Izquierdo/8.2));
    if (cv_Izquierdo > 0) atras(L3_PWM,R3_PWM,cv_Izquierdo); else adelante(L3_PWM,R3_PWM,abs(cv_Izquierdo));

    Serial.print("Encoder Derecho: ");
    Serial.print(nR);
    Serial.print("||||");
    Serial.print(" Encoder Izquierdo: ");
    Serial.println(nL);
    /*Serial.print("Cv2_Derecho: ");
    Serial.print(cv2_Derecho);
    Serial.print(" Cv4_Derecho: ");
    Serial.print(cv4_Derecho);
    Serial.print(" Cv1_Izquierdo: ");
    Serial.print(cv1_Izquierdo);
    Serial.print(" Cv3_Izquierdo: ");
    Serial.println(cv3_Izquierdo)*/;
   }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
/// FUNCIONES DE ENCODER /////
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

  if (antL == 2 && actL == 0) nL--;
  if (antL == 0 && actL == 1) nL--;
  if (antL == 3 && actL == 2) nL--;
  if (antL == 1 && actL == 3) nL--;

  if (antL == 1 && actL == 0) nL++;
  if (antL == 3 && actL == 1) nL++;
  if (antL == 0 && actL == 2) nL++;
  if (antL == 2 && actL == 3) nL++;
}

void computeWR(void) {
  wR = (constValue * nR) / (millis() - lastTime); // Calculamos velocidad Rueda Derecha rad/s
  nR = 0;  // Reiniciamos los pulsos.
}

void computeWL(void) {
  wL = (constValue * nL) / (millis() - lastTime); // Calculamos velocidad Rueda Izquierda rad/s
  nL = 0;  // Reiniciamos los pulsos.
}

///// Funciones de Control de Motores /////
// Funcion para mover en Sentido Horario
void adelante(int analogPinR, int analogPinL, int pwm)
{     
  analogWrite(analogPinR,pwm);
  analogWrite(analogPinL,0);
}
// Funcion para mover en Sentido AntiHorario
void atras(int analogPinR, int analogPinL, int pwm)
{
  analogWrite(analogPinR,0);
  analogWrite(analogPinL,pwm);
}