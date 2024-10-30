volatile unsigned long lastInterruptTimeDerecho = 0;
volatile unsigned long lastInterruptTimeIzquierdo = 0;
///////////////////// COMUNICACION SERIAL ////////////////
String inputString = "";
bool stringComplete = false;
const char separator = ',';
const int dataLength = 4;
double data[dataLength];

int cv2_Derecho=0;
int cv1_Izquierdo=0;

int cv4_Derecho=0;
int cv3_Izquierdo=0;

int cv=-100;
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
     
     cv2_Derecho = data[0];
     cv4_Derecho = data[1];
     cv1_Izquierdo = data[2];
     cv3_Izquierdo = data[3];

     inputString = "";
     stringComplete = false;
  }

  //Serial.print("Cuentas: ");Serial.println(n);
  if (millis() - lastTime >= sampleTime){  // Se actualiza cada sampleTime (milisegundos)
    lastTime = millis();

    if (cv > 0) adelante(L2_PWM,R2_PWM,(cv/8.2)); else atras(L2_PWM,R2_PWM,abs(cv/8.2));
    if (cv > 0) adelante(L4_PWM,R4_PWM,cv); else atras(L4_PWM,R4_PWM,abs(cv));

    if (cv > 0) atras(L1_PWM,R1_PWM,(cv/8.2)); else adelante(L1_PWM,R1_PWM,abs(cv/8.2));
    if (cv > 0) atras(L3_PWM,R3_PWM,cv); else adelante(L3_PWM,R3_PWM,abs(cv));

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