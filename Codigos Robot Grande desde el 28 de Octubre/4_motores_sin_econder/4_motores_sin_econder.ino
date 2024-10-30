/*
Este codigo agrega la detencion al conteo de encoders cuando el motor se detiene.Realiza de manera correcta la lectura de encoders hacia delante y hacia atras.
*/

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

/////// 2 y 4 Derecho   1 y 3 Izquierdo/////////
//////PINES DRIVER Y ENCODERS /////
// Pines Motor 1
int R1_EN=13;
int L1_EN=12;
int R1_PWM=38;
int L1_PWM=39;

// Pines Motor 2
int R2_EN=7;
int L2_EN=6;
int R2_PWM=32;
int L2_PWM=33;

// Pines Motor 3
int R3_EN=11;
int L3_EN=10;
int R3_PWM=36;
int L3_PWM=37;

// Pines Motor 4
int R4_EN=9;
int L4_EN=8;
int R4_PWM=34;
int L4_PWM=35;

/// Variables para Lectura ///
unsigned long lastTime = 0;  // Tiempo anterior
unsigned long sampleTime = 100;  // Tiempo de muestreo

void setup()
{
  Serial.begin(9600);

  // Pines Motor 1
  pinMode(R1_EN,OUTPUT);
  pinMode(L1_EN,OUTPUT);
  pinMode(R1_PWM,OUTPUT);
  pinMode(L1_PWM,OUTPUT);

  // Pines Motor 2
  pinMode(R2_EN,OUTPUT);
  pinMode(L2_EN,OUTPUT);
  pinMode(R2_PWM,OUTPUT);
  pinMode(L2_PWM,OUTPUT);
  
  // Pines Motor 3
  pinMode(R3_EN,OUTPUT);
  pinMode(L3_EN,OUTPUT);
  pinMode(R3_PWM,OUTPUT);
  pinMode(L3_PWM,OUTPUT);
  
  // Pines Motor 4
  pinMode(R4_EN,OUTPUT);
  pinMode(L4_EN,OUTPUT);
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

    if (cv2_Derecho > 0) anticlockwise(L2_PWM,R2_PWM,R2_EN,L2_EN,cv2_Derecho); else clockwise(L2_PWM,R2_PWM,R2_EN,L2_EN,abs(cv2_Derecho));
    if (cv4_Derecho > 0) anticlockwise(L4_PWM,R4_PWM,R4_EN,L4_EN,cv4_Derecho); else clockwise(L4_PWM,R4_PWM,R4_EN,L4_EN,abs(cv4_Derecho));

    if (cv1_Izquierdo > 0) clockwise(L1_PWM,R1_PWM,R1_EN,L1_EN,cv1_Izquierdo); else anticlockwise(L1_PWM,R1_PWM,R1_EN,L1_EN,abs(cv1_Izquierdo));
    if (cv3_Izquierdo > 0) clockwise(L3_PWM,R3_PWM,R3_EN,L3_EN,cv3_Izquierdo); else anticlockwise(L3_PWM,R3_PWM,R3_EN,L3_EN,abs(cv3_Izquierdo));
    Serial.print("Cv2_Derecho: ");
    Serial.print(cv2_Derecho);
    Serial.print(" Cv4_Derecho: ");
    Serial.print(cv4_Derecho);
    Serial.print(" Cv1_Izquierdo: ");
    Serial.print(cv1_Izquierdo);
    Serial.print(" Cv3_Izquierdo: ");
    Serial.println(cv3_Izquierdo);
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
void clockwise(int pin1, int pin2,int analogPin1, int analogPin2, int pwm)
{
  digitalWrite(pin1, LOW);  
  digitalWrite(pin2, HIGH);      
  analogWrite(analogPin1,pwm);
  analogWrite(analogPin2,pwm);
}
// Funcion para mover en Sentido AntiHorario
void anticlockwise(int pin1, int pin2,int analogPin1, int analogPin2, int pwm)
{
  digitalWrite(pin1, HIGH);  
  digitalWrite(pin2, LOW);      
  analogWrite(analogPin1,pwm);
  analogWrite(analogPin2,pwm);
}