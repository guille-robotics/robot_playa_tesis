/*
En este codigo se utiliza el metodo RISING dos veces para poder realizar la cuenta de manera correcta. Sin embargo, hay que limintar la velocidad del motor a 150 en CV, 
porque superior la lectura de los encoders se vuelve incorrecta.
MEJORAR: Que la lectura se detenga cuando el motor se detiene.
  */

int R_EN_LEFT=12;
int L_EN_LEFT=13;
int R_PWM_LEFT=10;
int L_PWM_LEFT=11;

///////////////////// COMUNICACION SERIAL ////////////////
String inputString = "";
bool stringComplete = false;
const char separator = ',';
const int dataLength = 1;
double data[dataLength];

int cv=0;

const int    C1 = 3; // Entrada de la señal A del encoder.
const int    C2 = 2; // Entrada de la señal B del encoder.

volatile long  n    = 0;
volatile byte ant  = 0;
volatile byte act  = 0;

bool ACW=false;
bool CW=false;

unsigned long lastTime = 0;  // Tiempo anterior
unsigned long sampleTime = 100;  // Tiempo de muestreo

void setup()
{
  Serial.begin(9600);

  pinMode(C1, INPUT);
  pinMode(C2, INPUT);

  pinMode(R_EN_LEFT,OUTPUT);
  pinMode(L_EN_LEFT,OUTPUT);
  pinMode(R_PWM_LEFT,OUTPUT);
  pinMode(L_PWM_LEFT,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(C1), encoder_A, RISING);
  attachInterrupt(digitalPinToInterrupt(C2), encoder_B, RISING);
  
  Serial.println("Numero de conteos");

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
     
     cv = data[0];

     inputString = "";
     stringComplete = false;
  }

  //Serial.print("Cuentas: ");Serial.println(n);
  if (millis() - lastTime >= sampleTime)
  {  // Se actualiza cada sampleTime (milisegundos)
      lastTime = millis();
        if (cv > 0) anticlockwise(L_PWM_LEFT,R_PWM_LEFT,R_EN_LEFT,L_EN_LEFT,cv); else clockwise(L_PWM_LEFT,R_PWM_LEFT,R_EN_LEFT,L_EN_LEFT,abs(cv));
      Serial.print("Cuentas: ");Serial.println(n);
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
// Encoder precisión cuádruple.
void encoder_A() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(C1);
 
  if(val == LOW) {
    CW = false; // Reverse
  }
  else {
    CW = true; // Forward
  }
   
  if (CW) {
    n++;
  }
}

void encoder_B() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(C2);
 
  if(val == LOW) {
    ACW = false; // Reverse
  }
  else {
    ACW = true; // Forward
  }
   
  if (ACW) {
    n--;
  }
}




void clockwise(int pin1, int pin2,int analogPin1, int analogPin2, int pwm)
{
  digitalWrite(pin1, LOW);  
  digitalWrite(pin2, HIGH);      
  analogWrite(analogPin1,pwm);
  analogWrite(analogPin2,pwm);
}

void anticlockwise(int pin1, int pin2,int analogPin1, int analogPin2, int pwm)
{
  digitalWrite(pin1, HIGH);  
  digitalWrite(pin2, LOW);      
  analogWrite(analogPin1,pwm);
  analogWrite(analogPin2,pwm);
}
