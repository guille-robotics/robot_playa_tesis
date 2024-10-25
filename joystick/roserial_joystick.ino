#include <ros.h>
#include <geometry_msgs/Twist.h>

#define VRX_PIN  A0 // Arduino pin connected to VRX pin
#define VRY_PIN  A1 // Arduino pin connected to VRY pin

int xValue = 0; // To store value of the X axis
int yValue = 0; // To store value of the Y axis

// Definir nodo ROS
ros::NodeHandle nh;

// Crear mensaje de tipo Twist
geometry_msgs::Twist cmd_vel_msg;

// Definir publisher para el topic cmd_vel
ros::Publisher pub_cmd_vel("cmd_vel", &cmd_vel_msg);

void setup() {
  // Inicializar nodo ROS
  
    // Inicializar pines analógicos
  pinMode(VRX_PIN, INPUT);
  pinMode(VRY_PIN, INPUT);

  // Configurar velocidad de comunicación
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  // Publicar en el topic /cmd_vel
  nh.advertise(pub_cmd_vel);


}

void loop() {
  // Leer valores del joystick
  xValue = analogRead(VRX_PIN);
  yValue = analogRead(VRY_PIN);

  // Movimiento hacia adelante o atrás
  if (yValue < 5) {
    cmd_vel_msg.linear.x = 0.5;   // Adelante
    cmd_vel_msg.angular.z = 0.0;  // Sin rotación
  } else if (yValue > 1020) {
    cmd_vel_msg.linear.x = -0.5;  // Atrás
    cmd_vel_msg.angular.z = 0.0;  // Sin rotación
  }

  // Giro hacia la izquierda o derecha
  if (xValue < 5) {
    cmd_vel_msg.linear.x = 0.0;   // Detenerse
    cmd_vel_msg.angular.z = 1.0;  // Girar a la izquierda
  } else if (xValue > 1020) {
    cmd_vel_msg.linear.x = 0.0;   // Detenerse
    cmd_vel_msg.angular.z = -1.0; // Girar a la derecha
  }

  // Publicar el mensaje en el topic /cmd_vel
  pub_cmd_vel.publish(&cmd_vel_msg);

  // Procesar comunicaciones de ROS
  nh.spinOnce();

  // Añadir un pequeño delay para la siguiente publicación
  delay(100);
}

