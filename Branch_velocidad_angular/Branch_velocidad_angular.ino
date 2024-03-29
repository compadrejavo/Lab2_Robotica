
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

char estado = '0';  //variable que recibe las instrucciones del celular

//Pines puente H
int ENA = 10; // MCU PWM Pin 10 to ENA on L298n Board
int IN1 = 9;  // MCU Digital Pin 9 to IN1 on L298n Board 
int IN2 = 8;  // MCU Digital Pin 8 to IN2 on L298n Board
 
int ENB = 5;  // MCU PWM Pin 5 to ENB on L298n Board
int IN3 = 7;  // MCU Digital pin 7 to IN3 on L298n Board
int IN4 = 6;  // MCU Digital pin 6 to IN4 on L298n Board

//Pines giroscopo
//GND - GND
//VCC - VCC
//SDA - Pin A4
//SCL - Pin A5

const int mpuAddress = 0x68;  // Puede ser 0x68 o 0x69
MPU6050 mpu(mpuAddress);
 
int ax, ay, az;
int gx, gy, gz;
int error = 0;
 
long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;
const float gyroScale = 250.0 / 32768.0;


void setup() 
{
  Serial.begin(9600);    
  pinMode(ENA, OUTPUT); //Set all the L298n Pin to output
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Wire.begin();
  mpu.initialize();
}

void loop() {
  estado='x'; // Resetear variable
  if(Serial.available()>0){  // Entrar solo cuando se envíe algo
    estado=Serial.read();  // Leer lo que la antena reciba   
    
    switch(estado){ 
      case 'f':  forward();     break;
      case 'b':  backward();   break;
      case 'l':  left();     break;
      case 'r':  right();    break;
      case 'h':  halt();    break;   
      default: halt();     
    }//Fin switch  
  }

  //------------------------------------------ Estabilizar ---------------------------
  int angulo_actual = angulox();
  int velocidad_angular = gx * gyroScale;
  Serial.print(F("Velocidad Angular en X:  "));
  Serial.println(velocidad_angular); 
  if(velocidad_angular==0){
    velocidad_angular=0.000000000000000000000000000000000001;
  }
  int tiempo_a_empujar = angulo_actual / velocidad_angular;
  tiempo_a_empujar = abs(tiempo_a_empujar);

  if(angulo_actual>0){
    analogWrite(ENA, 255);  //Full Power!
    analogWrite(ENB, 255);  //Full Power!
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW);  
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW);
    delay(tiempo_a_empujar);  //Empujar por el tiempo que diga la funcion despejada
  }
  else if(angulo_actual<0){
    analogWrite(ENA, 255);  //Full Power!
    analogWrite(ENB, 255);  //Full Power!
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(tiempo_a_empujar);  //Empujar por el tiempo que diga la funcion despejada
  }

   
}


//------------------------------------------ Movimiento Motores DC---------------------------
void forward(){  
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);  
  analogWrite(ENA, 200); 
  analogWrite(ENB, 200);
  delay(200); 
}

void right(){  
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);  
  analogWrite(ENA, 200); 
  analogWrite(ENB, 200);
  delay(200);
}

void backward(){   
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  analogWrite(ENA, 200); 
  analogWrite(ENB, 200);
  delay(200); 
}

void left (){ 
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);  
  analogWrite(ENA, 200); 
  analogWrite(ENB, 200);
  delay(200);
}

void halt(){ 
  digitalWrite(IN1, LOW); // Apagar Motor
  digitalWrite(IN2, LOW); // Apagar Motor
  digitalWrite(IN3, LOW); // Apagar Motor
  digitalWrite(IN4, LOW); // Apagar Motor 
}


//------------------------------------------ Sensor Inercial ---------------------------

int angulox()
{
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  updateFiltered();  
  delay(10);  
  return ang_x+error;
}

void updateFiltered()
{
   dt = (millis() - tiempo_prev) / 1000.0;
   tiempo_prev = millis();
 
   //Calcular los ángulos con acelerometro
   float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2)))*(180.0 / 3.14);
   float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2)))*(180.0 / 3.14);
 
   //Calcular angulo de rotación con giroscopio y filtro complementario
   ang_x = 0.98*(ang_x_prev + (gx / 131)*dt) + 0.02*accel_ang_x;
   ang_y = 0.98*(ang_y_prev + (gy / 131)*dt) + 0.02*accel_ang_y;
 
   ang_x_prev = ang_x;
   ang_y_prev = ang_y;
}
