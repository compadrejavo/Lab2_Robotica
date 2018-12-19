//Importar las Bibliotecas
#include "I2Cdev.h"     //Clases para la interfaz con dispositivos IC2, en este caso el MPU6050  
#include "MPU6050.h"    //Biblioteca interfaz especifica del MPU6050
#include "Wire.h"       //Biblioteca de comunicacion con la interfaz IC2

char estado = '0';  //variable que recibe las instrucciones del celular

//Pines puente H
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;

//Pines giroscopo
//GND - GND
//VCC - VCC
//SDA - Pin A4
//SCL - Pin A5

//Iniciar definición del sensor MPU6050
const int mpuAddress = 0x68;  // Puede ser 0x68 o 0x69
MPU6050 mpu(mpuAddress);

//Variables que guardaran los datos RAW del sensor
int ax, ay, az;
int gx, gy, gz;

//Variables que influyen en el algoritmo de control de estabilidad
int angulo_actual; 
int angulo_limite = 30; //Define el angulo en donde se deberia ejercer la máxima fuerza en las ruedas (en ambas direcciones)
int freq = 10;          //Frecuencia en que se realiza la correccion
int error = -4;         //Correcion de error en la medicion del angulo por parte del giroscopio
int potencia;           //Potencia del motor a aplicar en las ruedas

//Variables para transformar lecturas del sensor en el sistema internacional
long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

//---------------------------------------Setup--------------------------------------------------------------

void setup() 
{
  Serial.begin(9600);   //Setear ratio de transimision de datos
  pinMode(ENA, OUTPUT); //Setear todos los pines del puente H a output
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Wire.begin();         //Iniciar comunicacion con la interfaz I2C
  mpu.initialize();     //Iniciar giroscopio
}

//---------------------------------------Loop--------------------------------------------------------------

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
      case '7':  angulo_limite++;     break; 
      case '9':  angulo_limite--;     break; 
      case '4':  error++;     break; 
      case '6':  error--;     break; 
      case '1':  freq++;     break; 
      case '3':  freq--;     break; 
      default: halt();     
    }//Fin switch  
  }
  //Mostrar los valores de las variables en la app, ella los separa por medio del simbolo "|"
  Serial.print(angulo_limite);      
  Serial.print("|");
  Serial.print(error);         
  Serial.print("|");
  Serial.print(freq);
  

  //------------------------------------------ Estabilizar ---------------------------

      
  angulo_actual = angulox();  //Recibir el angulo del robot

  int ganancia = angulo_actual * 100 / angulo_limite;  //Establecer la ganancia del angulo actual con respecto al angulo limite establecido como maximo permitible
  
  if(angulo_actual<0 ){              //Si esta inclinado hacia adelante
    potencia = (-1)*55*ganancia/100; //Obtener potencia de los motores, consiste en que sea proporcional a lo inclinado que este el robot
    potencia+=200;                   //Dado que potencia inferior a 200 simplemente no mueve las ruedas, se utiliza de base 200
    if(potencia>255){                //Condicion de borde, dado que la pontencia maxima es 255
      potencia=255;
    }
    //Avanzar
    analogWrite(ENA, potencia); 
    analogWrite(ENB, potencia); 
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(freq);                      //Por caunto tiempo el robot avanza antes de la siguiente medicion, esta definido por la variable frecuencia
  }
  else if(angulo_actual>0 ){          //El mismo algoritmo que el caso anterior, pero para cuando se inclina hacia atras
    potencia = 55*ganancia/100;
    potencia+=200;
    if(potencia>255){
      potencia=255;
    }
    //Retroceder
    analogWrite(ENA, potencia); 
    analogWrite(ENB, potencia);
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW);  
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW);
    delay(freq);
  }   
  else{ //Si el robot se encuentra justo en el angulo 0, dejarlo estatico
    potencia = 0;
    analogWrite(ENA, potencia); 
    analogWrite(ENB, potencia);
    
    digitalWrite(IN1, LOW); // Apagar Motor
    digitalWrite(IN2, LOW); // Apagar Motor
    digitalWrite(IN3, LOW); // Apagar Motor
    digitalWrite(IN4, LOW); // Apagar Motor
    delay(freq);
  }   
}


//------------------------------------------ Movimiento Motores DC---------------------------
void forward(){  //Avanzar con 200 de potencia
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);  
  analogWrite(ENA, 200); 
  analogWrite(ENB, 200);
  delay(2000); 
}

void right(){  //Girar derecha con 200 de potencia
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);  
  analogWrite(ENA, 200); 
  analogWrite(ENB, 200);
  delay(2000);
}

void backward(){   //Retroceder con 200 de potencia
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  analogWrite(ENA, 200); 
  analogWrite(ENB, 200);
  delay(2000); 
}

void left (){    //Girar izquierda con 200 de potencia
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);  
  analogWrite(ENA, 200); 
  analogWrite(ENB, 200);
  delay(2000);
}

void halt(){    //Detenerse
  digitalWrite(IN1, LOW); // Apagar Motor
  digitalWrite(IN2, LOW); // Apagar Motor
  digitalWrite(IN3, LOW); // Apagar Motor
  digitalWrite(IN4, LOW); // Apagar Motor 
  delay(2000);
}


//------------------------------------------ Sensor Inercial ---------------------------

int angulox()
{
  mpu.getAcceleration(&ax, &ay, &az); //Obtener aceleracion del robot
  mpu.getRotation(&gx, &gy, &gz);     //Obtener rotacion del robot
  updateFiltered();                   //Aplicar filtro de Kalman
  delay(10);  
  // Mostrar resultados     
   return ang_x+error;                //Retornar el valor calculado mas el error
}

void updateFiltered() //Filtro de Kalman
{
   dt = (millis() - tiempo_prev) / 1000.0;  //Obtener la diferencia de tiempo respecto a la medicion anterior
   tiempo_prev = millis();                  //Acutalizar la medicion de tiempo anterior para la proxima medicion
 
   //Calcular los ángulos con acelerometro
   float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2)))*(180.0 / 3.14);
   float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2)))*(180.0 / 3.14);
 
   //Calcular angulo de rotación con giroscopio y filtro complementario
   ang_x = 0.98*(ang_x_prev + (gx / 131)*dt) + 0.02*accel_ang_x;
   ang_y = 0.98*(ang_y_prev + (gy / 131)*dt) + 0.02*accel_ang_y;

   //Actualizar los angulos de las variables globales con los de esta medicion
   ang_x_prev = ang_x;
   ang_y_prev = ang_y;
}
