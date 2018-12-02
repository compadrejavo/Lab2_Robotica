char estado = '0';  //variable que recibe las instrucciones del celular

int ENA = 6; // MCU PWM Pin 10 to ENA on L298n Board
int IN1 = 7;  // MCU Digital Pin 9 to IN1 on L298n Board 
int IN2 = 8;  // MCU Digital Pin 8 to IN2 on L298n Board 
int IN3 = 9;  // MCU Digital pin 7 to IN3 on L298n Board
int IN4 = 10;  // MCU Digital pin 6 to IN4 on L298n Board
int ENB = 11;  // MCU PWM Pin 5 to ENB on L298n Board


 
void setup() 
{
  Serial.begin(9600);    
  pinMode(ENA, OUTPUT); //Set all the L298n Pin to output
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
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
    }//Fin switch  
  }
  delay(100);
}



//------------------------------------------ Movimiento Motores DC---------------------------
void forward(){  
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 255);  // velocidad 0 a 255
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);  
  analogWrite(ENB, 255); // velocidad 0 a 255
}

void right(){  
  analogWrite(ENA, 255); // velocidad 0 a 255
  analogWrite(ENB, 255);
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); 
  delay(10);
}

void backward(){   
  analogWrite(ENA, 255); //Rueda izquierda
  analogWrite(ENB, 255); //Rueda derecha
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);   
}

void left (){ 
  analogWrite(ENA, 255); 
  analogWrite(ENB, 255);
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  delay(10);
}

void halt(){ 
  digitalWrite(IN1, LOW); // Apagar Motor
  digitalWrite(IN2, LOW); // Apagar Motor
  digitalWrite(IN3, LOW); // Apagar Motor
  digitalWrite(IN4, LOW); // Apagar Motor
}
