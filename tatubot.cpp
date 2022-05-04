#include <ArduinoQueue.h>
#include "tatubot.h"
#include "LedControl.h"     
#include "caritas.h"

Motor motor0(IN1, IN2, ENA);
Motor motor1(IN4, IN3, ENB);
Led led1(PIN_R_A,PIN_G_A,PIN_B_A);
Led led2(PIN_R_B,PIN_G_B,PIN_B_B);
ArduinoQueue<int> colaOrdenes(CANTIDAD_ORDENES);
LedControl matrizLeds=LedControl(DIN,CLK,CS,MATRICES);

bool contador0EnHigh = false;
bool contador1EnHigh = false;
int contador0;
int contador1;
int contadorDeUnosDelSensor0;
int  contadorDeCerosDelSensor1;
int  contadorDeCerosDelSensor0;
int  contadorDeUnosDelSensor1;
float VELOCIDAD_MOTOR0 = 75;
float VELOCIDAD_MOTOR1 = 75; 
float VELOCIDAD_FRENO0 = (VELOCIDAD_MOTOR0 - 70)/180; //(Velocidad final - velocidad minima(PWM)) / ciclos del bucle
float VELOCIDAD_FRENO1 = (VELOCIDAD_MOTOR1  - 70)/180;

void Robot::iniciar() {
  pinMode(pinEscucharOrdenes, INPUT_PULLUP);
  pinMode(pinAvanzar, INPUT_PULLUP);
  pinMode(pinReversa, INPUT_PULLUP);
  pinMode(pinGiroDerecha, INPUT_PULLUP);
  pinMode(pinGiroIzquierda, INPUT_PULLUP);
  pinMode(pinFinOrdenes, INPUT_PULLUP);
  pinMode(pinReset, INPUT_PULLUP);
  pinMode(SENSOR0, INPUT);
  pinMode(SENSOR1, INPUT);
  Serial.begin(115200);
  motor0.iniciar();
  motor1.iniciar();
  led1.iniciar();
  led2.iniciar();
  matrizLeds.shutdown(0,false);     
  matrizLeds.setIntensity(0,4);    
  matrizLeds.clearDisplay(0);    
  matrizLeds.shutdown(1,false);    
  matrizLeds.setIntensity(1,4);    
  matrizLeds.clearDisplay(1);    
}

void Robot::avanzar(int velocidad, int velocidad2) {
  contador0 = 0;
  contador1 = 0;
  movimientoAdelante(velocidad, velocidad2);
  //frenarAvance(velocidad, velocidad2);
  //autocorregir();
}

void Robot::retroceder(int velocidad, int velocidad2) {
  contador0 = 0;
  contador1 = 0;
  movimientoReversa(velocidad, velocidad2);
  frenarReversa(velocidad, velocidad2);
  //autocorregirReversa();
}

void Robot::girarDerecha() {
  contador0 = 0;
  contador1 = 0;
  movimientoDerecha();
}

void Robot::girarIzquierda() {
  contador0 = 0;
  contador1 = 0;
  movimientoIzquierda();
}

void Robot:: movimientoAdelante(int velocidad, int velocidad2)
{
  /*for(int ciclo = 0; ciclo < 120; ciclo++)
  {
      motor0.adelante(velocidad);
      motor1.adelante(velocidad2);
      led1.rgb('g');
      led2.rgb('g');
      contador();
      //restarleUnoAlContadorMayor();
      //autocorregir();
      delay (5);
  }*/
  contadorDeUnosDelSensor0 = 0;
  contadorDeCerosDelSensor1 = 0;
  contadorDeCerosDelSensor0 = 0;
  contadorDeUnosDelSensor1 = 0;
  
  
  int estadoSensor0 = digitalRead(SENSOR0);
  int estadoSensor1 = digitalRead(SENSOR1);
  if (estadoSensor0 == estadoSensor1)
  {
    while(contador0 <= 20 && contador1 <= 20)
    {
        motor0.adelante(velocidad);
        motor1.adelante(velocidad2);
        led1.rgb('g');
        led2.rgb('g');
        contador();
        
        
    }
    pararRobot();
    Serial.println("Estoy dentro del while donde el estado de los sensores están iguales");
    Serial.print(contador0);
    Serial.print(" ");
    Serial.println(contador1);
  }
  else if(estadoSensor0 > estadoSensor1)
  {
    while(contadorDeUnosDelSensor0 <= 20 && contadorDeCerosDelSensor1 <= 20)
    {
        motor0.adelante(velocidad);
        motor1.adelante(velocidad2);
        led1.rgb('g');
        led2.rgb('g');
        contadorDeCerosDelSensor_(SENSOR1);
        contadorDeUnosDelSensor_(SENSOR0);
        
    }
    pararRobot();
    Serial.println("Estoy dentro del while donde el estado del sensor0 =1");
    Serial.print(contadorDeUnosDelSensor0);
    Serial.print(" ");
    Serial.println(contadorDeCerosDelSensor1);
  }
    else if(estadoSensor0 < estadoSensor1)
  {
    while(contadorDeCerosDelSensor0 <= 20 && contadorDeUnosDelSensor1 <= 20)
    {
        motor0.adelante(velocidad);
        motor1.adelante(velocidad2);
        led1.rgb('g');
        led2.rgb('g');
        contadorDeCerosDelSensor_(SENSOR0);
        contadorDeUnosDelSensor_(SENSOR1);
        
        
    }
    pararRobot();
    Serial.println("Estoy dentro del while donde el estado del sensor1 =1");
    Serial.print(contadorDeCerosDelSensor0);
    Serial.print(" ");
    Serial.println(contadorDeUnosDelSensor1);
  }
}

void Robot::contadorDeCerosDelSensor_(int sensor){
  
  if (digitalRead(sensor) == HIGH && contador1EnHigh == false)  
  {
      contadorUnosYCeros(sensor); 
      contador1EnHigh = true;
  }
  if (digitalRead(sensor) == LOW)
  {
      contador1EnHigh = false;
  }
 }
  
void Robot::contadorDeUnosDelSensor_(int sensor){
  if (digitalRead(sensor) == LOW && contador0EnHigh == false )
  {
      contadorUnosYCeros(sensor);  
      contador0EnHigh = true;
  }
  if (digitalRead(SENSOR0) == HIGH)
  {
       contador0EnHigh = false;
  }
 }
  
void Robot::contadorUnosYCeros(int sensor){

  if (sensor == SENSOR0 && digitalRead(SENSOR0)== HIGH){contadorDeCerosDelSensor0++;}
  if (sensor == SENSOR0 && digitalRead(SENSOR0)== LOW){contadorDeUnosDelSensor0++;}
  if (sensor == SENSOR1 && digitalRead(SENSOR1)== HIGH){contadorDeCerosDelSensor1++;}
  if (sensor == SENSOR1 && digitalRead(SENSOR1)== LOW){contadorDeUnosDelSensor1++;}
  
}

void Robot:: movimientoReversa(int velocidad, int velocidad2)
{
  for(int ciclo = 0; ciclo < 120; ciclo++)
  {
      motor0.atras(velocidad);
      motor1.atras(velocidad2);
      led1.rgb('r');
      led2.rgb('r');
      contador();
      //autocorregirReversa();
      delay (5);
  }
}

void Robot:: movimientoDerecha()
{
  // led 1 izq mirando de atras
  while (contador1 < 15) {
      motor0.atras(85);
      motor1.adelante(85);
      delay(15);
      led1.rgb('b');
      contador();
  }
  pararRobot();
  led1.rgb('a');
}

void Robot:: movimientoIzquierda()
{
  // led 2 derecha mirando de atras
  while (contador0 < 15) {
      motor0.adelante(85);
      motor1.atras(85);
      delay(15);
      led2.rgb('b');
      contador();
      Serial.println("contador0:");
      Serial.println(contador0);
      Serial.println("contador 1:");
      Serial.println(contador1);
    }
  pararRobot();
  led2.rgb('a');
}

void Robot::restarleUnoAlContadorMayor() {
  if (contador0 > contador1) {
    contador0 --;
  }
  if (contador1 > contador0) {
    contador1 --;
  }
}

void Robot::frenarAvance(float velocidad, float velocidad2) {
  for(int ciclo = 0; ciclo < 180; ciclo++) {
    velocidad = velocidad - VELOCIDAD_FRENO0; 
    velocidad2 = velocidad2 - VELOCIDAD_FRENO1;
    motor0.adelante(velocidad);
    motor1.adelante(velocidad2);
    contador();
    restarleUnoAlContadorMayor();
    autocorregir();
    delay (5);
    led1.rgb('g');
    led2.rgb('g');
  }
  //autocorregir();
  delay (5);
  pararRobot();
  led1.rgb('a');
  led2.rgb('a');
}

void Robot::frenarReversa(float velocidad, float velocidad2) {
  for(int ciclo = 0; ciclo < 180; ciclo++) {
    velocidad = velocidad - VELOCIDAD_FRENO0; 
    velocidad2= velocidad2 - VELOCIDAD_FRENO1;
    motor0.atras(velocidad);
    motor1.atras(velocidad2);
    contador();
    //autocorregirReversa();
    delay (5);
    led1.rgb('r');
    led2.rgb('r');
  }
  pararRobot();
  led1.rgb('a');
  led2.rgb('a');
}

void Robot::pararRobot() {
  motor0.parar();
  motor1.parar();
  led1.rgb('a');
  led2.rgb('a');
}

void Robot::autocorregir() {
 if (contador0 > contador1) {
  igualarRuedaAdelantada0();
 }
 else if (contador1 > contador0) {
  igualarRuedaAdelantada1();
 }
}

void Robot::igualarRuedaAdelantada0() 
{
  while (contador1 != contador0) {
    motor0.parar();
    motor1.adelante(75);
    contador();
  }
}

void Robot::igualarRuedaAdelantada1() 
{
  while (contador0 != contador1) {
    motor1.parar();
    motor0.adelante(75);
    contador();
  }
}

void Robot::autocorregirReversa() {
 if (contador0 > contador1) {
  igualarRuedaAdelantadaReversa0();
 }
 else if (contador1 > contador0) {
  igualarRuedaAdelantadaReversa1();
 }
}

void Robot::igualarRuedaAdelantadaReversa0() 
{
  while (contador0 != contador1) {
    motor0.parar();
    motor1.atras(85);
    contador();
  }
}

void Robot::igualarRuedaAdelantadaReversa1() 
{
  while (contador1 != contador0) {
    motor1.parar();
    motor0.atras(85);
    contador();
  }
}


void Robot::contador() {
  if (digitalRead(SENSOR1) == LOW && contador1EnHigh == false)  
  {
      contador1++; 
      contador1EnHigh = true;
  }
  if (digitalRead(SENSOR1) == HIGH)
  {
      contador1EnHigh = false;
  }
  if (digitalRead(SENSOR0) == LOW && contador0EnHigh == false )
  {
      contador0++;  
      contador0EnHigh = true;
  }
  if (digitalRead(SENSOR0) == HIGH)
  {
       contador0EnHigh = false;
  }
}



void Robot::escucharOrdenes() {
  while(colaOrdenes.itemCount() <= CANTIDAD_ORDENES && digitalRead(pinFinOrdenes) != LOW) {
    if (digitalRead(pinAvanzar) == LOW) {
          dibujarCaritaSorprendida();          
          colaOrdenes.enqueue(1);
          led1.rgb('g');
          led2.rgb('g');
          Serial.print("Orden para avanzar en cola \n");
          delay(500);
          led1.rgb('a');
          led2.rgb('a');
          dibujarCaritaFeliz();      
    }
    if (digitalRead(pinReversa) == LOW) {
          dibujarCaritaSorprendida();           
          colaOrdenes.enqueue(2);
          led1.rgb('r');
          led2.rgb('r');
          Serial.print("Orden para retroceder en cola \n");
          delay(500);
          led1.rgb('a');
          led2.rgb('a');
          dibujarCaritaFeliz(); 
    }
    if (digitalRead(pinGiroDerecha) == LOW) {  
          dibujarCaritaSorprendida();         
          colaOrdenes.enqueue(3);
          led2.rgb('b');
          Serial.print("Orden para girar a la derecha en cola \n");
          delay(500);
          led2.rgb('a');
          dibujarCaritaFeliz(); 
    }
    if (digitalRead(pinGiroIzquierda) == LOW) {    
          dibujarCaritaSorprendida();       
          colaOrdenes.enqueue(4);
          led1.rgb('b');
          Serial.print("Orden para girar a la izquierda en cola \n");
          delay(500);
          led1.rgb('a');
          dibujarCaritaFeliz(); 
    }
   if (digitalRead (pinReset) == LOW) {
    while(colaOrdenes.itemCount() > 0)
    {
      colaOrdenes.dequeue();
    }
    Serial.print("resetee todas las ordenes");
  }
  } 
  
  if(colaOrdenes.itemCount() < 3) {
        Serial.print("Se enoja por cancelar antes de las 3 ordenes");
        dibujarCaritaEnojada();
        delay(500);
  }
 
  Serial.print("Se ejecutan las ordenes \n");
  Serial.print(colaOrdenes.itemCount());
  ejecutarTodasLasOrdenes();
}

void Robot::ejecutarTodasLasOrdenes() {
  int cantidadOrdenes = colaOrdenes.itemCount();
  delay(1000);
  for(int i=0; i<cantidadOrdenes; i++) {
        Serial.print(colaOrdenes.itemCount());
        dibujarCaritaEntusiasmada();
        
        ejecutarOrden(colaOrdenes.dequeue());
        dibujarCaritaFeliz();
        delay(1000);
  }
}


void Robot::ejecutarOrden(byte caso) {
  switch(caso) {
    case 1:
      avanzar(VELOCIDAD_MOTOR0,VELOCIDAD_MOTOR1);
      break;
    case 2:
      retroceder(VELOCIDAD_MOTOR0,VELOCIDAD_MOTOR1);
      break;
    case 3:
      girarDerecha();
      break;
    case 4:
      girarIzquierda();
      break;
  }
}
void Robot::dibujarCarita(byte ojos[8], byte boca[8]) {     
  for (int i = 0; i < 8; i++)     
  {
    matrizLeds.setRow(0,i,ojos[i]);   
  }
  for (int i = 0; i < 8; i++)     
  {
    matrizLeds.setRow(1,i,boca[i]);    
  }
}


void Robot::dibujarCaritaFeliz() { dibujarCarita(ojoFeliz,  ojoFeliz); }
void Robot::dibujarCaritaSorprendida() { dibujarCarita(ojoDefault,  ojoDefault); }
void Robot::dibujarCaritaEntusiasmada() { dibujarCarita(ojoCorazonCabeza,  ojoCorazonCabeza); }
void Robot::dibujarCaritaTriste() { dibujarCarita(ojoCerrado,  ojoCerrado); }
void Robot::dibujarCaritaEnojada() { dibujarCarita(ojoEnojadoDerCabeza,  ojoEnojadoIzqCabeza); }
void Robot::dibujarCaritaGuiniando() { dibujarCarita(ojoFeliz,  ojoCerrado); }
void Robot::despertar() {
  dibujarCarita(ojoCerrado,  ojoCerrado);
  delay(300);
  dibujarCarita(ojoDefault,  ojoDefault);
  delay(500);
  dibujarCarita(ojoCerrado,  ojoCerrado);
  delay(300);
  dibujarCarita(ojoFeliz,  ojoFeliz);
  delay(1000);
  dibujarCaritaGuiniando();
  delay(250);
  dibujarCaritaFeliz();
  delay(500);
}

//---------------------------------------------
Motor::Motor(byte pinAtras, byte pinAdelante, byte pinVelocidad) {
  this->pinAtras = pinAtras;
  this->pinAdelante = pinAdelante;
  this->pinVelocidad = pinVelocidad;
}

void Motor::iniciar() {
  pinMode(pinAtras, OUTPUT);
  pinMode(pinAdelante, OUTPUT);
  pinMode(pinVelocidad, OUTPUT);
  this->parar();
}

void Motor::adelante(int velocidad) {
  analogWrite(pinVelocidad, velocidad);
  digitalWrite(pinAtras, LOW);
  digitalWrite(pinAdelante, HIGH);
}

void Motor::atras(int velocidad) {
  analogWrite(pinVelocidad, velocidad);
  digitalWrite(pinAdelante, LOW);
  digitalWrite(pinAtras, HIGH);
}

void Motor::parar() {
  analogWrite(pinVelocidad, 0);
}


//-------------------------------------------------------
Led::Led(byte ledR, byte ledG, byte ledB) {
  this->ledR = ledR;
  this->ledG = ledG;
  this->ledB = ledB;
}

void Led::iniciar() {
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  this->rgb('a');
}


void Led::rgb(char color) {
  switch (color) {
    case 'r': //red
      digitalWrite(ledR, LOW);
      digitalWrite(ledG, HIGH);
      digitalWrite(ledB, HIGH);
      break;
    case 'g': //green
      digitalWrite(ledR, HIGH);
      digitalWrite(ledG, LOW);
      digitalWrite(ledB, HIGH);
      break;
    case 'y': //yellow
      digitalWrite(ledR, LOW);
      digitalWrite(ledG, LOW);
      digitalWrite(ledB, HIGH);
      break;
    case 'b': //blue
      digitalWrite(ledR, HIGH);
      digitalWrite(ledG, HIGH);
      digitalWrite(ledB, LOW);
      break;
    case 'a': //apagado
      digitalWrite(ledR, HIGH);
      digitalWrite(ledG, HIGH);
      digitalWrite(ledB, HIGH);
      break;
  }
}
