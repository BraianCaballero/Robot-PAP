/*
  Programa para manejar el robot Tatu-bot de Futhur Tech
 */
#include "tatubot.h"
Robot tatubot;

void setup(){
  tatubot.iniciar();
  tatubot.despertar();
}

void loop(){
  if (digitalRead(PIN_INICIO_ORDENES) == LOW) {
    Serial.print("Escuchando ordenes \n");
    tatubot.dibujarCaritaEntusiasmada();         
    tatubot.escucharOrdenes();
  }
  if (digitalRead(PIN_ORDEN_AVANZAR) == LOW) {          
    tatubot.dibujarCaritaFeliz();
  }
  if (digitalRead(PIN_ORDEN_REVERSA) == LOW) {          
    tatubot.dibujarCaritaEntusiasmada(); 
  }
  if (digitalRead(PIN_ORDEN_GIRO_DERECHA) == LOW) {          
    tatubot.dibujarCaritaEnojada();
  }
  if (digitalRead(PIN_ORDEN_GIRO_IZQUIERDA) == LOW) {          
    tatubot.dibujarCaritaSorprendida();
  }
  if (digitalRead(PIN_FIN_ORDENES) == LOW) {          
    tatubot.dibujarCaritaTriste();
  }
}
