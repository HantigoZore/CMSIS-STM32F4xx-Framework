// Archivo *.h //
#ifndef _DELAY_H
#define _DELAY_H
#include "stm32f4xx.h"
// Funcion para iniciar servicios de los retardos
void IniciarDelay(void);
void Delay_us(unsigned int t); // Retardo en microsegundos
void Delay_ms(unsigned int t); // Retardo en milisegundos
void delay(unsigned int t); // Retardo en milisegundos
void delayMicroseconds(unsigned int t); // Retardo en microsegundos
#endif

