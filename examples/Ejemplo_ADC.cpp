#include "stm32f4xx.h"
#include "Configuracion.h"
#include "Delay.h"

Analogo A;
USART   U;

double Dato=0,
char A[100];

int main(void){
    // Inicializa USART2 en el puerto A a 57600 baudios
  U.Comunicacion(2,'A',57600);
  A.Conversion(PC1); // Configura el pin PC1 para lectura analógica (ejemplo, ajustar según necesidad)

  while(1){
    A.IniciarADC();
    Dato=(ADC1->DR*3.3)/4095;  // Convierte el valor leído a voltaje (ejemplo para referencia de 3.3V y resolución de 12 bits)
    sprintf(A,"Dato: %f\n\r",Dato); // Convierte el valor a string para transmitir por USART
    U.TransmitirDatos(A); // Transmite el dato convertido por USART y se puede ver en el COMX
    Delay_ms(1000);
  }
}