#include "stm32f4xx.h"
#include "Configuracion.h"

Timers T;
USART U;

char A[100];
unsigned int Tiempo=0;
int main(){
U.Comunicacion(2,'A',9600);// USART PA9TX Y PA10RX
T.IniciarContador(2,1,PA0,0);
while(1){
Tiempo=T.GetConteo();
sprintf(A,"Tiempo: %d\n\r",Tiempo);
U.TransmitirDatos(A);
}
}
