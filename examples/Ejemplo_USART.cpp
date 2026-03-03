// Ejemplo básico: Transmitir texto por USART cada segundo
// Muestra cómo inicializar un puerto USART y enviar un string
#include "stm32f4xx.h"
#include "Configuracion.h"
#include "Delay.h"

USART serial;

int main(){
	// Inicializa USART2 en el puerto A a 9600 baudios
	serial.Comunicacion(2, 'A', 9600);
	while(1){
		serial.TransmitirDatos("Ejemplo1: Hola desde STM32 CMSIS\r\n");
		Delay_ms(1000);
	}
}
