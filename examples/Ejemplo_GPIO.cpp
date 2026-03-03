// Ejemplo:GPIO - configurar pin, escribir y leer, además usar interrupción externa
// Explica: muestra `ModoPin`, `SalidaPin`, `EntradaPin` e `Interrupcion`.
#include "stm32f4xx.h"
#include "Configuracion.h"
#include "Delay.h"

Pines gpio;

// Rutina que se ejecuta con la interrupción del pin (ej: botón)
void ISR_Boton(void){
    // En este ejemplo simple, conmutamos PA5 (LED)
    static int estado = 0;
    estado = !estado;
    gpio.SalidaPin(PA5, estado);
}

int main(){
    // Configura PA5 como salida (LED)
    gpio.ModoPin(PA5, 1); // Modo=1 -> salida
    gpio.SalidaPin(PA5, 0); // Inicialmente apagado

    // Configura PA0 como entrada con interrupción por flanco de subida
    gpio.Interrupcion(ISR_Boton, PA0, 1); // Lectura=1 -> flanco de subida

    // Bucle principal: parpadeo opcional (LED ya toggled por ISR)
    while(1){
        Delay_ms(500);
    }
}
