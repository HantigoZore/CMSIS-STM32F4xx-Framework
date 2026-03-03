// Ejemplo: Timers y PWM
// Explica: inicializar un PWM en un timer, ajustar ciclo de trabajo con `CicloUtil`
#include "stm32f4xx.h"
#include "Configuracion.h"
#include "Delay.h"

Timers T;

int main(){
    // Inicializa PWM en TIM3, puerto B, habilitando canal 1 (CH1)
    // Periodo de referencia (ejemplo): 1000 -> ejemplo de 1 kHz según implementación
    T.PWM(3, 'B', 1, 0, 0, 0, 1000); // Timer 3, puerto B, canal 1, sin remapeo, sin polaridad invertida, sin preescaler, periodo=1000

    // Barrido del ciclo de trabajo 0..100%
    while(1){
        for(int d=0; d<=100; d+=5){
            T.CicloUtil(3, 1, d); // Timer 3, canal 1
            Delay_ms(50);
        }
        for(int d=100; d>=0; d-=5){
            T.CicloUtil(3, 1, d);
            Delay_ms(50);
        }
    }
}
