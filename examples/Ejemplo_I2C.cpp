// Ejemplo: I2C - inicializar bus y leer/escribir bytes
// Explica: uso de `IDOSC`, `DatosI2C`, `RecibirI2C` y `TestearDireccion`.
#include "stm32f4xx.h"
#include "Configuracion.h"
#include "Delay.h"

i2c bus;

int main(){
    // Inicializa I2C1 a 100kHz (PB8=SCL, PB9=SDA según implementación)
    bus.IDOSC(1, 100000);

    unsigned char sla = 0x50; // ejemplo de dirección esclavo
    // Testea si hay un dispositivo en la dirección
    if(bus.TestearDireccion(sla)){
        // Escribe un registro (direccion, dato)
        bus.WriteDir(sla, 0x00, 0xA5);
        Delay_ms(10);
        // Lee un byte desde la dirección 0x00
        unsigned char v = bus.ReadDir(sla, 0x00);
        (void)v; // usar el valor leído según necesidad
    }

    while(1){
        Delay_ms(500);
    }
}
