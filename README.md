# ⚙️ CMSIS-STM32F4xx-Framework
> Librería modular en C++ basada en CMSIS para la familia STM32F4xx (sin HAL ni CubeMX).

---

## 🚀 Descripción
**CMSIS-STM32F4xx-Framework** es una librería orientada a objetos que permite programar microcontroladores **STM32F4xx** directamente con **CMSIS**, sin depender de HAL o CubeMX.  
Incluye control completo de periféricos con código claro, portable y de bajo nivel.

Ideal para quienes buscan:
- Aprender a usar CMSIS de forma práctica.
- Programar con total control sobre los registros.
- Crear proyectos bare-metal con sintaxis C++ limpia y moderna.

---

## 🧩 Características principales
- 🧠 **CMSIS puro**: sin HAL, sin CubeMX, acceso directo a los registros.
- 🔌 **Periféricos incluidos**:
  - GPIO con interrupciones externas.
  - Timers (PWM, Input Capture, Encoder).
  - USART (RX/TX + interrupciones).
  - ADC (conversión simple).
  - I2C (transmisión, recepción y prueba de dispositivos).
  - Reloj del sistema (HCLK, APB1, APB2).
- ⏱️ **Delay por software** en micro y milisegundos.
- 💡 Ejemplos prácticos para PlatformIO o STM32CubeIDE.

---

## 📁 Estructura del proyecto

BareMetal-STM32F4xx/
├── include/ → Headers principales
│ ├── Configuracion.h
│ ├── Delay.h
├── src/ → Implementaciones
│ ├── Configuracion.cpp
│ ├── Delay.cpp
├── examples/ → Ejemplos listos para compilar
│ ├── Blink/
│ ├── PWM_Test/
│ └── UART_Test/
├── LICENSE
└── README.md

## ⚙️ Uso en PlatformIO

Crea un nuevo proyecto para STM32F411RE (Nucleo).

Copia las carpetas include/ y src/ dentro de lib/BareMetal-STM32F4xx/.

Incluye en tu código:

#include "Configuracion.h"
#include "Delay.h"


Compila y sube al microcontrolador.

🧩 También puedes agregar esta librería como submódulo Git para mantenerla actualizada en múltiples proyectos.

## 🧰 Dependencias

CMSIS (ya incluida en el paquete de PlatformIO o STM32CubeIDE)

Compilador ARM-GCC

## 🪪 Licencia

Este proyecto está bajo la licencia MIT, por lo que puedes usarlo libremente en proyectos personales, educativos o comerciales.

## 🤝 Contribuciones

¡Las contribuciones son bienvenidas!
Puedes abrir un Issue o enviar un Pull Request con mejoras, correcciones o nuevos ejemplos.
