# ⚙️ CMSIS-STM32F4xx-Framework

> **Librería modular en C++ basada en CMSIS para la familia STM32F4xx.** > *Desarrollo Bare-Metal puro: sin HAL, sin CubeMX, control total.*

---

## 🚀 Descripción

**CMSIS-STM32F4xx-Framework** es una solución orientada a objetos diseñada para desarrolladores que buscan exprimir al máximo el rendimiento de los microcontroladores **STM32F4xx**. Al eliminar las capas de abstracción pesadas (HAL), obtienes un código más ligero, rápido y predecible.

### ✨ ¿Por qué usar este framework?
* **Aprendizaje profundo:** Entiende qué pasa realmente en los registros del procesador.
* **Eficiencia:** Código optimizado con la mínima huella de memoria.
* **Sintaxis Moderna:** El poder de C++ (clases y objetos) aplicado al bajo nivel.

---

## 🛠️ Características Principales

| Módulo | Descripción |
| :--- | :--- |
| 🧠 **CMSIS Core** | Acceso directo a registros sin intermediarios. |
| 📍 **GPIO** | Configuración dinámica e interrupciones externas (EXTI). |
| ⏱️ **Timers** | Soporte para PWM, Input Capture y modo Encoder. |
| 🛰️ **USART** | Comunicación serial asíncrona con soporte para interrupciones. |
| 📊 **ADC** | Conversiones analógicas precisas y rápidas. |
| 🔗 **I2C** | Protocolo maestro para sensores y periféricos externos. |
| 🕰️ **System Clock** | Gestión de HCLK, APB1 y APB2. |

---

## 📂 Estructura del Proyecto

```text
CMSIS-STM32F4xx-Framework/
├── include/              # Cabeceras (.h)
│   ├── Configuracion.h   # Clase principal de periféricos
│   └── Delay.h          # Gestión de tiempos
├── src/                  # Implementación (.cpp)
│   ├── Configuracion.cpp
│   └── Delay.cpp
├── examples/             # Snippets listos para usar
│   └── ...
├── LICENSE               # Licencia MIT
└── README.md

```

---

## 🚦 Guía de Inicio Rápido

### 1. Instalación en PlatformIO

Si usas **PlatformIO** (recomendado), sigue estos pasos:

1. Crea un proyecto para tu placa (ej: `Nucleo-F411RE`).
2. Copia las carpetas `include/` y `src/` dentro del directorio `lib/CMSIS-Framework/` de tu proyecto.
3. En tu `main.cpp`, incluye los módulos necesarios:

```cpp
#include "Configuracion.h"
#include "Delay.h"

```

### 2. Uso de Git Submodules

Para mantener la librería actualizada en varios proyectos:

```bash
git submodule add [https://github.com/tu-usuario/CMSIS-STM32F4xx-Framework.git](https://github.com/tu-usuario/CMSIS-STM32F4xx-Framework.git) lib/CMSIS-Framework

```

---

## 📚 Ejemplos de Uso

### 💡 GPIO & Blink

Control sencillo de pines digitales.

```cpp
Pines led;

int main() {
    led.ModoPin(PA5, 1); // PA5 como salida
    while (1) {
        led.SalidaPin(PA5, 1); 
        Delay_ms(500);
        led.SalidaPin(PA5, 0);
        Delay_ms(500);
    }
}

```

### 📡 Comunicación USART

Envío de datos seriales a 9600 baudios.

```cpp
USART serial;

int main() {
    serial.Comunicacion(2, 'A', 9600); // USART2, Puerto A
    while (1) {
        serial.TransmitirDatos("Hola STM32 CMSIS!\r\n");
        Delay_ms(1000);
    }
}

```

### 🌊 Control de PWM

Ideal para servos o control de intensidad LED.

```cpp
Timers timer;

int main() {
    // Timer3, Canal 1, Frecuencia 1kHz
    timer.PWM(3, 'B', 1, 0, 0, 0, 1000); 
    while (1) {
        for (int duty = 0; duty <= 100; duty += 5) {
            timer.CicloUtil(3, 1, duty);
            Delay_ms(100);
        }
    }
}

```

<details>
<summary><b>Ver más ejemplos (I2C, ADC, Interrupciones)</b></summary>

#### Interrupciones EXTI

```cpp
Pines p;
void ISR(void) { p.SalidaPin(PA5, 1); }

int main() {
    p.Interrupcion(ISR, PA0, 1);
    while(1);
}

```

#### Lectura ADC

```cpp
Analogo adc;
int main() { 
    adc.Conversion(PA0); 
    adc.IniciarADC(); // El resultado se lee en ADC1->DR
}

```

</details>

---

## 📋 Requisitos

* **Toolchain:** `ARM-GCC`
* **Entorno:** PlatformIO, STM32CubeIDE o Makefile propio.
* **Librería:** CMSIS (usualmente proveída por el IDE).

---

## 🤝 Contribuciones

¡Las ideas son bienvenidas!

1. Haz un **Fork** del proyecto.
2. Crea una rama para tu mejora (`git checkout -b feature/MejoraIncreible`).
3. Haz un **Commit** (`git commit -m 'Añadida nueva funcionalidad'`).
4. Haz **Push** (`git push origin feature/MejoraIncreible`).
5. Abre un **Pull Request**.

---

## ⚖️ Licencia

Este proyecto está bajo la **Licencia MIT**. Siéntete libre de usarlo en tus proyectos personales o comerciales.

---
