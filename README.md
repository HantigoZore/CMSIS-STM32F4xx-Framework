# ⚙️ CMSIS-STM32F4xx-Framework

> **Modular C++ library based on CMSIS for the STM32F4xx family.**
> *Pure Bare-Metal development: no HAL, no CubeMX, full control.*

---

## 🚀 Description

**CMSIS-STM32F4xx-Framework** is an object-oriented solution designed for developers who want to maximize the performance of **STM32F4xx** microcontrollers. By removing heavy abstraction layers (HAL), you get lighter, faster, and more predictable code.

### ✨ Why use this framework?

* **Deep Learning:** Understand what is really happening at the processor register level.
* **Efficiency:** Optimized code with minimal memory footprint.
* **Modern Syntax:** The power of C++ (classes and objects) applied to low-level programming.

---

## 🛠️ Main Features

| Module               | Description                                               |
| :------------------- | :-------------------------------------------------------- |
| 🧠 **CMSIS Core**    | Direct register access without intermediaries.            |
| 📍 **GPIO**          | Dynamic configuration and external interrupts (EXTI).     |
| ⏱️ **Timers**        | Support for PWM, Input Capture, and Encoder mode.         |
| 🛰️ **USART**        | Asynchronous serial communication with interrupt support. |
| 📊 **ADC**           | Fast and precise analog conversions.                      |
| 🔗 **I2C**           | Master protocol for sensors and external peripherals.     |
| 🕰️ **System Clock** | Management of HCLK, APB1, and APB2.                       |

---

## 📂 Project Structure

```text
CMSIS-STM32F4xx-Framework/
├── include/              # Headers (.h)
│   ├── Configuracion.h   # Main peripheral class
│   └── Delay.h           # Timing management
├── src/                  # Implementation (.cpp)
│   ├── Configuracion.cpp
│   └── Delay.cpp
├── examples/             # Ready-to-use snippets
│   └── ...
├── LICENSE               # MIT License
└── README.md
```

---

## 🚦 Quick Start Guide

### 1. Installation with PlatformIO

If you are using **PlatformIO** (recommended), follow these steps:

1. Create a project for your board (e.g., `Nucleo-F411RE`).
2. Copy the `include/` and `src/` folders into the `lib/CMSIS-Framework/` directory of your project.
3. Include the required modules in your `main.cpp`:

```cpp
#include "Configuracion.h"
#include "Delay.h"
```

---

### 2. Using Git Submodules

To keep the library updated across multiple projects:

```bash
git submodule add https://github.com/your-username/CMSIS-STM32F4xx-Framework.git lib/CMSIS-Framework
```

---

## 📚 Usage Examples

### 💡 GPIO & Blink

Basic digital pin control.

```cpp
Pines led;

int main() {
    led.ModoPin(PA5, 1); // PA5 as output
    while (1) {
        led.SalidaPin(PA5, 1); 
        Delay_ms(500);
        led.SalidaPin(PA5, 0);
        Delay_ms(500);
    }
}
```

---

### 📡 USART Communication

Send serial data at 9600 baud.

```cpp
USART serial;

int main() {
    serial.Comunicacion(2, 'A', 9600); // USART2, Port A
    while (1) {
        serial.TransmitirDatos("Hello STM32 CMSIS!\r\n");
        Delay_ms(1000);
    }
}
```

---

### 🌊 PWM Control

Ideal for servos or LED brightness control.

```cpp
Timers timer;

int main() {
    // Timer3, Channel 1, Frequency 1kHz
    timer.PWM(3, 'B', 1, 0, 0, 0, 1000); 
    while (1) {
        for (int duty = 0; duty <= 100; duty += 5) {
            timer.CicloUtil(3, 1, duty);
            Delay_ms(100);
        }
    }
}
```

---

<details>
<summary><b>See more examples (I2C, ADC, Interrupts)</b></summary>

#### EXTI Interrupts

```cpp
Pines p;
void ISR(void) { p.SalidaPin(PA5, 1); }

int main() {
    p.Interrupcion(ISR, PA0, 1);
    while(1);
}
```

#### ADC Reading

```cpp
Analogo adc;
int main() { 
    adc.Conversion(PA0); 
    adc.IniciarADC(); // Result available in ADC1->DR
}
```

</details>

---

## 📋 Requirements

* **Toolchain:** `ARM-GCC`
* **Environment:** PlatformIO, STM32CubeIDE, or custom Makefile
* **Library:** CMSIS (usually provided by the IDE)

---

## 🤝 Contributions

Contributions are welcome!

1. Fork the repository
2. Create a new branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add new feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## ⚖️ License

This project is licensed under the **MIT License**.
Feel free to use it in personal or commercial projects.

---
