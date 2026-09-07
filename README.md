# AstroPilot - Firmware ESP32 (Control de Movimiento Dual-Core)

Firmware de control cinemático y orientación para montura de telescopio altazimutal utilizando ESP32, periférico de conteo por hardware (PCNT) y enlace BLE.

## Características Principales
- **Arquitectura Multinúcleo (FreeRTOS):**
  - **Core 0:** Comunicaciones BLE, parsing de comandos JSON, interfaz Serial y Watchdog.
  - **Core 1:** Lazo de control PID determinista a 100 Hz ($T_s = 10\text{ ms}$).
- **Lectura por Hardware (PCNT):** Decodificación en cuadratura 4x (44.000 pulsos/vuelta) con filtros anti-glitch por hardware.
- **Perfil de Movimiento Trapezoidal:** Aceleración y velocidad máxima limitadas por software para cargas pesadas desbalanceadas (4.7 kg).
- **Compensación de Backlash:** Algoritmo dinámico de doble flanco para compensar la holgura en el eje de Azimut ($5^\circ$).

## Pines de Conexión
| Eje | IN1 (PWM) | IN2 (PWM) | Enc A | Enc B |
| :--- | :---: | :---: | :---: | :---: |
| **Altitud** | GPIO 16 | GPIO 17 | GPIO 25 | GPIO 26 |
| **Azimut**  | GPIO 19 | GPIO 18 | GPIO 32 | GPIO 33 |
