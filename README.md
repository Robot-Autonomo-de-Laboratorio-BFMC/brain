# Nodo de Control Jetson Brain para Comunicación ESP32

Módulo de control UART de alta velocidad para comunicación Jetson-ESP32 en sistemas de vehículos autónomos.

## Resumen

Este módulo implementa un protocolo binario para comunicación de control en tiempo real entre un Jetson (cerebro) y ESP32 (ejecutor). El Jetson maneja la percepción y la toma de decisiones, mientras que el ESP32 ejecuta el control de motores, dirección y otras operaciones de hardware usando FreeRTOS.

# Protocolo de Comunicación ESP32 - Guía para el Equipo Brain

Este documento describe cómo enviar comandos desde el sistema Brain (Jetson/PC) hacia el ESP32 que controla los motores, dirección y luces del vehículo.

## Formato de Comandos

Todos los comandos deben seguir el formato:

```
CHANNEL:COMMAND:VALUE\n
```

- **CHANNEL**: Un solo carácter que identifica el canal
- **COMMAND**: Nombre del comando (mayúsculas)
- **VALUE**: Valor numérico entero
- **Terminación**: Cada comando debe terminar con `\n` (newline)

## Canales Disponibles

### `C` - CONTROL
Comandos de control del vehículo (velocidad, dirección)

### `E` - EMERGENCY  
Comandos de emergencia (frenado inmediato)

### `M` - MANAGEMENT
Comandos de gestión del sistema (armado, modo, etc.)

## Comandos por Canal

### Canal CONTROL (`C`)

#### `C:SET_SPEED:<valor>`
Establece la velocidad del motor de tracción.

- **Valor**: 0-255 (0 = detenido, 255 = máxima velocidad)
- **TTL**: 200ms (el comando expira si no se renueva)
- **Ejemplo**: `C:SET_SPEED:120`

```python
# Ejemplo Python
ser.write(b"C:SET_SPEED:120\n")
```

#### `C:SET_STEER:<valor>`
Establece el ángulo de dirección del servo.

- **Valor**: 50-135 (50 = izquierda máxima, 105 = centro, 135 = derecha máxima)
- **TTL**: 200ms
- **Ejemplo**: `C:SET_STEER:105` (centro)

**⚠️ IMPORTANTE - Conversión de Grados a Valores de Servo:**

Si tu lane detector envía grados (ej: -45° a +45°), debes convertir así:

```python
# Servo range: 50 (izquierda) a 135 (derecha), centro = 105
# Range total: 85 unidades
SERVO_LEFT = 50
SERVO_CENTER = 105
SERVO_RIGHT = 135
SERVO_RANGE = 85  # 135 - 50

def degrees_to_servo(degrees, max_degrees=45):
    """
    Convierte grados de lane detector a valor de servo.
    
    Args:
        degrees: Ángulo en grados (-max_degrees a +max_degrees)
        max_degrees: Máximo ángulo permitido (default: 45°)
    
    Returns:
        Valor de servo (50-135)
    """
    # Normalizar a -1.0 a +1.0
    normalized = degrees / max_degrees
    # Limitar al rango [-1, 1]
    normalized = max(-1.0, min(1.0, normalized))
    # Convertir a valor de servo
    servo_value = SERVO_CENTER + (normalized * (SERVO_RANGE / 2))
    return int(round(servo_value))

# Ejemplos:
# degrees_to_servo(0)    -> 105 (centro)
# degrees_to_servo(-45) -> 50  (izquierda máxima)
# degrees_to_servo(45)  -> 135 (derecha máxima)
# degrees_to_servo(-20) -> ~82 (izquierda suave)
```

**Ejemplo completo con lane detector:**

```python
# En lugar de solo imprimir "turn left" o "turn right":
lane_angle = -25  # grados desde tu detector

# Convertir y enviar:
servo_value = degrees_to_servo(lane_angle)
command = f"C:SET_STEER:{servo_value}\n"
ser.write(command.encode())
```

### Canal EMERGENCY (`E`)

#### `E:BRAKE_NOW:0`
Freno de emergencia inmediato. Detiene el motor instantáneamente (<1ms de respuesta).

- **Valor**: Siempre 0 (ignorado)
- **Ejemplo**: `E:BRAKE_NOW:0`

```python
ser.write(b"E:BRAKE_NOW:0\n")
```

#### `E:STOP:0`
Alias para freno de emergencia (mismo comportamiento que BRAKE_NOW).

### Canal MANAGEMENT (`M`)

#### `M:SYS_ARM:0`
Arma el sistema (prepara para operación).

- **Valor**: Siempre 0
- **TTL**: 5000ms
- **Ejemplo**: `M:SYS_ARM:0`

#### `M:SYS_DISARM:0`
Desarma el sistema (modo seguro).

- **Valor**: Siempre 0
- **TTL**: 5000ms
- **Ejemplo**: `M:SYS_DISARM:0`

#### `M:SYS_MODE:<valor>`
Establece el modo del sistema.

- **Valor**: 0 = MANUAL, 1 = AUTO
- **TTL**: 5000ms
- **Ejemplo**: `M:SYS_MODE:1` (modo AUTO)

## Ejemplos de Uso

### Ejemplo 1: Control Básico

```python
import serial

ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)

# Armar sistema
ser.write(b"M:SYS_ARM:0\n")

# Establecer velocidad
ser.write(b"C:SET_SPEED:150\n")

# Centrar dirección
ser.write(b"C:SET_STEER:105\n")

# Frenar de emergencia
ser.write(b"E:BRAKE_NOW:0\n")
```

### Ejemplo 2: Lane Following

```python
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)

def degrees_to_servo(degrees, max_degrees=45):
    SERVO_CENTER = 105
    SERVO_RANGE = 85
    normalized = max(-1.0, min(1.0, degrees / max_degrees))
    return int(round(SERVO_CENTER + (normalized * (SERVO_RANGE / 2))))

# Loop de control
while True:
    # Obtener ángulo del lane detector
    lane_angle = get_lane_angle()  # Tu función
    
    # Convertir y enviar comando de dirección
    servo_value = degrees_to_servo(lane_angle)
    command = f"C:SET_STEER:{servo_value}\n"
    ser.write(command.encode())
    
    # Mantener velocidad constante
    ser.write(b"C:SET_SPEED:120\n")
    
    time.sleep(0.05)  # 20 Hz update rate
```

### Ejemplo 3: Control con Velocidad Variable

```python
def control_vehicle(speed, steering_degrees):
    """
    Función helper para controlar el vehículo.
    
    Args:
        speed: Velocidad 0-255
        steering_degrees: Ángulo de dirección en grados (-45 a +45)
    """
    # Validar velocidad
    speed = max(0, min(255, int(speed)))
    
    # Convertir dirección
    servo_value = degrees_to_servo(steering_degrees)
    
    # Enviar comandos
    ser.write(f"C:SET_SPEED:{speed}\n".encode())
    ser.write(f"C:SET_STEER:{servo_value}\n".encode())
```

## Consideraciones Importantes

### Time-to-Live (TTL)
Los comandos tienen un tiempo de vida limitado:
- **CONTROL**: 200ms - Debes enviar comandos periódicamente (mínimo 5 Hz)
- **MANAGEMENT**: 5000ms - Comandos de sistema duran más

**Recomendación**: Envía comandos de velocidad y dirección a **10-20 Hz** para mantener el control.

### Last-Writer-Wins
El sistema usa patrón "last-writer-wins". Si envías múltiples comandos rápidamente, solo el último es válido. No hay cola de comandos.

### Respuestas del ESP32
El ESP32 puede enviar mensajes de debug por serial. Puedes leerlos para debugging:

```python
if ser.in_waiting > 0:
    response = ser.readline().decode('utf-8', errors='ignore')
    print(f"ESP32: {response}")
```

### Manejo de Errores
- Si el comando no se parsea correctamente, el ESP32 imprime: `[LinkRxTask] Failed to parse message: ...`
- Verifica que el formato sea exacto: `CHANNEL:COMMAND:VALUE\n`
- Asegúrate de que el baud rate coincida (115200 para USB, 921600 para UART externo)

## Checklist de Integración

- [ ] Configurar conexión serial (USB o UART externo)
- [ ] Implementar función de conversión grados → servo
- [ ] Enviar comandos periódicamente (10-20 Hz)
- [ ] Manejar freno de emergencia en caso de detección de obstáculos
- [ ] Implementar heartbeat/supervisión si es necesario
- [ ] Probar con el simulador UART antes de integrar con lane detector

## Simulador de Pruebas

Puedes probar tus comandos con el simulador incluido:

```bash
cd embedded
pip install -r test/python/requirements.txt
python3 test/python/test_uart_simulator.py /dev/ttyUSB0 --baud 115200
```

Luego prueba comandos como:
- `C:SET_SPEED:120`
- `C:SET_STEER:105`
- `E:BRAKE_NOW:0`

## Soporte

Si tienes dudas sobre el protocolo o encuentras problemas, consulta:
- `embedded/src/link_rx_task.cpp` - Implementación del parser
- `embedded/include/messages.h` - Definiciones de canales y comandos
- `embedded/include/hardware.h` - Valores de servo (SERVO_CENTER, etc.)

## Instalación

```bash
pip install -r requirements.txt
```

## Uso

### Uso Básico

```python
from jetson_control import JetsonControlNode, SystemMode

# Crear nodo de control
node = JetsonControlNode(
    port='/dev/ttyUSB0',
    baudrate=921600,
    control_hz=100.0,      # Actualizaciones de control a 100 Hz
    heartbeat_hz=10.0      # Latido a 10 Hz
)

# Iniciar comunicación
node.start()

# Establecer modo del sistema y armar
node.set_mode(SystemMode.AUTO)
node.arm()

# Actualizar puntos de consigna (se envían automáticamente a 100 Hz)
node.set_control(speed=0.5, steer_angle=10.0)

# Freno de emergencia (se envía inmediatamente)
node.emergency_brake()

# Cierre limpio
node.stop_all()
```

### Pruebas con CLI Interactivo

Hay dos opciones para pruebas interactivas:

#### Opción 1: Tester con Menú (Recomendado)

Interfaz fácil de usar con menús para probar todos los comandos:

```bash
python test_interactive.py --port /dev/ttyUSB0 --baudrate 921600
```

**Características:**
- Menú principal con opciones numeradas
- Submenús para cada tipo de comando (Drive, Steering, Lights, System)
- Entrada de valores numéricos para comandos
- Visualización de estadísticas en tiempo real
- Estado actual del sistema visible en todo momento

#### Opción 2: CLI con Teclado (Para control rápido)

CLI con controles de teclado para pruebas rápidas:

```bash
python test_cli.py --port /dev/ttyUSB0 --baudrate 921600
```

**Controles:**
- `W/S`: Aumentar/Disminuir velocidad
- `A/D`: Aumentar/Disminuir ángulo de dirección
- `M`: Alternar modo AUTO/MANUAL
- `R`: Armar sistema
- `U`: Desarmar sistema
- `L`: Alternar luces
- `E`: Freno de emergencia
- `Q`: Salir
- `H/?`: Mostrar ayuda

### Integración con IA/Planificador

```python
import cv2
from jetson_control import JetsonControlNode, SystemMode

node = JetsonControlNode()
node.start()
node.set_mode(SystemMode.AUTO)
node.arm()

# Bucle principal de percepción/planificación
while True:
    # Tu código de percepción (ej. YOLO, OpenCV)
    frame = camera.read()
    detections = yolo_model(frame)
    
    # Tu código de planificación
    speed, angle = planner.compute_control(detections)
    
    # Enviar comandos de control
    node.set_control(speed=speed, steer_angle=angle)
    
    # Verificar condiciones de emergencia
    if emergency_detected:
        node.emergency_brake()
```

## Arquitectura

### Modelo de Threading

El nodo de control usa múltiples hilos de trabajo:

1. **Send Thread**: Procesa la cola de mensajes y envía por UART
2. **Receive Thread**: Recibe y valida mensajes entrantes
3. **Control Thread**: Envía actualizaciones de control
4. **Heartbeat Thread**

### Cola de Mensajes

Todos los mensajes se encolan y se envían de forma asíncrona para evitar bloqueos. Los mensajes de emergencia se pueden enviar inmediatamente mediante manejo de prioridad.

### Estadísticas

El nodo rastrea:
- Conteo de mensajes enviados/recibidos
- Errores CRC
- Latencia de ida y vuelta (si se implementan mensajes ACK)
- Tamaño de la cola

Acceso mediante `node.get_stats()`.

## Temporización de Mensajes

- **Actualizaciones de Control**: 100 Hz (cada 10 ms)
- **Latido**: 10 Hz (cada 100 ms)
- **Emergencia**: Inmediato (prioridad)
- **Valores TTL**:
  - Control: 100 ms
  - Latido: 200 ms
  - Emergencia: 80 ms

## Cálculo CRC16

El protocolo usa CRC16-XMODEM (polinomio 0x1021) calculado sobre el payload de 13 bytes (desde topic hasta TTL).

## Manejo de Errores

- Validación automática de CRC en mensajes recibidos
- Recuperación de errores del puerto serie
- Protección contra desbordamiento de cola
- Gestión de estado thread-safe

## Estructura de Archivos

```
brain/
├── protocol.py          # Definiciones de protocolo, packing/unpacking, CRC16
├── jetson_control.py    # Implementación principal del nodo de control
├── test_cli.py          # CLI interactivo con controles de teclado
├── test_interactive.py  # Tester interactivo con menús (recomendado)
├── test_protocol.py     # Tests unitarios del protocolo
├── requirements.txt     # Dependencias de Python
└── README.md           # Este archivo
```

## Configuración

### Configuración del Puerto Serie

- **Baud Rate**: 921600 (por defecto, configurable)
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Timeout**: 100 ms (lectura), 1 s (escritura)

### Parámetros Ajustables

- `control_hz`: Frecuencia de actualización de control (por defecto: 100 Hz)
- `heartbeat_hz`: Frecuencia de latido (por defecto: 10 Hz)
- `control_ttl`: TTL para mensajes de control (por defecto: 100 ms)
- `heartbeat_ttl`: TTL para latido (por defecto: 200 ms)
- `emergency_ttl`: TTL para emergencia (por defecto: 80 ms)

## Solución de Problemas

### Puerto Serie No Encontrado

Verificar puertos disponibles:
```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

Asegurar que el usuario tenga permisos:
```bash
sudo usermod -a -G dialout $USER
# Luego cerrar sesión/iniciar sesión
```

### Alta Latencia

- Reducir `control_hz` si el ESP32 no puede procesar a 100 Hz
- Verificar que la velocidad de baudios UART coincida en ambos lados
- Monitorear el tamaño de la cola mediante `get_stats()`

### Errores CRC

- Verificar que la velocidad de baudios coincida con la configuración del ESP32
- Revisar cableado/conexiones
- Asegurar que el ESP32 esté enviando mensajes con formato correcto

## Mejoras Futuras

- [ ] Manejo de mensajes ACK y medición de latencia de ida y vuelta
- [ ] Panel de visualización de telemetría
- [ ] Cola de prioridad de mensajes para comandos de emergencia
- [ ] Reenvío automático del último control si no se recibe actualización
- [ ] Soporte para archivo de configuración
- [ ] Wrapper de integración ROS2

## Licencia

Parte del proyecto Robot Autónomo de Laboratorio BFMC.
