# Guía de Uso - Detección de Carriles y Seguimiento

Esta guía explica cómo usar los diferentes scripts disponibles para detección de carriles y control del vehículo.

## Scripts Disponibles

### 1. `deteccion_carril.py` - Solo Detección (Sin Control UART)

Solo detecta carriles y muestra la visualización. No controla el vehículo.

#### Uso Básico (con ventanas OpenCV):
```bash
cd detector_lineas
python deteccion_carril.py
```

#### Con cámara específica:
```bash
python deteccion_carril.py --camera 4
```

#### Con web streaming (sin ventanas OpenCV):
```bash
python deteccion_carril.py --web-stream
```

#### Con web streaming en puerto personalizado:
```bash
python deteccion_carril.py --web-stream --web-port 8080
```

#### Sin visualización (headless):
```bash
python deteccion_carril.py --no-display
```

**Nota:** Cuando usas `--web-stream`, las ventanas de OpenCV se deshabilitan automáticamente. Solo se usa el servidor web.

---

### 2. `lane_follower.py` - Detección + Control UART

Detecta carriles y controla el vehículo automáticamente mediante UART.

#### Uso Básico (selección interactiva de puerto):
```bash
cd detector_lineas
python lane_follower.py --uart-interactive --arm-system
```

#### Con puerto específico:
```bash
# Linux/WSL
python lane_follower.py --uart-port /dev/ttyUSB0 --arm-system

# Windows
python lane_follower.py --uart-port COM5 --arm-system
```

#### Con velocidad personalizada:
```bash
python lane_follower.py --uart-port /dev/ttyUSB0 --arm-system --speed 200
```

#### Con modo automático:
```bash
python lane_follower.py --uart-port /dev/ttyUSB0 --arm-system --mode auto --speed 180
```

#### Con web streaming (sin ventanas OpenCV):
```bash
python lane_follower.py --uart-port /dev/ttyUSB0 --arm-system --web-stream
```

#### Con web streaming en puerto personalizado:
```bash
python lane_follower.py --uart-port /dev/ttyUSB0 --arm-system --web-stream --web-port 8080
```

#### Sin visualización (headless):
```bash
python lane_follower.py --uart-port /dev/ttyUSB0 --arm-system --no-display
```

---

## Parámetros Comunes

### Parámetros de Cámara
- `--camera <número>`: Índice de la cámara (ej: `--camera 0`, `--camera 4`)
- Si no se especifica, se selecciona automáticamente según el sistema operativo

### Parámetros de UART (solo `lane_follower.py`)
- `--uart-port <puerto>`: Puerto serie (ej: `/dev/ttyUSB0`, `COM5`)
- `--uart-interactive`: Selección interactiva del puerto
- `--uart-baud <velocidad>`: Velocidad de baudios (default: 115200)
- `--arm-system`: Armar el sistema antes de iniciar (requerido para control)
- `--mode <manual|auto>`: Modo del sistema (default: manual)
- `--speed <180-255>`: Velocidad del vehículo (mínimo funcional: 180)

### Parámetros de Visualización
- `--web-stream`: Habilitar streaming web (deshabilita ventanas OpenCV automáticamente)
- `--web-port <puerto>`: Puerto para el servidor web (default: 5000)
- `--no-display`: Deshabilitar ventanas OpenCV (headless mode)

---

## Ejemplos de Uso Comunes

### 1. Prueba de Detección (sin controlar el vehículo)
```bash
# Con ventanas OpenCV
python deteccion_carril.py

# Con web streaming (accesible desde navegador)
python deteccion_carril.py --web-stream
```
Luego abre en el navegador: `http://localhost:5000` o `http://<ip-del-raspberry>:5000`

### 2. Control Automático del Vehículo
```bash
# Selección interactiva de puerto
python lane_follower.py --uart-interactive --arm-system --speed 180

# Con puerto específico
python lane_follower.py --uart-port /dev/ttyUSB0 --arm-system --speed 200
```

### 3. Control Remoto con Web Streaming
```bash
# Control del vehículo + streaming web (sin ventanas OpenCV)
python lane_follower.py --uart-port /dev/ttyUSB0 --arm-system --web-stream --speed 180
```
Luego abre en el navegador: `http://localhost:5000` o `http://<ip-del-raspberry>:5000`

### 4. Modo Headless (sin visualización)
```bash
# Solo control, sin visualización
python lane_follower.py --uart-port /dev/ttyUSB0 --arm-system --no-display --speed 180
```

---

## Comportamiento de Visualización

### Con `--web-stream`:
- ✅ Servidor web habilitado (accesible desde navegador)
- ❌ Ventanas OpenCV deshabilitadas automáticamente
- 📡 Streaming disponible en `http://localhost:<puerto>`

### Sin `--web-stream` y sin `--no-display`:
- ✅ Ventanas OpenCV habilitadas
- ❌ Servidor web deshabilitado
- 🖥️ Visualización local en ventanas

### Con `--no-display`:
- ❌ Ventanas OpenCV deshabilitadas
- ❌ Servidor web deshabilitado
- 🔇 Modo headless (sin visualización)

---

## Acceso al Web Streaming

Cuando usas `--web-stream`, el servidor web se inicia y muestra:

```
🌐 Web streamer started!
   Access at: http://192.168.1.100:5000
   Or from network: http://<raspberry-pi-ip>:5000
   Press Ctrl+C to stop
```

Abre esa URL en cualquier navegador para ver el streaming en tiempo real.

---

## Detener el Programa

- **Con ventanas OpenCV**: Presiona `q` en la ventana de visualización
- **Con web streaming o headless**: Presiona `Ctrl+C` en la terminal

---

## Requisitos

### Dependencias básicas:
```bash
pip install opencv-python numpy pyserial
```

### Para web streaming:
```bash
pip install flask
```

O instalar todo desde `requirements.txt`:
```bash
pip install -r ../requirements.txt
```

---

## Troubleshooting

### Error: "No serial ports found"
- Verifica que el ESP32 esté conectado
- En Linux/WSL, verifica permisos: `sudo usermod -a -G dialout $USER` (luego reinicia sesión)

### Error: "Flask not installed"
- Instala Flask: `pip install flask`

### Las ventanas OpenCV no se abren
- Si usas `--web-stream`, es normal (se deshabilitan automáticamente)
- Verifica que tengas una pantalla conectada si no usas web streaming

### El vehículo no responde
- Verifica que hayas usado `--arm-system`
- Verifica que el puerto UART sea correcto
- Verifica la velocidad mínima (180)

