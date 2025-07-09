import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time

# Serial port configuration
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200

# Window size of time
MAX_POINTS = 100

# Initialize serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Conectado al puerto {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"Error al conectar al puerto serie: {e}")
    exit()

# Buffers
times = deque(maxlen=MAX_POINTS)
velocities = deque(maxlen=MAX_POINTS)

# Variables para tiempo relativo
start_time = None

def update(frame):
    global start_time
    
    while ser.in_waiting:
        line = ser.readline().decode().strip()
        print(f"Recibido: {line}")  # Debug line
        
        try:
            # Parse the data (asumiendo formato: tiempo,velocidad)
            t, v = map(float, line.split(','))  # Cambiado a float para mayor precisión
            
            # Si es el primer punto, establecer tiempo de inicio
            if start_time is None:
                start_time = t
            
            # Usar tiempo relativo
            relative_time = t - start_time
            
            times.append(relative_time)
            velocities.append(v)
            
        except ValueError as e:
            print(f"Error parsing line: {line} - {e}")
            pass
    
    # Clear and plot
    ax.clear()
    
    if times and velocities:
        ax.plot(times, velocities, marker='o', linewidth=2, markersize=4)
        
        # Configurar ejes
        ax.set_xlabel("Tiempo (ms)")
        ax.set_ylabel("Velocidad (RPM)")
        ax.set_title("Respuesta del motor en tiempo real")
        ax.grid(True, alpha=0.3)
        
        # Ajustar límites del eje Y para mejor visualización
        if len(velocities) > 0:
            y_min = min(velocities) * 0.95
            y_max = max(velocities) * 1.05
            ax.set_ylim(y_min, y_max)
        
        # Mostrar estadísticas en tiempo real
        if len(velocities) > 0:
            avg_vel = sum(velocities) / len(velocities)
            current_vel = velocities[-1]
            ax.text(0.02, 0.98, f'Actual: {current_vel:.1f} RPM\nPromedio: {avg_vel:.1f} RPM', 
                   transform=ax.transAxes, verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

# Configure the plot
fig, ax = plt.subplots(figsize=(12, 8))
plt.subplots_adjust(bottom=0.15)

# Añadir manejador de cierre
def on_close(event):
    ser.close()
    print("Conexión serie cerrada")

fig.canvas.mpl_connect('close_event', on_close)

try:
    ani = animation.FuncAnimation(fig, update, interval=100, blit=False)
    plt.show()
except KeyboardInterrupt:
    print("Programa interrumpido por el usuario")
finally:
    ser.close()
    print("Conexión serie cerrada")