import vtkmodules.vtkInteractionStyle
import vtkmodules.vtkRenderingOpenGL2
from vtkmodules.vtkCommonColor import vtkNamedColors
from vtkmodules.vtkFiltersSources import vtkCubeSource
from vtkmodules.vtkRenderingAnnotation import vtkAxesActor
from vtkmodules.vtkCommonTransforms import vtkTransform
from vtkmodules.vtkRenderingCore import (
    vtkActor, vtkPolyDataMapper, vtkRenderWindow,
    vtkRenderWindowInteractor, vtkRenderer
)
import threading
import asyncio
from bleak import BleakClient
import struct
from collections import deque

# --- Konfiguration ---
MAC_ADDRESS = "FC:C8:EB:49:44:06"
NOTIFY_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"

class SensorData:
    def __init__(self, smoothing_window=5):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.offset_r = 0.0
        self.offset_p = 0.0
        self.offset_y = 0.0
        self.new_data = False
        
        # Gleitender Mittelwert für Glättung
        self.smoothing_window = smoothing_window
        self.roll_buffer = deque([0.0] * smoothing_window, maxlen=smoothing_window)
        self.pitch_buffer = deque([0.0] * smoothing_window, maxlen=smoothing_window)
        self.yaw_buffer = deque([0.0] * smoothing_window, maxlen=smoothing_window)

    def add_raw_angles(self, roll, pitch, yaw):
        """Fügt neue Rohdaten hinzu und aktualisiert die gepufferten Werte"""
        self.roll_buffer.append(roll)
        self.pitch_buffer.append(pitch)
        self.yaw_buffer.append(yaw)
        
        # Berechne gleitenden Mittelwert
        self.roll = sum(self.roll_buffer) / len(self.roll_buffer)
        self.pitch = sum(self.pitch_buffer) / len(self.pitch_buffer)
        self.yaw = sum(self.yaw_buffer) / len(self.yaw_buffer)
        self.new_data = True

    def calibrate(self):
        """Setzt die aktuelle Position als Nullpunkt"""
        self.offset_r, self.offset_p, self.offset_y = self.roll, self.pitch, self.yaw
        print(f"Kalibriert auf: R:{self.offset_r:.2f} P:{self.offset_p:.2f} Y:{self.offset_y:.2f}")

shared_data = SensorData(smoothing_window=5)  # 5 Messwerte glätten

# --- Bluetooth Datenverarbeitung ---
def decode_witmotion(data):
    """
    Witmotion Protokoll für WT9011DCL (BLE 5.0):
    0x55 0x61 -> Euler Winkel
    Byte 2-3: Roll, 4-5: Pitch, 6-7: Yaw
    """
    try:
        # Wir suchen den Header 0x55 0x61 im Stream
        for i in range(len(data) - 7):
            if data[i] == 0x55 and data[i+1] == 0x61:
                # Little-endian signed short (<h)
                # Formel: (ShortValue / 32768) * 180
                r = struct.unpack('<h', data[i+2:i+4])[0] / 32768.0 * 180.0
                p = struct.unpack('<h', data[i+4:i+6])[0] / 32768.0 * 180.0
                y = struct.unpack('<h', data[i+6:i+8])[0] / 32768.0 * 180.0
                return r, p, y
    except Exception as e:
        print(f"Decode Fehler: {e}")
    return None

async def run_bluetooth():
    def callback(sender, data):
        angles = decode_witmotion(data)
        if angles:
            shared_data.add_raw_angles(angles[0], angles[1], angles[2])
            
            # Optional: Debug-Ausgabe
            # print(f"Raw: R:{angles[0]:.1f}, P:{angles[1]:.1f}, Y:{angles[2]:.1f} | Smoothed: R:{shared_data.roll:.1f}, P:{shared_data.pitch:.1f}, Y:{shared_data.yaw:.1f}")

    async with BleakClient(MAC_ADDRESS) as client:
        print(f"Verbunden mit {MAC_ADDRESS}")
        await client.start_notify(NOTIFY_UUID, callback)
        while True:
            await asyncio.sleep(0.01)  # 10ms Intervall für schnellere Updates

def start_ble_thread():
    try:
        asyncio.run(run_bluetooth())
    except Exception as e:
        print(f"Bluetooth-Fehler: {e}")

# --- VTK Update Logik ---
def update_view(obj, event):
    if shared_data.new_data:
        # Berechne relative Winkel (aktuell minus Kalibrierung)
        r = shared_data.roll - shared_data.offset_r
        p = shared_data.pitch - shared_data.offset_p
        y = shared_data.yaw - shared_data.offset_y

        # Optional: Winkel begrenzen falls sie zu extrem sind
        max_angle = 180.0  # Grad
        r = max(min(r, max_angle), -max_angle)
        p = max(min(p, max_angle), -max_angle)
        y = max(min(y, max_angle), -max_angle)

        # Transform für saubere Rotation
        transform = vtkTransform()
        transform.Identity()  # Start mit neutraler Transformation
        
        # ACHTUNG: Du könntest diese Reihenfolge anpassen müssen je nach Sensorausrichtung!
        transform.RotateZ(r)  # Teste auch: RotateY(-y), RotateX(p)
        transform.RotateY(-y)
        transform.RotateX(p)

        cube_actor.SetUserTransform(transform)
        axes_actor.SetUserTransform(transform)
        
        obj.GetRenderWindow().Render()
        shared_data.new_data = False

# --- Hauptprogramm ---
if __name__ == '__main__':
    colors = vtkNamedColors()

    # Cube mit realistischen Maßen (Smartphone-ähnlich)
    cubeSource = vtkCubeSource()
    cubeSource.SetXLength(0.6)
    cubeSource.SetYLength(1.0)
    cubeSource.SetZLength(0.15)
    
    cubeMapper = vtkPolyDataMapper()
    cubeMapper.SetInputConnection(cubeSource.GetOutputPort())

    cube_actor = vtkActor()
    cube_actor.SetMapper(cubeMapper)
    cube_actor.GetProperty().SetOpacity(0.7)
    cube_actor.GetProperty().SetColor(colors.GetColor3d('Tomato'))

    axes_actor = vtkAxesActor()
    axes_actor.SetTotalLength(1.0, 1.0, 1.0)

    renderer = vtkRenderer()
    renderWindow = vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindow.SetSize(1000, 1000)
    
    interactor = vtkRenderWindowInteractor()
    interactor.SetRenderWindow(renderWindow)

    renderer.AddActor(cube_actor)
    renderer.AddActor(axes_actor)
    renderer.SetBackground(colors.GetColor3d('SlateGray'))

    renderer.ResetCamera()
    renderer.GetActiveCamera().Zoom(1.5)

    # Bluetooth Thread starten
    ble_thread = threading.Thread(target=start_ble_thread, daemon=True)
    ble_thread.start()

    # Interaktive Kalibrierung
    def key_press(obj, event):
        key = obj.GetKeySym()
        if key.lower() == 'c':  # Funktioniert jetzt auch mit Shift+C
            shared_data.calibrate()
        elif key.lower() == 'r':  # Zurücksetzen der Kameraansicht
            renderer.ResetCamera()
            renderer.GetActiveCamera().Zoom(1.5)

    interactor.AddObserver('KeyPressEvent', key_press)
    interactor.Initialize()
    interactor.AddObserver('TimerEvent', update_view)
    interactor.CreateRepeatingTimer(10)  # ~100Hz Update

    print("--- STEUERUNG ---")
    print("Taste 'C' drücken, um den Sensor zu kalibrieren (Nullpunkt setzen)")
    print("Taste 'R' drücken, um die Kameraansicht zurückzusetzen")
    print("Taste 'Q' zum Beenden")
    
    renderWindow.Render()
    interactor.Start()