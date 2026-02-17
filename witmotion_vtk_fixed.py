import vtkmodules.vtkInteractionStyle
import vtkmodules.vtkRenderingOpenGL2
from vtkmodules.vtkCommonColor import vtkNamedColors
from vtkmodules.vtkFiltersSources import vtkCubeSource
from vtkmodules.vtkRenderingAnnotation import vtkAxesActor
from vtkmodules.vtkCommonMath import vtkMatrix4x4
from vtkmodules.vtkRenderingCore import (
    vtkActor, vtkPolyDataMapper, vtkRenderWindow,
    vtkRenderWindowInteractor, vtkRenderer
)
import threading
import asyncio
from bleak import BleakClient
import struct
import math

# --- Konfiguration ---
MAC_ADDRESS = "FC:C8:EB:49:44:06"
NOTIFY_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"

class SensorData:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.offset_r = 0.0
        self.offset_p = 0.0
        self.offset_y = 0.0
        self.new_data = False
        self.lock = threading.Lock()

    def calibrate(self):
        """Setzt die aktuelle Position als Nullpunkt"""
        with self.lock:
            self.offset_r, self.offset_p, self.offset_y = self.roll, self.pitch, self.yaw
        print(f"Kalibriert auf: R:{self.offset_r:.2f} P:{self.offset_p:.2f} Y:{self.offset_y:.2f}")

    def get_calibrated_angles(self):
        """Gibt kalibrierte Winkel zurück (thread-safe)"""
        with self.lock:
            r = self.roll - self.offset_r
            p = self.pitch - self.offset_p
            y = self.yaw - self.offset_y
        return r, p, y

shared_data = SensorData()

# --- Bluetooth Datenverarbeitung ---
def decode_witmotion(data):
    """
    Witmotion Protokoll für WT9011DCL (BLE 5.0):
    0x55 0x61 -> Euler Winkel
    Byte 2-3: Roll, 4-5: Pitch, 6-7: Yaw
    
    WICHTIG: Witmotion verwendet typischerweise:
    - Roll: Rotation um X-Achse
    - Pitch: Rotation um Y-Achse  
    - Yaw: Rotation um Z-Achse
    Formel: (ShortValue / 32768) * 180 ergibt Winkel in Grad
    """
    try:
        for i in range(len(data) - 7):
            if data[i] == 0x55 and data[i+1] == 0x61:
                # Little-endian signed short (<h)
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
            with shared_data.lock:
                shared_data.roll, shared_data.pitch, shared_data.yaw = angles
                shared_data.new_data = True

    async with BleakClient(MAC_ADDRESS) as client:
        print(f"Verbunden mit {MAC_ADDRESS}")
        await client.start_notify(NOTIFY_UUID, callback)
        while True:
            await asyncio.sleep(0.01)  # Schnelleres Polling

def start_ble_thread():
    asyncio.run(run_bluetooth())


def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Erstellt eine Rotationsmatrix aus Euler-Winkeln (ZYX Konvention - Tait-Bryan).
    
    Die meisten IMUs (inkl. Witmotion) verwenden die ZYX-Reihenfolge:
    1. Yaw (Z-Achse)
    2. Pitch (Y-Achse)
    3. Roll (X-Achse)
    
    Dies entspricht der "Z-Y-X" extrinsischen Rotation oder
    "X-Y-Z" intrinsischen Rotation.
    
    @param roll:  Rotation um X-Achse in Grad
    @param pitch: Rotation um Y-Achse in Grad
    @param yaw:   Rotation um Z-Achse in Grad
    @return: vtkMatrix4x4 Rotationsmatrix
    """
    # Umrechnung in Radianten
    rx = math.radians(roll)
    ry = math.radians(pitch)
    ryaw = math.radians(yaw)
    
    # Sinus und Cosinus vorberechnen
    cr, sr = math.cos(rx), math.sin(rx)
    cp, sp = math.cos(ry), math.sin(ry)
    cy, sy = math.cos(ryaw), math.sin(ryaw)
    
    # Rotationsmatrix nach ZYX-Euler (Yaw-Pitch-Roll)
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    #
    # | cy*cp           cy*sp*sr - sy*cr    cy*sp*cr + sy*sr |
    # | sy*cp           sy*sp*sr + cy*cr    sy*sp*cr - cy*sr |
    # | -sp             cp*sr               cp*cr            |
    
    matrix = vtkMatrix4x4()
    
    # Erste Zeile
    matrix.SetElement(0, 0, cy * cp)
    matrix.SetElement(0, 1, cy * sp * sr - sy * cr)
    matrix.SetElement(0, 2, cy * sp * cr + sy * sr)
    matrix.SetElement(0, 3, 0)
    
    # Zweite Zeile
    matrix.SetElement(1, 0, sy * cp)
    matrix.SetElement(1, 1, sy * sp * sr + cy * cr)
    matrix.SetElement(1, 2, sy * sp * cr - cy * sr)
    matrix.SetElement(1, 3, 0)
    
    # Dritte Zeile
    matrix.SetElement(2, 0, -sp)
    matrix.SetElement(2, 1, cp * sr)
    matrix.SetElement(2, 2, cp * cr)
    matrix.SetElement(2, 3, 0)
    
    # Vierte Zeile
    matrix.SetElement(3, 0, 0)
    matrix.SetElement(3, 1, 0)
    matrix.SetElement(3, 2, 0)
    matrix.SetElement(3, 3, 1)
    
    return matrix


# --- Alternative: Quaternion-basierte Rotation (falls verfügbar) ---
def euler_to_quaternion(roll, pitch, yaw):
    """
    Konvertiert Euler-Winkel zu Quaternion.
    Vermeidet Gimbal-Lock Probleme.
    """
    rx = math.radians(roll) / 2.0
    ry = math.radians(pitch) / 2.0
    rz = math.radians(yaw) / 2.0
    
    cr, sr = math.cos(rx), math.sin(rx)
    cp, sp = math.cos(ry), math.sin(ry)
    cy, sy = math.cos(rz), math.sin(rz)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return w, x, y, z


def quaternion_to_matrix(w, x, y, z):
    """
    Konvertiert Quaternion zu Rotationsmatrix.
    """
    matrix = vtkMatrix4x4()
    
    # Normalisieren
    n = math.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/n, x/n, y/n, z/n
    
    # Matrix aus Quaternion
    matrix.SetElement(0, 0, 1 - 2*(y*y + z*z))
    matrix.SetElement(0, 1, 2*(x*y - z*w))
    matrix.SetElement(0, 2, 2*(x*z + y*w))
    
    matrix.SetElement(1, 0, 2*(x*y + z*w))
    matrix.SetElement(1, 1, 1 - 2*(x*x + z*z))
    matrix.SetElement(1, 2, 2*(y*z - x*w))
    
    matrix.SetElement(2, 0, 2*(x*z - y*w))
    matrix.SetElement(2, 1, 2*(y*z + x*w))
    matrix.SetElement(2, 2, 1 - 2*(x*x + y*y))
    
    return matrix


# --- VTK Update Logik ---
def update_view(obj, event):
    """Aktualisiert die Visualisierung mit Sensordaten."""
    if shared_data.new_data:
        r, p, y = shared_data.get_calibrated_angles()
        
        # Debug-Ausgabe
        print(f"\rAngles - Roll: {r:7.2f}° | Pitch: {p:7.2f}° | Yaw: {y:7.2f}°", end="", flush=True)
        
        # Option 1: Euler-Winkel zu Matrix (ZYX Konvention)
        rotation_matrix = euler_to_rotation_matrix(r, p, y)
        
        # Option 2: Falls Probleme auftreten, versuche Quaternion (auskommentieren zum Testen)
        # w, qx, qy, qz = euler_to_quaternion(r, p, y)
        # rotation_matrix = quaternion_to_matrix(w, qx, qy, qz)
        
        # Matrix auf Actors anwenden
        cube_actor.SetUserMatrix(rotation_matrix)
        axes_actor.SetUserMatrix(rotation_matrix)
        
        obj.GetRenderWindow().Render()
        shared_data.new_data = False


# --- Achsen-Korrekturen (falls nötig) ---
def update_view_with_axis_correction(obj, event):
    """
    Alternative Update-Funktion mit Achsen-Korrektur.
    Verwende diese, falls die Standard-Methode nicht funktioniert.
    
    Typische Korrekturen für Witmotion:
    - Roll:  Korrekt
    - Pitch: Möglicherweise invertiert
    - Yaw:   Möglicherweise invertiert oder 90° Offset
    """
    if shared_data.new_data:
        r, p, yaw = shared_data.get_calibrated_angles()
        
        # Achsen-Korrektur anwenden
        # Experimentiere mit diesen Werten, falls die Rotation nicht stimmt:
        corrected_roll = r           # oder -r
        corrected_pitch = -p         # oft invertiert
        corrected_yaw = -yaw         # oft invertiert
        
        # Alternativ: 90° Offset für Yaw (falls Sensor anders ausgerichtet)
        # corrected_yaw = -(yaw + 90)
        
        rotation_matrix = euler_to_rotation_matrix(corrected_roll, corrected_pitch, corrected_yaw)
        
        cube_actor.SetUserMatrix(rotation_matrix)
        axes_actor.SetUserMatrix(rotation_matrix)
        
        obj.GetRenderWindow().Render()
        shared_data.new_data = False


# --- Hauptprogramm ---
if __name__ == '__main__':
    colors = vtkNamedColors()

    # Cube mit realistischeren Maßen (Handy-Form)
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
    renderWindow.SetWindowName("Witmotion IMU Visualizer")
    
    interactor = vtkRenderWindowInteractor()
    interactor.SetRenderWindow(renderWindow)

    renderer.AddActor(cube_actor)
    renderer.AddActor(axes_actor)
    renderer.SetBackground(colors.GetColor3d('SlateGray'))

    renderer.ResetCamera()
    renderer.GetActiveCamera().Zoom(1.5)

    # Bluetooth Thread starten
    print("Starte Bluetooth-Verbindung...")
    ble_thread = threading.Thread(target=start_ble_thread, daemon=True)
    ble_thread.start()

    # Tastatursteuerung
    def key_press(obj, event):
        key = obj.GetKeySym()
        if key == 'c':
            shared_data.calibrate()
        elif key == 'q':
            obj.TerminateApp()

    interactor.AddObserver('KeyPressEvent', key_press)
    interactor.Initialize()
    
    # Timer für Update (10ms = 100Hz)
    interactor.AddObserver('TimerEvent', update_view)
    timer_id = interactor.CreateRepeatingTimer(10)

    print("\n" + "="*50)
    print("STEUEUNG:")
    print("  'c' - Kalibrierung (Nullpunkt setzen)")
    print("  'q' - Beenden")
    print("="*50 + "\n")
    print("Warte auf Sensordaten...")
    print("(Bewege den Sensor um Daten zu sehen)\n")
    
    renderWindow.Render()
    interactor.Start()
