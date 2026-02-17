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
from queue import Queue, Empty

# --- Konfiguration ---
MAC_ADDRESS = "FC:C8:EB:49:44:06"
NOTIFY_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"

# --- IMU Data Manager mit Thread-Sicherheit ---
class IMUManager:
    def __init__(self):
        self.data_queue = Queue(maxsize=3)  # Kleine Queue, alte Daten werden verworfen
        self.offset_lock = threading.Lock()
        self.offsets = (0.0, 0.0, 0.0)
        
    def set_offsets(self, r, p, y):
        with self.offset_lock:
            self.offsets = (r, p, y)
            
    def get_offsets(self):
        with self.offset_lock:
            return self.offsets

imu_manager = IMUManager()

# --- Bluetooth Datenverarbeitung ---
def decode_witmotion(data):
    """
    Witmotion Protokoll für WT9011DCL (BLE 5.0):
    0x55 0x61 -> Euler Winkel
    Byte 2-3: Roll, 4-5: Pitch, 6-7: Yaw
    """
    try:
        for i in range(len(data) - 7):
            if data[i] == 0x55 and data[i+1] == 0x61:
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
            # Alte Daten verwerfen wenn Queue voll (verhindert Lag)
            if imu_manager.data_queue.full():
                imu_manager.data_queue.get_nowait()
            imu_manager.data_queue.put_nowait(angles)

    async with BleakClient(MAC_ADDRESS) as client:
        print(f"Verbunden mit {MAC_ADDRESS}")
        await client.start_notify(NOTIFY_UUID, callback)
        while True:
            await asyncio.sleep(0.1)

def start_ble_thread():
    asyncio.run(run_bluetooth())

# --- VTK Update Logik ---
def update_view(obj, event):
    try:
        r, p, y = imu_manager.data_queue.get_nowait()
        offset_r, offset_p, offset_y = imu_manager.get_offsets()
        
        # Kalibrierung anwenden
        r -= offset_r
        p -= offset_p
        y -= offset_y
        
        # Transform mit korrekter Rotationsreihenfolge
        transform = vtkTransform()
        transform.PostMultiply()
        transform.RotateZ(r)   # Roll
        transform.RotateX(p)   # Pitch
        transform.RotateY(-y)  # Yaw (invertiert)
        
        cube_actor.SetUserTransform(transform)
        axes_actor.SetUserTransform(transform)
        
        obj.GetRenderWindow().Render()
        
    except Empty:
        pass  # Keine neuen Daten, nichts tun

# --- Kalibrierung per Tastatur ---
def key_press(obj, event):
    key = obj.GetKeySym()
    if key == 'c':
        try:
            r, p, y = imu_manager.data_queue.get_nowait()
            imu_manager.set_offsets(r, p, y)
            print(f"Kalibriert auf: R={r:.2f} P={p:.2f} Y={y:.2f}")
        except Empty:
            print("Keine IMU-Daten verfügbar für Kalibrierung")
    elif key == 'r':
        imu_manager.set_offsets(0.0, 0.0, 0.0)
        print("Kalibrierung zurückgesetzt")

# --- Hauptprogramm ---
if __name__ == '__main__':
    colors = vtkNamedColors()

    # Cube mit Handy-Form
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
    threading.Thread(target=start_ble_thread, daemon=True).start()

    # Event Handler registrieren
    interactor.AddObserver('KeyPressEvent', key_press)
    interactor.Initialize()
    interactor.AddObserver('TimerEvent', update_view)
    interactor.CreateRepeatingTimer(20)  # 50 Hz Update-Rate

    print("=" * 40)
    print("IMU-VTK Visualisierung gestartet")
    print("=" * 40)
    print("Taste 'c' -> Kalibrierung (Nullpunkt setzen)")
    print("Taste 'r' -> Kalibrierung zurücksetzen")
    print("Taste 'q' -> Beenden")
    print("=" * 40)
    
    renderWindow.Render()
    interactor.Start()