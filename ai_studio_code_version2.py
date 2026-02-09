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

    def calibrate(self):
        """Setzt die aktuelle Position als Nullpunkt"""
        self.offset_r, self.offset_p, self.offset_y = self.roll, self.pitch, self.yaw
        print(f"Kalibriert auf: R:{self.offset_r:.2f} P:{self.offset_p:.2f} Y:{self.offset_y:.2f}")

shared_data = SensorData()

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
            shared_data.roll, shared_data.pitch, shared_data.yaw = angles
            shared_data.new_data = True
            # Debug Print (auskommentieren wenn es nervt)
            # print(f"Sensor: R:{angles[0]:.1f} P:{angles[1]:.1f} Y:{angles[2]:.1f}")

    async with BleakClient(MAC_ADDRESS) as client:
        print(f"Verbunden mit {MAC_ADDRESS}")
        await client.start_notify(NOTIFY_UUID, callback)
        while True:
            await asyncio.sleep(0.1)

def start_ble_thread():
    asyncio.run(run_bluetooth())

# --- VTK Update Logik ---
def update_view(obj, event):
    if shared_data.new_data:
        # Berechne relative Winkel (aktuell minus Kalibrierung)
        r = shared_data.roll - shared_data.offset_r
        p = shared_data.pitch - shared_data.offset_p
        y = shared_data.yaw - shared_data.offset_y

        # Transform für saubere Rotation
        # Wichtig: Die Reihenfolge der Rotationen (Y-X-Z) muss zur Hardware passen
        transform = vtkTransform()
        transform.PostMultiply() # Transformationen werden nacheinander angewendet
        
        # Achsen-Mapping (Eventuell musst du hier X, Y, Z tauschen!)
        transform.RotateX(p)
        transform.RotateY(-y) # Yaw oft invertiert
        transform.RotateZ(r)

        cube_actor.SetUserTransform(transform)
        axes_actor.SetUserTransform(transform)
        
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
    
    interactor = vtkRenderWindowInteractor()
    interactor.SetRenderWindow(renderWindow)

    renderer.AddActor(cube_actor)
    renderer.AddActor(axes_actor)
    renderer.SetBackground(colors.GetColor3d('SlateGray'))

    renderer.ResetCamera()
    renderer.GetActiveCamera().Zoom(1.5)

    # Bluetooth Thread
    threading.Thread(target=start_ble_thread, daemon=True).start()

    # Interaktive Kalibrierung: Taste 'c' drücken zum Nullen
    def key_press(obj, event):
        key = obj.GetKeySym()
        if key == 'c':
            shared_data.calibrate()

    interactor.AddObserver('KeyPressEvent', key_press)
    interactor.Initialize()
    interactor.AddObserver('TimerEvent', update_view)
    interactor.CreateRepeatingTimer(10) # 100 Hz Update

    print("--- STEUERUNG ---")
    print("Taste 'c' drücken, um den Sensor zu nullen (Kalibrierung)")
    print("Taste 'q' zum Beenden")
    
    renderWindow.Render()
    interactor.Start()