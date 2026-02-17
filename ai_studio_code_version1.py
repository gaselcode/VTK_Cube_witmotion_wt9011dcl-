import vtkmodules.vtkInteractionStyle
import vtkmodules.vtkRenderingOpenGL2
from vtkmodules.vtkCommonColor import vtkNamedColors
from vtkmodules.vtkFiltersSources import vtkCubeSource
from vtkmodules.vtkRenderingAnnotation import vtkAxesActor
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
# Witmotion BLE Characteristic für Daten (Standard für WT901BLE)
NOTIFY_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb" 

class SensorData:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.new_data = False

shared_data = SensorData()

# --- Bluetooth Datenverarbeitung ---
def decode_witmotion(data):
    """Interpretiert das Witmotion Protokoll für Euler-Winkel"""
    # Witmotion sendet Pakete, die mit 0x55 beginnen
    # Winkel-Paket Typ ist 0x61
    if len(data) >= 11 and data[0] == 0x55 and data[1] == 0x61:
        # Formel: (HighByte << 8 | LowByte) / 32768 * 180
        r = struct.unpack('<h', data[2:4])[0] / 32768.0 * 180.0
        p = struct.unpack('<h', data[4:6])[0] / 32768.0 * 180.0
        y = struct.unpack('<h', data[6:8])[0] / 32768.0 * 180.0
        return r, p, y
    return None

async def run_bluetooth():
    def callback(sender, data):
        # Manchmal kommen mehrere Pakete in einem BLE-Frame
        for i in range(len(data)-10):
            if data[i] == 0x55 and data[i+1] == 0x61:
                angles = decode_witmotion(data[i:i+11])
                if angles:
                    shared_data.roll, shared_data.pitch, shared_data.yaw = angles
                    shared_data.new_data = True

    print(f"Versuche Verbindung zu {MAC_ADDRESS}...")
    try:
        async with BleakClient(MAC_ADDRESS) as client:
            print("Verbunden!")
            await client.start_notify(NOTIFY_UUID, callback)
            while True:
                await asyncio.sleep(1)
    except Exception as e:
        print(f"Bluetooth Fehler: {e}")

def start_ble_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(run_bluetooth())

# --- VTK Animation Callback ---
def update_orientation(obj, event):
    if shared_data.new_data:
        # VTK nutzt SetOrientation(Pitch, Yaw, Roll)
        # Die Achsenbelegung muss evtl. je nach Sensor-Montage angepasst werden
        cube_actor.SetOrientation(shared_data.pitch, shared_data.yaw, shared_data.roll)
        axes_actor.SetOrientation(shared_data.pitch, shared_data.yaw, shared_data.roll)
        
        obj.GetRenderWindow().Render()
        shared_data.new_data = False

# --- Hauptprogramm ---
if __name__ == '__main__':
    colors = vtkNamedColors()

    # Cube Setup
    cubeSource = vtkCubeSource()
    cubeSource.SetXLength(0.5)
    cubeSource.SetYLength(0.8)
    cubeSource.SetZLength(0.2)
    cubeSource.Update()

    cubeMapper = vtkPolyDataMapper()
    cubeMapper.SetInputConnection(cubeSource.GetOutputPort())

    cube_actor = vtkActor()
    cube_actor.SetMapper(cubeMapper)
    cube_actor.GetProperty().SetOpacity(0.5)
    cube_actor.GetProperty().SetColor(colors.GetColor3d('Red'))

    # Achsen Setup
    axes_actor = vtkAxesActor()

    renderer = vtkRenderer()
    renderWindow = vtkRenderWindow()
    renderWindow.SetSize(800, 800)
    renderWindow.AddRenderer(renderer)
    renderWindow.SetWindowName('WT901 BLE Sensor Tracker')

    interactor = vtkRenderWindowInteractor()
    interactor.SetRenderWindow(renderWindow)

    renderer.AddActor(cube_actor)
    renderer.AddActor(axes_actor)
    renderer.SetBackground(colors.GetColor3d('DimGray'))

    renderer.ResetCamera()
    renderer.GetActiveCamera().Azimuth(30)
    renderer.GetActiveCamera().Elevation(30)

    # Bluetooth Thread starten
    ble_thread = threading.Thread(target=start_ble_thread, daemon=True)
    ble_thread.start()

    # Timer für die Aktualisierung hinzufügen (ms)
    interactor.Initialize()
    interactor.AddObserver('TimerEvent', update_orientation)
    timer_id = interactor.CreateRepeatingTimer(20) # 50 FPS

    print("Starte Visualisierung...")
    renderWindow.Render()
    interactor.Start()