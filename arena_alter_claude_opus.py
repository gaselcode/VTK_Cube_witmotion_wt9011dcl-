import vtkmodules.vtkInteractionStyle
import vtkmodules.vtkRenderingOpenGL2
from vtkmodules.vtkCommonColor import vtkNamedColors
from vtkmodules.vtkFiltersSources import vtkCubeSource, vtkPlaneSource
from vtkmodules.vtkRenderingAnnotation import vtkAxesActor
from vtkmodules.vtkCommonTransforms import vtkTransform
from vtkmodules.vtkRenderingCore import (
    vtkActor, vtkPolyDataMapper, vtkRenderWindow,
    vtkRenderWindowInteractor, vtkRenderer
)
import threading
import asyncio
import time
from bleak import BleakClient
import struct

# --- Konfiguration ---
MAC_ADDRESS = "FC:C8:EB:49:44:06"
NOTIFY_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"

# --- Achsen-Konfiguration (WICHTIG für Sync) ---
# Witmotion liefert oft: X=Vorwärts, Y=Links, Z=Hoch
# VTK nutzt: X=Rechts, Y=Hoch, Z=Aus Bildschirm
# Experimentiere mit den Werten (1 oder -1), bis die Bewegung passt
AXIS_CONFIG = {
    'roll_sign': 1.0,    # Oft 1.0 oder -1.0
    'pitch_sign': 1.0,   # Oft 1.0 oder -1.0
    'yaw_sign': -1.0,    # Oft -1.0 (wegen Uhrzeigersinn vs. Gegenuhrzeigersinn)
    'swap_xy': False,    # Falls X und Y vertauscht sind
}

# Glättungsfaktor (0.0 = starr, 1.0 = roh). 0.2 bis 0.4 ist meist gut.
SMOOTHING_ALPHA = 0.3 

class SensorData:
    def __init__(self):
        self.lock = threading.Lock()
        self.raw_roll = 0.0
        self.raw_pitch = 0.0
        self.raw_yaw = 0.0
        
        # Gefilterte Werte für die Anzeige
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.offset_r = 0.0
        self.offset_p = 0.0
        self.offset_y = 0.0
        self.new_data = False
        self.connected = False
        self.last_update_time = time.time()

    def update(self, r, p, y):
        """Wird vom BLE Thread aufgerufen"""
        with self.lock:
            self.raw_roll = r
            self.raw_pitch = p
            self.raw_yaw = y
            self.new_data = True
            self.last_update_time = time.time()

    def get_smoothed_angles(self):
        """Wird vom VTK Thread aufgerufen. Wendet Filter und Offset an."""
        with self.lock:
            # 1. Low-Pass Filter (Exponential Moving Average)
            # Formel: neu = alt * (1-alpha) + roh * alpha
            self.roll = self.roll * (1 - SMOOTHING_ALPHA) + self.raw_roll * SMOOTHING_ALPHA
            self.pitch = self.pitch * (1 - SMOOTHING_ALPHA) + self.raw_pitch * SMOOTHING_ALPHA
            self.yaw = self.yaw * (1 - SMOOTHING_ALPHA) + self.raw_yaw * SMOOTHING_ALPHA
            
            # 2. Offset abziehen (Kalibrierung)
            r = self.roll - self.offset_r
            p = self.pitch - self.offset_p
            y = self.yaw - self.offset_y
            
            # 3. Achsen-Korrektur zurücksetzen für nächsten Loop
            self.new_data = False 
            return r, p, y

    def calibrate(self):
        """Setzt die aktuelle Position als Nullpunkt"""
        with self.lock:
            self.offset_r = self.raw_roll
            self.offset_p = self.raw_pitch
            self.offset_y = self.raw_yaw
            # Reset filtered values to current raw values to avoid jump
            self.roll = self.raw_roll
            self.pitch = self.raw_pitch
            self.yaw = self.raw_yaw
            
        print(f"--- Kalibriert ---")
        print(f"R: {self.offset_r:.2f}, P: {self.offset_p:.2f}, Y: {self.offset_y:.2f}")

    def is_stale(self):
        """Prüft ob die Daten zu alt sind (Verbindung verloren?)"""
        return (time.time() - self.last_update_time) > 2.0

shared_data = SensorData()

# --- Bluetooth Datenverarbeitung ---
def decode_witmotion(data):
    """
    Witmotion Protokoll für WT9011DCL (BLE 5.0):
    0x55 0x61 -> Euler Winkel
    Byte 2-3: Roll, 4-5: Pitch, 6-7: Yaw
    """
    try:
        # Suche nach Header 0x55 0x61 (Euler Winkel)
        for i in range(len(data) - 7):
            if data[i] == 0x55 and data[i+1] == 0x61:
                # Witmotion Formel: Wert / 32768 * 180
                r = struct.unpack('<h', data[i+2:i+4])[0] / 32768.0 * 180.0
                p = struct.unpack('<h', data[i+4:i+6])[0] / 32768.0 * 180.0
                y = struct.unpack('<h', data[i+6:i+8])[0] / 32768.0 * 180.0
                return r, p, y
    except Exception as e:
        pass # Silent fail to avoid console spam
    return None

async def run_bluetooth():
    def callback(sender, data):
        angles = decode_witmotion(data)
        if angles:
            shared_data.update(*angles)

    max_retries = 3
    retry_count = 0
    
    while retry_count < max_retries:
        try:
            print(f"Verbindungsversuch {retry_count + 1}/{max_retries}...")
            async with BleakClient(MAC_ADDRESS, timeout=10.0) as client:
                shared_data.connected = True
                print(f"✓ Verbunden mit {MAC_ADDRESS}")
                
                # Versuche Intervall zu verkürzen (nicht garantiert vom OS unterstützt)
                if hasattr(client, '_request_connection_interval'):
                    try:
                        await client._request_connection_interval(7.5) # 7.5ms theoretisch
                    except:
                        pass
                
                await client.start_notify(NOTIFY_UUID, callback)
                
                # Hauptschleife - läuft bis Verbindung abbricht
                while client.is_connected:
                    await asyncio.sleep(0.05) # 20Hz Check, Callback ist event-basiert
                    
                    # Prüfe auf veraltete Daten
                    if shared_data.is_stale():
                        print("⚠ Keine Daten empfangen - Verbindung prüfen")
                        break
                        
        except Exception as e:
            print(f"✗ Bluetooth Fehler: {e}")
            retry_count += 1
            if retry_count < max_retries:
                print(f"Warte 2 Sekunden vor erneutem Versuch...")
                await asyncio.sleep(2)
        finally:
            shared_data.connected = False
    
    print("Bluetooth Thread beendet - Maximale Versuche erreicht")

def start_ble_thread():
    # Event Loop für den Thread erstellen
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(run_bluetooth())

# --- VTK Update Logik ---
def update_view(obj, event):
    if shared_data.new_data:
        # Hole gefilterte und kalibrierte Winkel
        r, p, y = shared_data.get_smoothed_angles()

        # Konfiguration anwenden
        if AXIS_CONFIG['swap_xy']:
            r, p = p, r # Tausche Roll und Pitch falls nötig
            
        r *= AXIS_CONFIG['roll_sign']
        p *= AXIS_CONFIG['pitch_sign']
        y *= AXIS_CONFIG['yaw_sign']

        # Transform erstellen
        transform = vtkTransform()
        transform.Identity() # Wichtig: Vorherige Transformationen löschen
        
        # Rotationsreihenfolge ist kritisch!
        # Standard für Flugzeuge/IMU oft: Yaw (Z) -> Pitch (X) -> Roll (Y)
        # Hier testen wir Z -> X -> Y (häufig für Witmotion)
        transform.RotateZ(y)
        transform.RotateX(p)
        transform.RotateY(r)

        cube_actor.SetUserTransform(transform)
        
        # Status Update (Farbe ändert sich bei Verbindung)
        if shared_data.connected:
            cube_actor.GetProperty().SetColor(colors.GetColor3d('Tomato'))
        else:
            cube_actor.GetProperty().SetColor(colors.GetColor3d('Gray'))
        
        # Debug Info im Titel
        renderWindow.SetWindowName(
            f"IMU Visualizer | R:{r:.1f}° P:{p:.1f}° Y:{y:.1f}° | "
            f"{'✓ Verbunden' if shared_data.connected else '✗ Getrennt'}"
        )
        
        obj.GetRenderWindow().Render()

# --- Hauptprogramm ---
if __name__ == '__main__':
    colors = vtkNamedColors()

    # --- Szene aufbauen ---
    renderer = vtkRenderer()
    renderWindow = vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindow.SetSize(1200, 900)
    renderWindow.SetWindowName("IMU Visualizer - Warte auf Verbindung...")
    
    interactor = vtkRenderWindowInteractor()
    interactor.SetRenderWindow(renderWindow)

    # 1. Der Würfel (Gerät)
    cubeSource = vtkCubeSource()
    cubeSource.SetXLength(0.6)
    cubeSource.SetYLength(1.0)
    cubeSource.SetZLength(0.15)
    
    cubeMapper = vtkPolyDataMapper()
    cubeMapper.SetInputConnection(cubeSource.GetOutputPort())

    cube_actor = vtkActor()
    cube_actor.SetMapper(cubeMapper)
    cube_actor.GetProperty().SetColor(colors.GetColor3d('Gray')) # Startet grau (nicht verbunden)
    cube_actor.GetProperty().SetOpacity(0.9)
    cube_actor.GetProperty().EdgeVisibilityOn() # Kanten sichtbar machen

    # 2. Koordinatenachsen (Welt) - bleiben fest
    axes_actor = vtkAxesActor()
    axes_actor.SetTotalLength(2.0, 2.0, 2.0)
    axes_actor.GetXAxisCaptionActor2D().GetTextActor().SetTextScaleModeToNone()
    axes_actor.GetYAxisCaptionActor2D().GetTextActor().SetTextScaleModeToNone()
    axes_actor.GetZAxisCaptionActor2D().GetTextActor().SetTextScaleModeToNone()
    axes_actor.GetXAxisCaptionActor2D().GetCaptionTextProperty().SetFontSize(20)
    axes_actor.GetYAxisCaptionActor2D().GetCaptionTextProperty().SetFontSize(20)
    axes_actor.GetZAxisCaptionActor2D().GetCaptionTextProperty().SetFontSize(20)

    # 3. Boden (Grid) als Referenz
    planeSource = vtkPlaneSource()
    planeSource.SetXResolution(20)
    planeSource.SetYResolution(20)
    planeSource.SetOrigin(-3, 0, -3)
    planeSource.SetPoint1(3, 0, -3)
    planeSource.SetPoint2(-3, 0, 3)
    
    planeMapper = vtkPolyDataMapper()
    planeMapper.SetInputConnection(planeSource.GetOutputPort())
    
    planeActor = vtkActor()
    planeActor.SetMapper(planeMapper)
    planeActor.GetProperty().SetColor(colors.GetColor3d('LightGray'))
    planeActor.GetProperty().SetOpacity(0.3)
    planeActor.GetProperty().SetRepresentationToWireframe()
    planeActor.SetPosition(0, -1.5, 0) # Unter den Würfel

    # --- Szene zusammenfügen ---
    renderer.AddActor(cube_actor)
    renderer.AddActor(axes_actor)
    renderer.AddActor(planeActor)
    renderer.SetBackground(0.1, 0.1, 0.2) # Dunkles Blau

    # Kamera positionieren
    renderer.ResetCamera()
    camera = renderer.GetActiveCamera()
    camera.SetPosition(3, 4, 5)
    camera.SetFocalPoint(0, 0, 0)
    camera.SetViewUp(0, 1, 0)
    camera.Zoom(1.2)

    # --- Threads & Events ---
    ble_thread = threading.Thread(target=start_ble_thread, daemon=True)
    ble_thread.start()

    # Tastatur-Events
    def key_press(obj, event):
        key = obj.GetKeySym()
        if key == 'c' or key == 'C':
            shared_data.calibrate()
        elif key == 'r' or key == 'R':
            # Reset Kamera
            renderer.ResetCamera()
            camera.SetPosition(3, 4, 5)
            camera.SetFocalPoint(0, 0, 0)
            camera.SetViewUp(0, 1, 0)
            renderWindow.Render()
        elif key == 'h' or key == 'H':
            # Hilfe anzeigen
            print("\n=== HILFE ===")
            print("C - Kalibrieren (Nullpunkt setzen)")
            print("R - Kamera zurücksetzen")
            print("Q - Beenden")
            print("Maus - Kamera drehen/zoomen")
            print("=============\n")
        elif key == 'q' or key == 'Q':
            interactor.TerminateApp()

    interactor.AddObserver('KeyPressEvent', key_press)
    interactor.Initialize()
    
    # Timer für Render-Loop (60 FPS Ziel)
    interactor.AddObserver('TimerEvent', update_view)
    interactor.CreateRepeatingTimer(16) # 16ms = ~60 FPS

    # --- Startmeldung ---
    print("\n" + "="*50)
    print(" IMU 3D VISUALIZER ")
    print("="*50)
    print("\nSTATUS:")
    print(f"  MAC-Adresse: {MAC_ADDRESS}")
    print(f"  Glättung: {SMOOTHING_ALPHA}")
    print(f"  Achsen-Config: Roll={AXIS_CONFIG['roll_sign']}, "
          f"Pitch={AXIS_CONFIG['pitch_sign']}, Yaw={AXIS_CONFIG['yaw_sign']}")
    
    print("\nSTEUERUNG:")
    print("  [C] - Kalibrieren (aktuelle Position = Nullpunkt)")
    print("  [R] - Kamera zurücksetzen")
    print("  [H] - Hilfe anzeigen")
    print("  [Q] - Beenden")
    print("  Maus - Kamera drehen (Links), zoomen (Rechts/Scroll)")
    
    print("\nFEHLERBEHEBUNG:")
    print("  Bewegung falsch? -> Ändere AXIS_CONFIG Vorzeichen")
    print("  Zu träge? -> Erhöhe SMOOTHING_ALPHA")
    print("  Zu zittrig? -> Verringere SMOOTHING_ALPHA")
    print("\n" + "="*50 + "\n")
    
    # Starte Visualisierung
    renderWindow.Render()
    interactor.Start()
