import sys
import asyncio
from PyQt6.QtWidgets import (QApplication, QMainWindow, QPushButton, QLabel, 
                            QVBoxLayout, QWidget, QTextEdit)
from qasync import QEventLoop, asyncSlot
from src.fly.core.drone import Drone

class DroneApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Control")
        self.setGeometry(100, 100, 400, 400)
        
        self.drone = Drone(port="udp://:14540")
        
        central = QWidget()
        layout = QVBoxLayout()
        
        # Connect Button
        self.btn_connect = QPushButton("1. Connect Drone")
        self.btn_connect.setCheckable(True)
        self.btn_connect.clicked.connect(self.on_connect)
        
        # Takeoff Button
        self.btn_takeoff = QPushButton("2. Takeoff (10m)")
        self.btn_takeoff.setEnabled(False) # Disabled until connected
        self.btn_takeoff.clicked.connect(self.on_takeoff)
        
        self.status = QLabel("Status: Disconnected")
        self.console = QTextEdit()
        
        layout.addWidget(self.btn_connect)
        layout.addWidget(self.btn_takeoff)
        layout.addWidget(self.status)
        layout.addWidget(self.console)
        central.setLayout(layout)
        self.setCentralWidget(central)

    def log(self, msg):
        self.console.append(msg)
        print(msg)

    @asyncSlot()
    async def on_connect(self):
        if self.btn_connect.isChecked():
            self.log("Connecting...")
            try:
                connected = await self.drone.connect()
                if connected:
                    self.status.setText("Status: Connected")
                    self.log("Connected...")
                    self.btn_takeoff.setEnabled(True) # Enable Takeoff
                    
                    # Optional: Get position confirmation
                    lat, lon, alt = await self.drone.current_position()
                    self.log(f"Pos: {lat:.5f}, {lon:.5f}, {alt:.1f}m")
                else:
                    self.status.setText("Status: Connection Failed")
                    self.btn_connect.setChecked(False)
            except Exception as e:
                self.log(f"Error: {e}")
                self.btn_connect.setChecked(False)
        else:
            self.status.setText("Status: Disconnected")
            self.btn_takeoff.setEnabled(False)

    @asyncSlot()
    async def on_takeoff(self):
        self.log("Starting Takeoff Sequence...")
        self.btn_takeoff.setEnabled(False) # Prevent double click
        try:
            # Call your drone class takeoff method
            # Altitude = 10 meters
            await self.drone.takeoff(10) 
            self.log("Takeoff...")
            self.status.setText("Status: Takenoff")
        except Exception as e:
            self.log(f"Takeoff Error: {e}")
            self.btn_takeoff.setEnabled(True) # Re-enable if failed

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # Correct loop setup for macOS/qasync
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)
    
    win = DroneApp()
    win.show()
    
    with loop:
        loop.run_forever()
