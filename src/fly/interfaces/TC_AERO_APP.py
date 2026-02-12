import sys
import asyncio
import re

from fly.core.drone import Drone
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QLabel,QDoubleSpinBox,
    QVBoxLayout, QWidget, QTextEdit, QLineEdit, QTabWidget, QGridLayout
)
from qasync import asyncSlot, QEventLoop

def is_valid_port(port: str) -> bool:
    udp_pattern = r"^udp(?:in|out)?://([0-9]{1,3}\.){3}[0-9]{1,3}:[0-9]{1,5}$"
    tcp_pattern = r"^tcp(?:in|out)?://([0-9]{1,3}\.){3}[0-9]{1,3}:[0-9]{1,5}$"
    serial_pattern = r"^serial://(/dev/[a-zA-Z0-9_-]+|COM[0-9]+)(:[0-9]+)?$"

    if re.match(udp_pattern, port):
        return True
    if re.match(tcp_pattern, port):
        return True
    if re.match(serial_pattern, port):
        return True
    return False


class TC_Drone_App(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Controls")
        self.drone = None
    
        central = QWidget()
        main_layout = QVBoxLayout()

        # === CREATE TABS ===
        self.tabs = QTabWidget()
        
        # TAB 1: GENERAL CONTROLS + CONSOLE
        general_widget = QWidget()
        general_layout = QVBoxLayout()
        
        self.port_edit = QLineEdit()
        self.port_edit.setPlaceholderText("Port, e.g. udpin://0.0.0.0:14540")
        
        self.button_connect = QPushButton("1. Connect to the drone")
        self.button_connect.setCheckable(True)
        self.button_connect.clicked.connect(self.on_connect)
        self.button_connect.setStyleSheet("""
            QPushButton:checked { background-color: green; color: white; }
        """)

        self.button_takeoff = QPushButton("2. Takeoff (10m)")
        self.button_takeoff.setEnabled(False)
        self.button_takeoff.clicked.connect(self.on_takeoff)

        self.button_land = QPushButton("3. Land The Drone")
        self.button_land.setCheckable(True)
        self.button_land.setEnabled(False)
        self.button_land.clicked.connect(self.on_land)
        self.button_land.setStyleSheet("""
            QPushButton:disabled { background-color: lightcoral; color: white; }
            QPushButton:!checked { background-color: red; color: white; }
            QPushButton:pressed { background-color: darkorange; color: white; }
        """)

        self.status = QLabel("Status: Disconnected")
        
        # CONSOLE 1 - General tab
        self.general_console = QTextEdit()
        self.general_console.setReadOnly(True)
        
        general_layout.addWidget(self.port_edit)
        general_layout.addWidget(self.button_connect)
        general_layout.addWidget(self.button_takeoff)
        general_layout.addWidget(self.button_land)
        general_layout.addWidget(self.status)
        general_layout.addWidget(self.general_console)
        general_widget.setLayout(general_layout)
        self.tabs.addTab(general_widget, "General")

        # TAB 2: MOVEMENT CONTROLS + CONSOLE
        movement_widget = QWidget()
        movement_layout = QVBoxLayout()
        
        self.movement_controls_text = QLabel("Movement Controls")
        
        # FIXED GRID
        grid = QGridLayout()  # ← () creates instance!
        
        self.button_up = QPushButton("Move Up")
        self.button_up.clicked.connect(self.move_up)
        self.button_down = QPushButton("Move Down")
        self.button_down.clicked.connect(self.move_down)
        self.button_left = QPushButton("Move Left")
        self.button_left.clicked.connect(self.move_left)
        self.button_right = QPushButton("Move Right")
        self.button_right.clicked.connect(self.move_right)
        self.button_forward = QPushButton("Move Forward")
        self.button_forward.clicked.connect(self.move_forward)
        self.button_backward = QPushButton("Move Backward")
        self.button_backward.clicked.connect(self.move_backward)
        self.button_stop_movement = QPushButton("Stop Movement")
        self.button_stop_movement.clicked.connect(self.stoping_movement)
        self.button_rth = QPushButton("Return to Home")
        self.button_rth.clicked.connect(self.return_to_launch)

        # Your exact grid layout
        grid.addWidget(self.button_up, 0, 0)
        grid.addWidget(self.button_down, 0, 2)
        grid.addWidget(self.button_forward, 0, 1)
        grid.addWidget(self.button_backward, 2, 1)
        grid.addWidget(self.button_left, 1, 0)
        grid.addWidget(self.button_right, 1, 2)
        grid.addWidget(self.button_stop_movement, 1, 1)
        grid.addWidget(self.button_rth, 2, 2)
        
        self.velocity_text = QLabel("Velocity")
        self.velocity = QDoubleSpinBox()
        self.yaw_text = QLabel("Yaw")
        self.yaw = QDoubleSpinBox()

        # CONSOLE 2 - Movement tab  
        self.movement_console = QTextEdit()
        self.movement_console.setReadOnly(True)

        # Movement tab layout
        movement_layout.addWidget(self.movement_controls_text)
        movement_layout.addLayout(grid)
        movement_layout.addWidget(self.velocity_text)
        movement_layout.addWidget(self.velocity)
        movement_layout.addWidget(self.yaw_text)
        movement_layout.addWidget(self.yaw)
        movement_layout.addWidget(self.movement_console)
        movement_widget.setLayout(movement_layout)
        self.tabs.addTab(movement_widget, "Movement")

        main_layout.addWidget(self.tabs)
        central.setLayout(main_layout)
        self.setCentralWidget(central)

    def log(self, msg):
        # Write to BOTH consoles
        self.general_console.append(msg)
        self.movement_console.append(msg)
        print(msg)

    @asyncSlot()
    async def on_connect(self):
        if self.button_connect.isChecked():
            port = self.port_edit.text().strip()
            if not port:
                port = "udpin://0.0.0.0:14540"
                self.log("-- Port not specified, defaulting to udpin://0.0.0.0:14540")

            # Validate port string
            if not is_valid_port(port):
                self.log("-- Invalid port format. Please specify a valid UDP, TCP, or Serial port.")
                self.status.setText("Status: Invalid port")
                self.button_connect.setChecked(False)
                return
            
            self.drone = Drone(port=port)

            self.log("Connecting...")
            try:
                connected = await self.drone.connect()
                if connected:
                    self.status.setText("Status: Connected")
                    self.log("Connected...")
                    self.button_takeoff.setEnabled(True) # Enable Takeoff
                    
                    # Optional: Get position confirmation
                    lat, lon, alt = await self.drone.current_position()
                    self.log(f"Pos: {lat:.5f}, {lon:.5f}, {alt:.1f}m")
                else:
                    self.status.setText("Status: Connection Failed")
                    self.log("-- Connection test failed; consider trying a different port.")
                    self.button_connect.setChecked(False)
            except Exception as e:
                self.log(f"Error: {e}")
                self.button_connect.setEnabled(False)
        else:
            self.status.setText("Status: Disconnected")
            self.button_takeoff.setEnabled(False)

    @asyncSlot()
    async def on_takeoff(self):
        if not self.drone:
            self.log("No drone instance; connect first.")
            return
        self.log("Starting Takeoff Sequence...")
        self.button_takeoff.setEnabled(False)
        try:
            await self.drone.takeoff(10.0) 
            self.log("Takeoff...")
            self.status.setText("Status: Taken off")
            self.button_land.setEnabled(True)
        except Exception as e:
            self.log(f"Takeoff Error: {e}")
            self.button_takeoff.setEnabled(True)

    @asyncSlot()
    async def on_land(self):
        self.log("Starting Landing Sequence...")
        try:
            await self.drone.land() 
            self.log("Landing...")
            self.status.setText("Status: Landing")
            self.button_land.setChecked(False)  # Reset toggle
            self.button_land.setEnabled(False)  # Disable again
            self.button_takeoff.setEnabled(True)

        except Exception as e:
            self.log(f"Landing Error: {e}")
            self.button_takeoff.setEnabled(True)

    @asyncSlot()
    async def move_up(self):
        self.log("Starting Moving Up Sequence...")
        self.velocity_flt = self.velocity.value()
        self.yaw_flt = self.yaw.value()
        print(self.velocity_flt, self.yaw_flt)
        try:
            await self.drone.move_up_offset(velocity=self.velocity_flt, yaw=self.yaw_flt)
            self.log("Moving Up...")
            self.status.setText("Status: Moving Up")

        except Exception as e:
            self.log(f"Moving Up Error: {e}")

    @asyncSlot()
    async def move_down(self):
        self.log("Starting Moving Down Sequence...")
        self.velocity_flt = self.velocity.value()
        self.yaw_flt = self.yaw.value()
        print(self.velocity_flt, self.yaw_flt)
        try:
            await self.drone.move_down_offset(velocity=self.velocity_flt, yaw=self.yaw_flt)
            self.log("Moving Down...")
            self.status.setText("Status: Moving Down")

        except Exception as e:
            self.log(f"Moving Down Error: {e}")

    @asyncSlot()
    async def move_left(self):
        self.log("Starting Moving Left Sequence...")
        self.velocity_flt = self.velocity.value()
        self.yaw_flt = self.yaw.value()
        print(self.velocity_flt, self.yaw_flt)
        try:
            await self.drone.move_left_offset(velocity=self.velocity_flt, yaw=self.yaw_flt)
            self.log("Moving Left...")
            self.status.setText("Status: Moving Left")

        except Exception as e:
            self.log(f"Moving Left Error: {e}")

    @asyncSlot()
    async def move_right(self):
        self.log("Starting Moving Right Sequence...")
        self.velocity_flt = self.velocity.value()
        self.yaw_flt = self.yaw.value()
        print(self.velocity_flt, self.yaw_flt)
        try:
            await self.drone.move_right_offset(velocity=self.velocity_flt, yaw=self.yaw_flt)
            self.log("Moving Right...")
            self.status.setText("Status: Moving Right")

        except Exception as e:
            self.log(f"Moving Right Error: {e}")

    @asyncSlot()
    async def move_forward(self):
        self.log("Starting Moving Forward Sequence...")
        self.velocity_flt = self.velocity.value()
        self.yaw_flt = self.yaw.value()
        print(self.velocity_flt, self.yaw_flt)
        try:
            await self.drone.move_forward_offset(velocity=self.velocity_flt, yaw=self.yaw_flt)
            self.log("Moving Forward...")
            self.status.setText("Status: Moving Forward")

        except Exception as e:
            self.log(f"Moving Forward Error: {e}")

    @asyncSlot()
    async def move_backward(self):
        self.log("Starting Moving Backward Sequence...")
        self.velocity_flt = self.velocity.value()
        self.yaw_flt = self.yaw.value()
        print(self.velocity_flt, self.yaw_flt)
        try:
            await self.drone.move_backward_offset(velocity=self.velocity_flt, yaw=self.yaw_flt)
            self.log("Moving Backward...")
            self.status.setText("Status: Moving Backward")

        except Exception as e:
            self.log(f"Moving Backward Error: {e}")

    @asyncSlot()
    async def stoping_movement(self):
        self.log("Stoping Movement...")
        try:
            await self.drone.stop_movement()
            self.log("Stoping Movement...")
            self.status.setText("Status: Stoping Movement")

        except Exception as e:
            self.log(f"Stoping Movement Error: {e}")

    @asyncSlot()
    async def return_to_launch(self):
        self.log("Starting to Return to Launch...")
        try:
            await self.drone.return_to_home()
            self.log("Returning to Launch...")
            self.status.setText("Status: Return to Launch")

        except Exception as e:
            self.log(f"Return to Launch Error: {e}")
    




if __name__ == "__main__":
    app = QApplication(sys.argv)
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)
    
    win = TC_Drone_App()
    win.show()
    
    with loop:
        loop.run_forever()