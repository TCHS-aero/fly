import sys
import asyncio
import re

from fly.core.drone import Drone
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QLabel,QDoubleSpinBox,
    QVBoxLayout, QWidget, QTextEdit, QLineEdit, QTabWidget, QGridLayout,QSizePolicy
)
from PyQt6.QtGui import QFont, QFontDatabase, QIcon
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
        self.setWindowIcon(QIcon("src/fly/assets/tc_aero_logo.png"))
        central = QWidget()
        main_layout = QVBoxLayout()

        
        self.tabs = QTabWidget()

        #implementing font
        self.font_id = QFontDatabase.addApplicationFont("src/fly/assets/BlackOpsOne-Regular.ttf")
        if self.font_id != -1:  # Success check
            font_families = QFontDatabase.applicationFontFamilies(self.font_id)
            custom_font_name = font_families[0]  # Usually index 0
        else:
            print("Font failed to load")
            custom_font_name = "Arial"  # fallback

        general_widget = QWidget()
        general_layout = QVBoxLayout()
        
        self.port_edit = QLineEdit()
        self.port_edit.setPlaceholderText("Port, e.g. udpin://0.0.0.0:14540")

        self.logo = QLabel("TCHS Aero GUI v1")
        self.logo.setStyleSheet(f"font-size:40px; font-family: {custom_font_name};")
        self.status = QLabel("Status: Disconnected")
        self.console = QTextEdit()
        self.console.setStyleSheet("background-color: black")
        self.console.setReadOnly(True)

        self.button_connect = QPushButton("1. Connect to the drone")
        self.button_connect.setCheckable(True)
        self.button_connect.clicked.connect(self.on_connect)
        self.button_connect.setStyleSheet("""
            QPushButton {background-color: gray}
            QPushButton:checked { background-color: green; color: white; }
            QPushButton {border-radius: 4px}
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
            QPushButton {border-radius: 4px}
        """)

        general_layout.addWidget(self.port_edit)
        general_layout.addWidget(self.button_connect)
        general_layout.addWidget(self.button_takeoff)
        general_layout.addWidget(self.button_land)
        general_widget.setLayout(general_layout)
        self.tabs.addTab(general_widget, "General")


        movement_widget = QWidget()
        movement_layout = QVBoxLayout()
        
        self.movement_controls_text = QLabel("Movement Controls")
        
        self.logo = QLabel("TCHS Aero GUI ver.1")
        self.logo.setStyleSheet("background-color: red")
        self.logo.setGeometry(400,100,500,50)


        grid = QGridLayout()
        #buttons for drone movement
        self.button_up = QPushButton("Up")
        self.button_up.clicked.connect(self.move_up)
        self.button_down = QPushButton("Down")
        self.button_down.clicked.connect(self.move_down)
        self.button_left = QPushButton("⬅︎")
        self.button_left.clicked.connect(self.move_left)
        self.button_right = QPushButton("➡︎")
        self.button_right.clicked.connect(self.move_right)
        self.button_forward = QPushButton("⬆︎")
        self.button_forward.clicked.connect(self.move_forward)
        self.button_backward = QPushButton("⬇︎")
        self.button_backward.clicked.connect(self.move_backward)
        self.button_stop_movement = QPushButton("Stop")
        self.button_stop_movement.clicked.connect(self.stoping_movement)
        self.button_stop_movement.setStyleSheet("""
            QPushButton {border-radius: 4px}
            QPushButton:!checked { background-color: red; color: white; }
            QPushButton:pressed { background-color: darkorange; color: white; }

        """)
        self.button_rth = QPushButton("Return to\nHome")
        self.button_rth.clicked.connect(self.return_to_launch)
        self.button_rth.setStyleSheet("""
            QPushButton {border-radius: 4px}
            QPushButton:!checked { background-color: darkgreen; color: white; }
        """)


        #dynamic button sizes acccording to the user's expansion of the window
        policy_connect = self.button_connect.sizePolicy()
        policy_connect.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.button_connect.setSizePolicy(policy_connect)
        self.button_connect.setMinimumHeight(60)
    
        policy_takeoff = self.button_takeoff.sizePolicy()
        policy_takeoff.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.button_takeoff.setSizePolicy(policy_takeoff)
        self.button_takeoff.setMinimumHeight(60)

        policy_land = self.button_land.sizePolicy()
        policy_land.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.button_land.setSizePolicy(policy_land)
        self.button_land.setMinimumHeight(60)



        policy_stop_mvt = self.button_stop_movement.sizePolicy()
        policy_stop_mvt.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.button_stop_movement.setSizePolicy(policy_stop_mvt)
        self.button_stop_movement.setMinimumHeight(60)

        policy_rth = self.button_rth.sizePolicy()
        policy_rth.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.button_rth.setSizePolicy(policy_rth)
        self.button_rth.setMinimumHeight(60)


        policy_up = self.button_up.sizePolicy()
        policy_up.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.button_up.setSizePolicy(policy_up)
        self.button_up.setMinimumHeight(60)

        policy_down = self.button_down.sizePolicy()
        policy_down.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.button_down.setSizePolicy(policy_down)
        self.button_down.setMinimumHeight(60)

        policy_left = self.button_down.sizePolicy()
        policy_left.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.button_left.setSizePolicy(policy_left)
        self.button_left.setMinimumHeight(60)

        policy_right = self.button_down.sizePolicy()
        policy_right.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.button_right.setSizePolicy(policy_right)
        self.button_right.setMinimumHeight(60)

        policy_foward = self.button_down.sizePolicy()
        policy_foward.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.button_forward.setSizePolicy(policy_foward)
        self.button_forward.setMinimumHeight(60)

        policy_backward = self.button_down.sizePolicy()
        policy_backward.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.button_backward.setSizePolicy(policy_backward)
        self.button_backward.setMinimumHeight(60)


        grid.addWidget(self.button_up, 0, 4)
        grid.addWidget(self.button_down, 2, 4)
        grid.addWidget(self.button_forward, 0, 1)
        grid.addWidget(self.button_backward, 2, 1)
        grid.addWidget(self.button_left, 1, 0)
        grid.addWidget(self.button_right, 1, 2)
        grid.addWidget(self.button_stop_movement, 1, 1)
        grid.addWidget(self.button_rth, 1, 4)
        
        self.velocity_text = QLabel("Velocity")
        self.velocity = QDoubleSpinBox()
        self.yaw_text = QLabel("Yaw")
        self.yaw = QDoubleSpinBox()

        movement_layout.addWidget(self.movement_controls_text)
        movement_layout.addLayout(grid)

        movement_layout.addWidget(self.velocity_text)
        movement_layout.addWidget(self.velocity)
        movement_layout.addWidget(self.yaw_text)
        movement_layout.addWidget(self.yaw)
        movement_widget.setLayout(movement_layout)
        self.tabs.addTab(movement_widget, "Movement")

        main_layout.addWidget(self.logo)
        main_layout.addWidget(self.status)
        main_layout.addWidget(self.console)
        main_layout.addWidget(self.tabs)
        central.setLayout(main_layout)
        self.setCentralWidget(central)

    def log(self, msg):
        self.console.append(msg)
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
                    self.log("-- Connected")
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
    



def main():
    app = QApplication(sys.argv)
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)
    
    win = TC_Drone_App()
    win.show()
    
    with loop:
        loop.run_forever()

if __name__ == "__main__":
    main()