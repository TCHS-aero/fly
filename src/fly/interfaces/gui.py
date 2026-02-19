import sys
import asyncio
import re

from fly.core.drone import Drone
from fly.core.dataManager import (
    write_to_json,
    pull_from_json,
    drone_instance_json,
)

from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QPushButton,
    QLabel,
    QDoubleSpinBox,
    QVBoxLayout,
    QWidget,
    QTextEdit,
    QLineEdit,
    QTabWidget,
    QGridLayout,
    QSizePolicy,
)
from PyQt6.QtGui import QIcon
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

    print("-- Testing for a stable connection")
    drone = Drone(port)
    if not drone.connect():
        print("-- Connection test failed; consider trying a different port.")
        return 1

    write_to_json(
        {
            drone_instance_json: {
                "port": port,
            }
        }
    )


class HistoryLineEdit(QComboBox):
    def __init__(self, max_history=5, parent=None):
        super().__init__(parent)
        self.max_history = max_history
        self.setEditable(True)
        self.lineEdit().setPlaceholderText("Port, e.g. udpin://0.0.0.0:14540")
        self.lineEdit().returnPressed.connect(self.save_history)
        self.load_history()

    def load_history(self):
        data = pull_from_json()
        ports = data.get(drone_instance_json, {}).get("port-history", [])
        self.addItems(ports[-self.max_history:])

    def save_history(self):
        history = [self.itemText(i) for i in range(self.count())]
        data = pull_from_json() or {}
        drone_data = data.setdefault(drone_instance_json, {})
        drone_data["port-history"] = history
        drone_data["port"] = history[0] if history else None
        write_to_json({drone_instance_json: drone_data})

    def text(self):
        return self.currentText()


class TC_Drone_App(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Controls")
        self.drone = None
        self.setWindowIcon(QIcon("src/fly/assets/tc_aero_logo.png"))
        central = QWidget()
        main_layout = QVBoxLayout()

        self.tabs = QTabWidget()

        # implementing font
        # self.font_id = QFontDatabase.addApplicationFont(
        #     "src/fly/assets/BlackOpsOne-Regular.ttf"
        # )
        # if self.font_id != -1:  # Success check
        #     font_families = QFontDatabase.applicationFontFamilies(self.font_id)
        #     custom_font_name = font_families[0]  # Usually index 0
        # else:
        #     print("Font failed to load")
        #     custom_font_name = "Arial"  # fallback

        general_widget = QWidget()
        general_layout = QVBoxLayout()
        
        self.port_edit = HistoryLineEdit()

        data = pull_from_json()
        latest_port = data.get(drone_instance_json, {}).get("port")
        if latest_port:
            self.port_edit.lineEdit().setText(latest_port)

        self.logo = QLabel("TCHS Aero GUI v1")
        self.console = QTextEdit()
        self.console.setStyleSheet("background-color: black; color: white;")
        self.console.setReadOnly(True)

        self.button_connect = QPushButton("Connect to the drone")
        self.button_connect.clicked.connect(self.on_connect)

        self.button_takeoff = QPushButton("2. Takeoff (10m)")
        self.button_takeoff.setEnabled(False)
        self.button_takeoff.clicked.connect(self.on_takeoff)

        self.button_land = QPushButton("3. Land The Drone")
        self.button_land.setCheckable(True)
        self.button_land.setEnabled(False)
        self.button_land.clicked.connect(self.on_land)

        general_layout.addWidget(self.port_edit)
        general_layout.addWidget(self.button_connect)
        general_layout.addWidget(self.button_takeoff)
        general_layout.addWidget(self.button_land)
        general_widget.setLayout(general_layout)
        self.tabs.addTab(general_widget, "General")

        movement_widget = QWidget()
        movement_layout = QVBoxLayout()

        self.movement_controls_text = QLabel("Movement Controls")

        grid = QGridLayout()
        # buttons for drone movement
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
        self.button_rth = QPushButton("Return to\nHome")
        self.button_rth.clicked.connect(self.return_to_launch)

        # dynamic button sizes acccording to the user's expansion of the window
        for btn in [self.button_connect, self.button_takeoff, self.button_land, 
                    self.button_stop_movement, self.button_rth, self.button_up, 
                    self.button_down, self.button_left, self.button_right, 
                    self.button_forward, self.button_backward]:
            policy = btn.sizePolicy()
            policy.setVerticalPolicy(QSizePolicy.Policy.Expanding)
            btn.setSizePolicy(policy)
            btn.setMinimumHeight(60)

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
        self.distance_text = QLabel("Distance")
        self.distance = QDoubleSpinBox()
        self.yaw_text = QLabel("Yaw")
        self.yaw = QDoubleSpinBox()

        movement_layout.addWidget(self.movement_controls_text)
        movement_layout.addLayout(grid)
        movement_layout.addWidget(self.velocity_text)
        movement_layout.addWidget(self.velocity)
        movement_layout.addWidget(self.yaw_text)
        movement_layout.addWidget(self.yaw)
        movement_layout.addWidget(self.distance_text)
        movement_layout.addWidget(self.distance)
        movement_widget.setLayout(movement_layout)
        self.tabs.addTab(movement_widget, "Movement")

        main_layout.addWidget(self.logo)
        main_layout.addWidget(self.console)
        main_layout.addWidget(self.tabs)
        central.setLayout(main_layout)
        self.setCentralWidget(central)

    def log(self, msg):
        self.console.append(msg)
        print(msg)

    async def takeoff_land_toggle(self):
        async for telemetry in self.drone.drone.telemetry.in_air():
            if telemetry:
                self.button_land.setEnabled(True)
                self.button_takeoff.setEnabled(False)
            else:
                self.button_land.setEnabled(False)
                self.button_takeoff.setEnabled(True)

    @asyncSlot()
    async def on_connect(self):
        self.button_connect.setEnabled(False)
        port = self.port_edit.text().strip()
        if not is_valid_port(port):
            self.log("-- Invalid port format. Please specify a valid UDP, TCP, or Serial port.")
            self.button_connect.setEnabled(True)
            return

        try:
            self.drone = Drone(port)
            self.log("Connecting...")
            connected = await self.drone.connect()
            if connected:
                self.log("-- Connected")
                lat, lon, alt = await self.drone.current_position()
                self.log(f"Pos: {lat:.5f}, {lon:.5f}, {alt:.1f}m")
                asyncio.create_task(self.takeoff_land_toggle())
            else:
                self.log(f"-- Failed to connect to the drone within {self.drone.connection_timeout} seconds.")
                self.button_connect.setEnabled(True)
        except Exception as e:
            self.log(f"Error: {e}")


    @asyncSlot()
    async def on_takeoff(self):
        self.log("Starting Takeoff Sequence...")
        self.button_takeoff.setEnabled(False)
        try:
            await self.drone.takeoff(10.0)
            self.log("Takeoff...")
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

    async def _execute_movement(self, direction, method):
        self.log(f"Starting Moving {direction} Sequence...")
        velocity = self.velocity.value()
        yaw = self.yaw.value()
        distance = self.distance.value()

        if distance < 0 or velocity < 0:
            self.log("All values must be greater than 0.")
            return
        elif not distance or not velocity:
            self.log("Distance and Velocity are required to execute this command.")
            return

        try:
            self.log(f"Moving {direction}...")
            await method(distance=distance, velocity=velocity, yaw=yaw)
            self.log("Movement finished")
        except Exception as e:
            self.log(f"Moving {direction} Error: {e}")
            self.log("Are you connected?")

    @asyncSlot()
    async def move_up(self):
        await self._execute_movement("Up", self.drone.move_up_offset)

    @asyncSlot()
    async def move_down(self):
        await self._execute_movement("Down", self.drone.move_down_offset)

    @asyncSlot()
    async def move_left(self):
        await self._execute_movement("Left", self.drone.move_left_offset)

    @asyncSlot()
    async def move_right(self):
        await self._execute_movement("Right", self.drone.move_right_offset)

    @asyncSlot()
    async def move_forward(self):
        await self._execute_movement("Forward", self.drone.move_forward_offset)

    @asyncSlot()
    async def move_backward(self):
        await self._execute_movement("Backward", self.drone.move_backward_offset)

    @asyncSlot()
    async def stoping_movement(self):
        self.log("Stoping Movement...")
        try:
            await self.drone.stop_movement()
        except Exception as e:
            self.log(f"Stoping Movement Error: {e}")

    @asyncSlot()
    async def return_to_launch(self):
        self.log("Starting to Return to Launch...")
        try:
            await self.drone.return_to_home()
            self.log("Returning to Launch...")
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