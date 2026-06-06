import sys
import asyncio
import re

from fly.core.drone import Drone
from fly.core.mission import Mission
from fly.core.dataManager import (
    update_port_data,
    pull_port_data,
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
    QComboBox,
    QTabWidget,
    QGridLayout,
    QFileDialog,
    QSizePolicy,
    QMessageBox,
    QMenu,
    QCheckBox,
    QToolButton,
)
from PyQt6.QtGui import QIcon, QAction
from PyQt6.QtCore import Qt
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

class HistoryLineEdit(QComboBox):
    def __init__(self, max_history=5, parent=None):
        super().__init__(parent)
        self.max_history = max_history
        self.setEditable(True)
        self.lineEdit().setPlaceholderText("Port, e.g. udpin://0.0.0.0:14540")
        self.load_history()

    def set_empty_state(self):
        self.addItem("No Port History")
        self.model().item(0).setFlags(Qt.ItemFlag.NoItemFlags)
        self.setCurrentIndex(-1)

    def update_items(self, items):
        self.clear()
        if not items:
            self.set_empty_state()
            return

        if self.count() == 1 and self.itemText(0) == "No Port History":
            self.removeItem(0)
            self.addItems(items)
            return
        
        self.addItems(items)

    def load_history(self):
        data = pull_port_data()
        _, history = data 
        self.update_items(history)

    def save_history(self):
        data = pull_port_data()
        if not data:
            return

        port, history = data
        if self.currentText() in history:
            history.remove(self.currentText())
        history.insert(0, self.currentText())
        port = history[0]
        update_port_data(port = port, history = history)
        self.update_items(history)

    def text(self):
        return self.currentText()


class TC_Drone_App(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Controls")
        self.drone = None
        self.mission = None
        self.setWindowIcon(QIcon("src/fly/assets/tc_aero_logo.png"))
        central = QWidget()
        main_layout = QVBoxLayout()
        self._tasks = []
        self.tabs = QTabWidget()
        self.connected = False
        

        general_widget = QWidget()
        general_layout = QVBoxLayout()

        mission_widget = QWidget()
        mission_layout = QVBoxLayout()

        self.port_edit = HistoryLineEdit()

        data = pull_port_data()
        if data[0]:
            self.port_edit.lineEdit().setText(data[0])

        printo = QLabel("TCHS Aero GUI v1")
        self.console = QTextEdit(self)
        self.console.setStyleSheet("background-color: black; color: white;")
        self.console.setReadOnly(True)

#---- general tab

        self.button_connect = QPushButton("Connect")
        self.button_connect.clicked.connect(self.on_connect)

        self.button_disconnect = QPushButton("Disconnect")
        self.button_disconnect.setEnabled(False)
        self.button_disconnect.clicked.connect(self.on_disconnect)

        self.button_takeoff = QPushButton("Takeoff")
        self.button_takeoff.setEnabled(False)
        self.button_takeoff.clicked.connect(self.on_takeoff)

        self.button_land = QPushButton("Land")
        self.button_land.setEnabled(False)
        self.button_land.clicked.connect(self.on_land)

        general_layout.addWidget(self.port_edit)
        general_layout.addWidget(self.button_connect)
        general_layout.addWidget(self.button_disconnect)
        general_layout.addWidget(self.button_takeoff)
        general_layout.addWidget(self.button_land)
        general_widget.setLayout(general_layout)
        self.tabs.addTab(general_widget, "General")

#-- mission tab
        try:
            self.StartMission = QPushButton("Start the Mission (TAKEOFF FIRST)")
            self.StartMission.clicked.connect(self.StartMissionFunc)
            self.StartMission.setEnabled(False)

            self.UploadMission = QPushButton('Upload Mission')
            self.UploadMission.clicked.connect(self.UploadMissionFunc)

            self.ReturnToLaunch = QCheckBox('Drone Returns to Launch Waypoint')
            self.ReturnToLaunch.setCheckState(Qt.CheckState.Unchecked)
            self.ReturnToLaunch.stateChanged.connect(self.RTLFunction)

            self.ResetMission = QPushButton("Reset the Mission") #makes drone go to 1st waypoint
            self.ResetMission.clicked.connect(self.ResetMissionFunc)

            self.ClearMission = QPushButton("Clear the Mission") #deletes the uploaded mission so it starts with a clean slate
            self.ClearMission.clicked.connect(self.ClearMissionFunc)

            self.PauseMission = QPushButton('Pause Ongoing Mission')
            self.PauseMission.clicked.connect(self.PauseMissionFunc)

            mission_layout.addWidget(self.UploadMission)
            mission_layout.addWidget(self.StartMission)
            mission_layout.addWidget(self.ReturnToLaunch)
            mission_layout.addWidget(self.ResetMission)
            mission_layout.addWidget(self.ClearMission)
            mission_layout.addWidget(self.PauseMission)
            mission_widget.setLayout(mission_layout)
            self.tabs.addTab(mission_widget, "Mission")
        except Exception as e:
            print(e)





#---movement tab

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
        for btn in [
            self.button_connect, self.button_disconnect,
            self.button_takeoff, self.button_land,
            self.button_stop_movement, self.button_rth,
            self.button_up, self.button_down, self.button_left,
            self.button_right, self.button_forward, self.button_backward
        ]:
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

        main_layout.addWidget(printo)
        main_layout.addWidget(self.console)
        main_layout.addWidget(self.tabs)
        central.setLayout(main_layout)
        self.setCentralWidget(central)

#---battery percentage counter

        self.battery_menu = QMenu()
        self.battery_menu.setToolTipsVisible(True)

        self.batt_percent_action = QAction("Remaining: --(%)")
        self.batt_voltage_action = QAction("Voltage: --(V)")
        self.batt_temp_action = QAction("Temperature: --(C)")
        self.batt_time_action = QAction("Time Remaining: --(s)")

        self.battery_menu.addAction(self.batt_percent_action)
        self.battery_menu.addAction(self.batt_voltage_action)
        self.battery_menu.addAction(self.batt_temp_action)
        self.battery_menu.addAction(self.batt_time_action)

        self.battery_button = QToolButton()
        self.battery_button.setText("Battery: --(%)")
        self.battery_button.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        self.battery_button.setMenu(self.battery_menu)
        self.battery_button.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextOnly)

        self.statusBar().addPermanentWidget(self.battery_button)

    class StreamToTextBox:
        def __init__(self, text_edit):
            self.text_edit = text_edit
   
        def write(self, text):
            self.text_edit.insertPlainText(text)
            self.text_edit.ensureCursorVisible()
        
        def flush(self):
            pass
    
    async def update_battery(self):
        async for telemetry in self.drone.drone.telemetry.battery():
            self.battery_button.setText(f"Battery: {telemetry.remaining_percent}(%)")
            self.batt_percent_action.setText(f"Remaining: {telemetry.remaining_percent}(%)")
            self.batt_voltage_action.setText(f"Voltage: {round(telemetry.voltage_v)}(V)")
            self.batt_temp_action.setText(f"Temperature: {round(telemetry.temperature_degc, 1)}(C)")
            self.batt_time_action.setText(f"Time Remaining: {telemetry.time_remaining_s}(s)")

    async def takeoff_land_toggle(self):

        async for is_in_air in self.drone.drone.telemetry.in_air():
            if not self.connected:
                self.button_land.setEnabled(False)
                self.button_takeoff.setEnabled(False)
                continue

            if not is_in_air:
                self.button_land.setEnabled(False)
                self.button_takeoff.setEnabled(True)
            else:
                self.button_land.setEnabled(True)
                self.button_takeoff.setEnabled(False)
    
    async def run_checks_on_connect(self):
        battery = asyncio.ensure_future(self.update_battery())
        takeoff_toggle = asyncio.ensure_future(self.takeoff_land_toggle())
        self._tasks.extend([battery, takeoff_toggle])

    def closeEvent(self, event):
        if self.connected:
            reply = QMessageBox.warning(self, "Woah there!", "Make sure that you disconnected your drone before closing this program.")
            event.ignore()
            return

        reply = QMessageBox.question(self, 'Warning!',
                                     "Are you sure you want to quit?",
                                     QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                     QMessageBox.StandardButton.No)
    
        if reply == QMessageBox.StandardButton.Yes:
            event.accept()
        else:
            event.ignore()
 
    @asyncSlot()
    async def on_connect(self):
        sys.stdout = self.StreamToTextBox(self.console)
        self.button_connect.setEnabled(False)
        port = self.port_edit.text().strip()
        if not is_valid_port(port):
            print("-- Invalid port format. Please specify a valid UDP, TCP, or Serial port.")
            self.button_connect.setEnabled(True)
            self.button_disconnect.setEnabled(False)
            return

        try:
            self.drone = Drone(port)
            self.connected = await self.drone.connect()
            if self.connected:
                self.button_disconnect.setEnabled(True)
                self.port_edit.save_history()
                lat, lon, alt = await self.drone.current_position()
                print(f"\nCurrent Position\n    - Latitude: {lat:.5f}\n    - Longitude: {lon:.5f}\n    - Altitude: {alt:.1f} (meters)\n")
                await self.run_checks_on_connect()
            else:
                sys.stdout = sys.__stdout__
                print(f"-- Failed to connect to the drone within {self.drone.connection_timeout} seconds.")
                self.button_connect.setEnabled(True)
                self.button_disconnect.setEnabled(False)
        except Exception as e:
            sys.stdout = sys.__stdout__
            print(f"Error: {e}")


    @asyncSlot()
    async def on_disconnect(self):
        if self.drone is None:
            print("-- No drone is connected.")
            return

        print("-- Disconnecting...")

        try:
            for task in self._tasks:
                task.cancel()
            self._tasks.clear()

            del self.drone

            self.button_connect.setEnabled(True)
            self.button_disconnect.setEnabled(False)
            self.button_takeoff.setEnabled(False)
            self.button_land.setEnabled(False)

            self.battery_button.setText("Battery: --(%)")
            self.batt_percent_action.setText("Remaining: --(%)")
            self.batt_voltage_action.setText("Voltage: --(V)")
            self.batt_temp_action.setText("Temperature: --(C)")
            self.batt_time_action.setText("Time Remaining: --(s)")

            print("-- Disconnected")
            self.connected = False
            sys.stdout = sys.__stdout__
        except Exception as e:
            print(f"Disconnect Error: {e}")


    @asyncSlot()
    async def on_takeoff(self):
        if self.drone is None:
            print("\nYou are not connected to a drone.\n")
            return

        print("-- Starting Takeoff Sequence...")
        try:
            await self.drone.takeoff(10.0)
            print("-- Takeoff...")
        except Exception as e:
            print(f"-- Takeoff Error: {e}")
    
    @asyncSlot()
    async def on_land(self):
        print("-- Starting Landing Sequence...")
        try:
            await self.drone.land()
            print("-- Landing...")

        except Exception as e:
            print(f"-- Landing Error: {e}")

    async def execute_movement(self, direction, method):
        print(f"-- Starting Moving {direction} Sequence...")
        velocity = self.velocity.value()
        yaw = self.yaw.value()
        distance = self.distance.value()

        if distance < 0 or velocity < 0:
            print("-- All values must be greater than 0.")
            return
        elif not distance or not velocity:
            print("-- Distance and Velocity are required to execute this command.")
            return

        try:
            print(f"-- Moving {direction}...")
            await method(distance=distance, velocity=velocity, yaw=yaw)
            print("-- Movement finished")
        except Exception as e:
            print(f"-- Moving {direction} Error: {e}")
            print("-- Are you connected?")

    @asyncSlot()
    async def move_up(self):
        await self.execute_movement("Up", self.drone.move_up_offset)

    @asyncSlot()
    async def move_down(self):
        await self.execute_movement("Down", self.drone.move_down_offset)

    @asyncSlot()
    async def move_left(self):
        await self.execute_movement("Left", self.drone.move_left_offset)

    @asyncSlot()
    async def move_right(self):
        await self.execute_movement("Right", self.drone.move_right_offset)

    @asyncSlot()
    async def move_forward(self):
        await self.execute_movement("Forward", self.drone.move_forward_offset)

    @asyncSlot()
    async def move_backward(self):
        await self.execute_movement("Backward", self.drone.move_backward_offset)

    @asyncSlot()
    async def stoping_movement(self):
        if not self.connected:
            return

        print("-- Stopping Movement...")
        try:
            await self.drone.stop_movement()
        except Exception as e:
            print(f"-- Stopping Movement Error: {e}")

    @asyncSlot()
    async def return_to_launch(self):
        if not self.connected:
            return

        print("-- Starting to Return to Launch...")
        try:
            await self.drone.return_to_home()
            print("-- Returning to Launch...")
        except Exception as e:
            print(f"-- Return to Launch Error: {e}")


    @asyncSlot()
    async def StartMissionFunc(self):
        if not self.connected:
            return

        print("-- Starting Mission...")
        try:
            await self.mission.start_mission(self.drone)
            print('-- Mission Started...')
        except Exception as e:
            print(f'-- Starting Mission Error: {e}')

    def UploadMissionFunc(self):
        
        if not self.connected:
            return

        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Mission File",
            "",
            "JSON files (*.json)"
        )

        if not file_path:
            return

        task = asyncio.ensure_future(self.do_upload(file_path))
        self._tasks.append(task)

    async def do_upload(self, file_path):
        try:
            print("-- Uploading Mission...")
            self.mission = Mission(file_path)
            await self.mission.upload_mission(self.drone)
            self.StartMission.setEnabled(True)
            print("-- Mission Uploaded Successfully")
        except Exception as e:
            print(f'-- Uploading Mission Error: {e}')

    @asyncSlot()
    async def RTLFunction(self):
        pass

    @asyncSlot()
    async def ResetMissionFunc(self):
        pass

    @asyncSlot()
    async def ClearMissionFunc(self):
        pass
    
    @asyncSlot()
    async def PauseMissionFunc(self):
        pass

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
