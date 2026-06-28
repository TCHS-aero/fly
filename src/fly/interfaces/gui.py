import sys
import asyncio
import re

from pathlib import Path
from fly.core.drone import Drone
from fly.core.mission import Mission
from fly.core.dataManager import (
    update_port_data,
    pull_data,
    update_mission_data,
)

from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QPushButton,
    QLabel,
    QDoubleSpinBox,
    QVBoxLayout,
    QHBoxLayout,
    QWidget,
    QTextEdit,
    QComboBox,
    QTabWidget,
    QGridLayout,
    QFileDialog,
    QProgressBar,
    QSizePolicy,
    QMessageBox,
    QMenu,
    QCheckBox,
    QToolButton,
    QLineEdit
)
from PyQt6.QtGui import QIcon, QAction, QPixmap
from PyQt6.QtCore import Qt, QSize
from qasync import asyncSlot, QEventLoop

base_dir = Path(__file__).resolve().parent.parent

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
        data = pull_data()
        history = data.get("port-history", None)
        self.update_items(history)

    def save_history(self):  
        data = pull_data()
        if not data:
            return

        port = data["port"]
        history = data["port-history"]
        if self.currentText() in history:
            history.remove(self.currentText())
        history.insert(0, self.currentText())
        port = history[0]
        update_port_data(port = port, history = history)
        self.update_items(history)

    def text(self):
        return self.currentText()

class Waypoint_Info_Window(QWidget):
    def __init__(self, drone, mission, connected):
        super().__init__()

        self.drone = drone
        self.mission = mission
        self.connected = connected

        self.setWindowTitle("Waypoint Info")
        self.resize(800, 500)

        self.setMinimumHeight(500)
        self.setMaximumHeight(500)

        self.box_layout = QHBoxLayout()

        self.next_layout = QVBoxLayout()
        self.current_layout = QVBoxLayout()

        self.next_grid = QGridLayout()
        self.current_grid = QGridLayout()

        self.next_title = QLabel("Next Objective Waypoint")
        self.current_title = QLabel("Last Completed Waypoint")

        self.nwp_latitude = QLineEdit()
        self.nwp_longitude = QLineEdit()
        self.nwp_relative_altitude = QLineEdit()
        self.nwp_speed = QLineEdit()
        self.nwp_is_fly_through = QLineEdit()
        self.nwp_gimbal_pitch = QLineEdit()
        self.nwp_gimbal_yaw = QLineEdit()
        self.nwp_camera_action = QLineEdit()
        self.nwp_loiter_time = QLineEdit()
        self.nwp_camera_photo_interval = QLineEdit()
        self.nwp_acceptance_radius = QLineEdit()
        self.nwp_yaw = QLineEdit()
        self.nwp_camera_photo_distance = QLineEdit()
        self.nwp_vehicle_action = QLineEdit()

        self.cwp_latitude = QLineEdit()
        self.cwp_longitude = QLineEdit()
        self.cwp_relative_altitude = QLineEdit()
        self.cwp_speed = QLineEdit()
        self.cwp_is_fly_through = QLineEdit()
        self.cwp_gimbal_pitch = QLineEdit()
        self.cwp_gimbal_yaw = QLineEdit()
        self.cwp_camera_action = QLineEdit()
        self.cwp_loiter_time = QLineEdit()
        self.cwp_camera_photo_interval = QLineEdit()
        self.cwp_acceptance_radius = QLineEdit()
        self.cwp_yaw = QLineEdit()
        self.cwp_camera_photo_distance = QLineEdit()
        self.cwp_vehicle_action = QLineEdit()

        self.next_waypoint_fields = [
            ("Latitude (deg)", self.nwp_latitude),
            ("Longitude (deg)", self.nwp_longitude),
            ("Relative Altitude (m)", self.nwp_relative_altitude),
            ("Speed (m/s)", self.nwp_speed),
            ("Is Fly Through", self.nwp_is_fly_through),
            ("Gimbal Pitch (deg)", self.nwp_gimbal_pitch),
            ("Gimbal Yaw (deg)", self.nwp_gimbal_yaw),
            ("Camera Action", self.nwp_camera_action),
            ("Loiter Time (s)", self.nwp_loiter_time),
            ("Camera Photo Interval (s)", self.nwp_camera_photo_interval),
            ("Acceptance Radius (m)", self.nwp_acceptance_radius),
            ("Yaw (deg)", self.nwp_yaw),
            ("Camera Photo Distance (m)", self.nwp_camera_photo_distance),
            ("Vehicle Action", self.nwp_vehicle_action),
        ]

        self.current_waypoint_fields = [
            ("Latitude (deg)", self.cwp_latitude),
            ("Longitude (deg)", self.cwp_longitude),
            ("Relative Altitude (m)", self.cwp_relative_altitude),
            ("Speed (m/s)", self.cwp_speed),
            ("Is Fly Through", self.cwp_is_fly_through),
            ("Gimbal Pitch (deg)", self.cwp_gimbal_pitch),
            ("Gimbal Yaw (deg)", self.cwp_gimbal_yaw),
            ("Camera Action", self.cwp_camera_action),
            ("Loiter Time (s)", self.cwp_loiter_time),
            ("Camera Photo Interval (s)", self.cwp_camera_photo_interval),
            ("Acceptance Radius (m)", self.cwp_acceptance_radius),
            ("Yaw (deg)", self.cwp_yaw),
            ("Camera Photo Distance (m)", self.cwp_camera_photo_distance),
            ("Vehicle Action", self.cwp_vehicle_action),
        ]

        for row, (label_text, field) in enumerate(self.next_waypoint_fields):
            field.setReadOnly(True)
            self.next_grid.addWidget(QLabel(label_text), row, 0)
            self.next_grid.addWidget(field, row, 1)

        for row, (label_text, field) in enumerate(self.current_waypoint_fields):
            field.setReadOnly(True)
            self.current_grid.addWidget(QLabel(label_text), row, 0)
            self.current_grid.addWidget(field, row, 1)

        self.next_layout.addWidget(self.next_title)
        self.next_layout.addLayout(self.next_grid)

        self.current_layout.addWidget(self.current_title)
        self.current_layout.addLayout(self.current_grid)

        self.box_layout.addLayout(self.current_layout)
        self.box_layout.addLayout(self.next_layout)


        self.setLayout(self.box_layout)
        main_widget = QWidget()
        main_widget.setLayout(self.box_layout)

        self.tabs = QTabWidget()
        self.tabs.addTab(main_widget, "Info")

        window_layout = QVBoxLayout()
        window_layout.addWidget(self.tabs)
        self.setLayout(window_layout)       

    async def refresh_waypoint_information(self, current_item, next_item):
        try:
            self.set_next_waypoint_info(next_item)
            self.set_current_waypoint_info(current_item)

            print("-- Waypoint info updated.")

        except Exception as e:
            print(f"-- Waypoint Info Error: {e}")

    def set_next_waypoint_info(self, next_item):
        if next_item is None:
            next_item = "No Waypoint Available"
            self.nwp_latitude.setText(next_item)
            self.nwp_longitude.setText(next_item)
            self.nwp_relative_altitude.setText(next_item)
            self.nwp_speed.setText(next_item)
            self.nwp_is_fly_through.setText(next_item)
            self.nwp_gimbal_pitch.setText(next_item)
            self.nwp_gimbal_yaw.setText(next_item)
            self.nwp_camera_action.setText(next_item)
            self.nwp_loiter_time.setText(next_item)
            self.nwp_camera_photo_interval.setText(next_item)
            self.nwp_acceptance_radius.setText(next_item)
            self.nwp_yaw.setText(next_item)
            self.nwp_camera_photo_distance.setText(next_item)
            self.nwp_vehicle_action.setText(next_item)        
        else:
            self.nwp_latitude.setText(str(next_item.latitude_deg))  
            self.nwp_longitude.setText(str(next_item.longitude_deg))
            self.nwp_relative_altitude.setText(str(next_item.relative_altitude_m))
            self.nwp_speed.setText(str(next_item.speed_m_s))
            self.nwp_is_fly_through.setText(str(next_item.is_fly_through))
            self.nwp_gimbal_pitch.setText(str(next_item.gimbal_pitch_deg))
            self.nwp_gimbal_yaw.setText(str(next_item.gimbal_yaw_deg))
            self.nwp_camera_action.setText(next_item.camera_action.name)
            self.nwp_loiter_time.setText(str(next_item.loiter_time_s))
            self.nwp_camera_photo_interval.setText(str(next_item.camera_photo_interval_s))
            self.nwp_acceptance_radius.setText(str(next_item.acceptance_radius_m))
            self.nwp_yaw.setText(str(next_item.yaw_deg))
            self.nwp_camera_photo_distance.setText(str(next_item.camera_photo_distance_m))
            self.nwp_vehicle_action.setText(next_item.vehicle_action.name)

    def set_current_waypoint_info(self, current_item):
        if current_item is None:
            current_item = "No Waypoint Available"
            self.cwp_latitude.setText(current_item)
            self.cwp_longitude.setText(current_item)
            self.cwp_relative_altitude.setText(current_item)
            self.cwp_speed.setText(current_item)
            self.cwp_is_fly_through.setText(current_item)
            self.cwp_gimbal_pitch.setText(current_item)
            self.cwp_gimbal_yaw.setText(current_item)
            self.cwp_camera_action.setText(current_item)
            self.cwp_loiter_time.setText(current_item)
            self.cwp_camera_photo_interval.setText(current_item)
            self.cwp_acceptance_radius.setText(current_item)
            self.cwp_yaw.setText(current_item)
            self.cwp_camera_photo_distance.setText(current_item)
            self.cwp_vehicle_action.setText(current_item)        
        else:
            self.cwp_latitude.setText(str(current_item.latitude_deg))
            self.cwp_longitude.setText(str(current_item.longitude_deg))
            self.cwp_relative_altitude.setText(str(current_item.relative_altitude_m))
            self.cwp_speed.setText(str(current_item.speed_m_s))
            self.cwp_is_fly_through.setText(str(current_item.is_fly_through))
            self.cwp_gimbal_pitch.setText(str(current_item.gimbal_pitch_deg))
            self.cwp_gimbal_yaw.setText(str(current_item.gimbal_yaw_deg))
            self.cwp_camera_action.setText(current_item.camera_action.name)
            self.cwp_loiter_time.setText(str(current_item.loiter_time_s))
            self.cwp_camera_photo_interval.setText(str(current_item.camera_photo_interval_s))
            self.cwp_acceptance_radius.setText(str(current_item.acceptance_radius_m))
            self.cwp_yaw.setText(str(current_item.yaw_deg))
            self.cwp_camera_photo_distance.setText(str(current_item.camera_photo_distance_m))
            self.cwp_vehicle_action.setText(str(current_item.vehicle_action.name))

class TC_Drone_App(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Ground Control")
        self.drone = None
        self.mission = Mission()
        window_icon = base_dir / "assets" / "logo_small.png"
        self.setWindowIcon(QIcon(str(window_icon)))
        self.setMinimumWidth(400)
        central = QWidget()
        main_layout = QVBoxLayout()
        self._tasks = []
        self.tabs = QTabWidget()
        self.connected = False
        
        self.new_window = None

        general_widget = QWidget()
        general_layout = QVBoxLayout()

        mission_widget = QWidget()
        mission_layout = QVBoxLayout()

        progress_layout = QHBoxLayout()

        self.port_edit = HistoryLineEdit()
        self.clear_history_button = QPushButton()
        self.clear_history_button.setFixedSize(28, 28)
        icon_path = base_dir / "assets" / "broom.png"
        self.clear_history_button.setIcon(QIcon(str(icon_path)))
        self.clear_history_button.setEnabled(True)
        self.clear_history_button.clicked.connect(self.clear_history)

        data = pull_data()
        if data["port"]:
            self.port_edit.lineEdit().setText(data["port"])

        #--- Title Icon
        logo_label = QLabel()
        small_pixmap = QPixmap(str(window_icon)).scaled(
            QSize(150, 150), 
            Qt.AspectRatioMode.KeepAspectRatio, 
            Qt.TransformationMode.SmoothTransformation
        )

        logo_label.setPixmap(small_pixmap)
        logo_label.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter)

        #--- Console
        self.console = QTextEdit(self)
        self.console.setStyleSheet("background-color: black; color: white;")
        self.console.setReadOnly(True)

        #--- Progress Bar
        self.progress_bar = QProgressBar(self)
        self.progress_bar.setMaximum(0)
        self.progress_bar.setValue(0)

        self.waypoint_info = QPushButton("+")
        self.waypoint_info.setFixedSize(30,30)
        self.waypoint_info.clicked.connect(self.openNewWindow)

        #--- Progress Bar Title + file name
        self.pbtitle = QLabel(self)
        self.pbtitle.setText("")

        progress_layout.addWidget(self.progress_bar)
        progress_layout.addWidget(self.waypoint_info)

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

        self.history_group = QHBoxLayout()
        self.history_group.addWidget(self.port_edit)
        self.history_group.addWidget(self.clear_history_button)

        general_layout.addLayout(self.history_group)
        general_layout.addWidget(self.button_connect)
        general_layout.addWidget(self.button_disconnect)
        general_layout.addWidget(self.button_takeoff)
        general_layout.addWidget(self.button_land)
        general_widget.setLayout(general_layout)
        self.tabs.addTab(general_widget, "General")

        self.StartMission = QPushButton("Start the Mission")
        self.StartMission.clicked.connect(self.StartMissionFunc)

        self.UploadMission = QPushButton('Upload Mission')
        self.UploadMission.clicked.connect(self.UploadMissionFunc)

        self.ResetMission = QPushButton("Reset the Mission") #makes drone go to 1st waypoint
        self.ResetMission.clicked.connect(self.ResetMissionFunc)

        self.ClearMission = QPushButton("Clear the Mission") #deletes the uploaded mission so it starts with a clean slate
        self.ClearMission.clicked.connect(self.ClearMissionFunc)

        self.PauseMission = QPushButton('Pause Ongoing Mission')
        self.PauseMission.clicked.connect(self.PauseMissionFunc)

        mission_layout.addWidget(self.UploadMission)
        mission_layout.addWidget(self.StartMission)
        mission_layout.addWidget(self.ResetMission)
        mission_layout.addWidget(self.ClearMission)
        mission_layout.addWidget(self.PauseMission)
        mission_widget.setLayout(mission_layout)
        self.tabs.addTab(mission_widget, "Mission")

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
            self.button_right, self.button_forward, self.button_backward,
            self.StartMission, self.UploadMission, self.PauseMission, 
            self.ResetMission, self.ClearMission, 
        ]:
            policy = btn.sizePolicy()
            policy.setVerticalPolicy(QSizePolicy.Policy.Expanding)
            btn.setSizePolicy(policy)
            btn.setMinimumHeight(60)

        policyWP = self.waypoint_info.sizePolicy()
        policyWP.setHorizontalPolicy(QSizePolicy.Policy.Fixed)
        policyWP.setVerticalPolicy(QSizePolicy.Policy.Fixed)
        self.waypoint_info.setSizePolicy(policyWP)

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

        main_layout.addWidget(logo_label)
        main_layout.addWidget(self.console)
        main_layout.addWidget(self.pbtitle)
        main_layout.addLayout(progress_layout)
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

    @asyncSlot()
    async def clear_history(self):
        print("-- Clearing port history...")
        update_port_data(history = [])
        self.port_edit.load_history()
        data = pull_data()
        if data["port"]:
            self.port_edit.lineEdit().setText(data["port"])
        print("-- Port history cleared! Your current port is saved.")

    #--- opens new window to waypoint info
    @asyncSlot()
    async def openNewWindow(self):
        if not self.drone:
            print("-- Please connect the drone to view additional waypoint information/")
            return

        if self.new_window is None:
            self.new_window = Waypoint_Info_Window(self.drone, self.mission, self.connected)

        self.new_window.show()
        self.new_window.raise_()
        self.new_window.activateWindow()

        try:
            if not await self.mission.drone_have_mission(self.drone):
                await self.new_window.refresh_waypoint_information(None, None)
                return

            current_progress = pull_data()["current-mission-progress"]
            current_item, next_item = await self.mission.get_current_next_waypoint_info(self.drone, current_progress)
            await self.new_window.refresh_waypoint_information(current_item, next_item)
        except Exception as e:
            print(e)

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
   
    async def mission_button_toggles(self):
        while True:
            if await self.mission.is_drone_on_mission(self.drone):
                self.StartMission.setEnabled(False)
                self.UploadMission.setEnabled(False)
                self.ClearMission.setEnabled(False)
                self.PauseMission.setEnabled(True)
                self.ResetMission.setEnabled(True)
            else:
                self.StartMission.setEnabled(True)
                self.UploadMission.setEnabled(True)
                self.ClearMission.setEnabled(True)
                self.PauseMission.setEnabled(False)
                self.ResetMission.setEnabled(True)
            await asyncio.sleep(2)
   
    async def mission_uploaded(self):
        while True:
            if await self.mission.drone_have_mission(self.drone):
                self.pbtitle.setText("Mission Progress:")
                self.progress_bar.setEnabled(True)
            else:
                self.pbtitle.setText("No Uploaded Mission")
                self.progress_bar.setMaximum(0)
                self.progress_bar.setValue(0)
                self.progress_bar.setFormat("")
                self.progress_bar.setEnabled(False)
            await asyncio.sleep(2)

    async def run_checks_on_connect(self):
        def log_task_result(task):
            try:
                task.result()
            except asyncio.CancelledError:
                return
            except Exception as e:
                print(f"Task failed: {e}")

        battery = asyncio.create_task(self.update_battery())
        takeoff_toggle = asyncio.create_task(self.takeoff_land_toggle())
        mission_buttons = asyncio.create_task(self.mission_button_toggles())
        mission_progress = asyncio.create_task(self.track_mission_progress())
        mission_uploaded = asyncio.create_task(self.mission_uploaded())

        battery.add_done_callback(log_task_result)
        takeoff_toggle.add_done_callback(log_task_result)
        mission_buttons.add_done_callback(log_task_result)
        mission_progress.add_done_callback(log_task_result)
        mission_uploaded.add_done_callback(log_task_result)

        self._tasks.extend([battery, takeoff_toggle, mission_buttons, mission_progress, mission_uploaded])

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
            if self.new_window:
                self.new_window.close()
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

                data = pull_data()
                if data:
                    current = data.get("current-mission-progress", None)
                    total = data.get("total-mission-progress", None)
        
                    if current == total:
                        self.progress_bar.setMaximum(1)
                        self.progress_bar.setValue(1)
                        self.progress_bar.setFormat("Mission Complete!")
                    else:
                        self.progress_bar.setMaximum(total)
                        self.progress_bar.setValue(current)
                        self.progress_bar.setFormat(f"Waypoint {current} / {total}")
 
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

    async def track_mission_progress(self):
        while True:
            await asyncio.sleep(2)
            current, total = await self.mission.get_mission_progress(self.drone)
            update_mission_data(current, total)

            if hasattr(self, "new_window") and self.new_window is not None:
                current_item, next_item = await self.mission.get_current_next_waypoint_info(self.drone, current)
                await self.new_window.refresh_waypoint_information(current_item, next_item)

            self.progress_bar.setMaximum(total)
            self.progress_bar.setValue(current)
            self.progress_bar.setFormat(f"Waypoint {current} / {total}")
            if current == total:
                self.progress_bar.setMaximum(1)
                self.progress_bar.setValue(1)
                self.progress_bar.setFormat("Mission Complete!")

        
    @asyncSlot()
    async def StartMissionFunc(self):
        if not self.connected:
            return

        print("-- Starting Mission...")
        try:
            async for state in self.drone.drone.telemetry.in_air(): 
                in_the_air = state
                break 

            if in_the_air:
                await self.mission.start_mission(self.drone)
                print('-- Mission Started...')
                self.progress_bar.setEnabled(True)

                while not await self.mission.is_mission_finished(self.drone):
                        await asyncio.sleep(1)

                print("-- Mission Finished!")
                if self.mission.RTL == True:
                    await self.return_to_launch()
                    print("-- Returning to launch...")
                else:
                    print("-- Not Returning to launch...")
                    return
            else:
                print('-- Takeoff required to Start Mission!...')
                return
            
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

        task = asyncio.create_task(self.do_upload(file_path))
        task.add_done_callback(lambda t: task.cancel())

    async def do_upload(self, file_path):
        try:
            print("-- Uploading Mission...")
            self.mission.parse_file(file_path)
            await self.mission.upload_mission(self.drone)
            print("-- Mission Uploaded Successfully")
        except Exception as e:
            print(f'-- Uploading Mission Error: {e}')


    @asyncSlot()
    async def ResetMissionFunc(self):
        try:
            print("-- Resetting Mission to 1st waypoint...")
            await self.mission.reset_mission(self.drone)
            print("-- Reset Mission Success!")
        except Exception as e:
            print(f"-- Reset Mission Error: {e}")

    @asyncSlot()
    async def ClearMissionFunc(self):
        try:
            print('-- Clearing old mission from drone...')
            await self.mission.clear_mission(self.drone)
            update_mission_data(current = None, total = None)
            print("-- Clearing Mission Success!")
        except Exception as e:
            print(f"Clearing Mission Error: {e}")
    
    @asyncSlot()
    async def PauseMissionFunc(self):
        try:
            print("-- Pausing Mission...")
            await self.mission.pause_mission(self.drone)
            print("-- Pause Mission Success!")

        except Exception as e:
            print(f'-- Pausing Mission Error: {e}')

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
