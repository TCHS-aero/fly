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
    QApplication, QMainWindow, QPushButton, QLabel,QDoubleSpinBox, QComboBox,
    QVBoxLayout, QWidget, QTextEdit, QTabWidget, QGridLayout,QSizePolicy
)
from PyQt6.QtGui import QFontDatabase, QIcon, QAction
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
        
        self.port_edit = HistoryLineEdit()

        data = pull_from_json()
        latest_port = data.get(drone_instance_json, {}).get("port")
        if latest_port:
            self.port_edit.lineEdit().setText(latest_port)

        self.logo = QLabel("TCHS Aero GUI v1")
        self.logo.setStyleSheet(f"font-size:40px; font-family: {custom_font_name};")

        # Status Bar
        self.status = QLabel("Status: Disconnected")
        self.battery_percentage = QLabel("(Battery: --%)")
        self.battery_voltage = QLabel("(Voltage: --V)")

        self.statusBar().addPermanentWidget(self.battery_percentage)
        self.statusBar().addPermanentWidget(self.battery_voltage)

        menubar = self.menuBar()
        menubar.setNativeMenuBar(False)

        status_menu = menubar.addMenu("Battery ")



        self.battery_consumed_action = QAction("Consumed: -- mAh", self)
        self.battery_consumed_action.setEnabled(False)
        status_menu.addAction(self.battery_consumed_action)
        
        self.battery_temp_action = QAction("Temperature: -- ℃", self)
        self.battery_temp_action.setEnabled(False)
        status_menu.addAction(self.battery_temp_action)

        self.battery_current_action = QAction("Current: -- A", self)
        self.battery_current_action.setEnabled(False)
        status_menu.addAction(self.battery_current_action)

        self.battery_time_action = QAction("Time Remaining: -- s", self)
        self.battery_time_action.setEnabled(False)
        status_menu.addAction(self.battery_time_action)

        
        self.console = QTextEdit()
        self.console.setStyleSheet("background-color: black")
        self.console.setReadOnly(True)

        self.button_connect = QPushButton("1. Connect to the drone")
        self.button_connect.setCheckable(True)
        self.button_connect.clicked.connect(self.on_connect)
        self.button_connect.setStyleSheet("""
            QPushButton {background-color: gray}
            QPushButton:checked {background-color: green; color: white; }
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

        self.movement_tab()
        self.tabs.addTab(self.movement_widget, "Movement")
  
        
        
        
        self.logo = QLabel("TCHS Aero GUI ver.1")
        self.logo.setStyleSheet("background-color: red")
        self.logo.setGeometry(400,100,500,50)

        main_layout.addWidget(self.logo)
        main_layout.addWidget(self.console)
        main_layout.addWidget(self.tabs)

        self.statusBar().addWidget(self.status)
        
        central.setLayout(main_layout)
        self.setCentralWidget(central)
    
    def start_battery_monitoring(self):
        if hasattr(self, 'battery_task') and self.battery_task:
            self.battery_task.cancel()
        
        self.battery_task = asyncio.create_task(self.monitor_battery())

    def stop_battery_monitoring(self):
        if hasattr(self, 'battery_task') and self.battery_task:
            self.battery_task.cancel()
            self.battery_task = None

    async def monitor_battery(self):
        while self.drone:
            try:
                battery_info = await self.drone.get_battery_info()


                bp = battery_info.get('percentage')
                try:
                    self.battery_percentage.setText(f"(Battery Percentage: {float(bp):.1f}%)")
                except Exception as e:
                    self.log(f"Battery Percentage Error: {e}")

                
                bv = battery_info.get('voltage')
                try:
                    self.battery_voltage.setText(f"(Battery Voltage: {float(bv):.1f}V)")
                except Exception as e:
                    self.log(f"Battery Voltage Error: {e}")


                bconsumed = battery_info.get('consumed')
                try:
                    self.battery_consumed_action.setText(f"Battery Consumed: {bconsumed}")

                except Exception as e:
                    self.log(f"Battery Consumed Error: {e}")
                    self.battery_consumed_action.setText("Battery Consumed: --mAh")

                btemp = battery_info.get('temperature')
                if isinstance(btemp, (int, float)):
                    self.battery_temp_action.setText(f"Temperature: {btemp:.1f}℃")
                else: 
                    self.battery_temp_action.setText("Temperature: -- ℃")

                bc = battery_info.get('battery_current')
                if isinstance(bc, (int, float)):
                    self.battery_current_action.setText(f"Current: {bc:.1f}A")
                else: 
                    self.battery_current_action.setText("Current: -- A")

                btr = battery_info.get('time_remaining')
                if isinstance(btr, (int, float)):
                    self.battery_time_action.setText(f"Time Remaining: {btr:.1f}s")
                else: 
                    self.battery_time_action.setText("Time Remaining: -- s")

            
            except Exception as e:
                self.log(f"Battery Monitoring Error: {e}")
                self.battery_percentage.setText("Battery Percentage: --%")
                self.battery_voltage.setText("Battery Voltage: --V")
                self.battery_consumed_action.setText("Battery Consumed: -- mAh") 
                self.battery_temp_action.setText("Battery Temperature: --℃")
                self.battery_current_action.setText("Battery Current: --A")
                self.battery_time_action.setText("Battery Time Remaining: --s")

            await asyncio.sleep(2)
        
        

    def log(self, msg):
        self.console.append(msg)
        print(msg)

    def movement_tab(self):

        self.movement_widget = QWidget()
        movement_layout = QVBoxLayout()
        grid = QGridLayout()

        self.movement_controls_text = QLabel("Movement Controls")

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
        self.movement_widget.setLayout(movement_layout)
        
    @asyncSlot()
    async def on_connect(self):
        if self.button_connect.isChecked():
            port = self.port_edit.text().strip()
            
            if not port:
                data = pull_from_json()
                port = data.get(drone_instance_json, {}).get("port", "udpin://0.0.0.0:14540")
                self.port_edit.lineEdit().setText(port)
            
            self.log(f"Trying port: {port}")
            
            if not is_valid_port(port):
                self.log("-- Invalid port format")
                self.button_connect.setChecked(False)
                return
            
            self.port_edit.save_history()
            
            self.drone = Drone(port)
            self.log("Connecting...")
            
            try:
                connected = await self.drone.connect()
                if connected:
                    self.status.setText("Status: Connected")
                    self.log("-- Connected!")
                    self.button_takeoff.setEnabled(True)
                    self.start_battery_monitoring()

                else:
                    self.status.setText("Status: Failed")
                    self.button_connect.setChecked(False)
                    self.battery_percentage.setText("(Battery: --%)")
                    self.battery_voltage.setText("(Battery Voltage: --V)")
                    self.battery_consumed_action.setText("Battery Consumed: -- mAh")
                    self.battery_temp_action.setText("Battery Temperature: --℃")
                    self.battery_current_action.setText("Battery Current: --A")
                    self.battery_time_action.setText("Battery Time Remaining: --s")
            except Exception as e:
                self.log(f"Connect error: {e}")
                self.button_connect.setChecked(False)
        else:
            self.status.setText("Status: Disconnected")
            self.button_takeoff.setEnabled(False)
            self.battery_percentage.setText("(Battery: --%)")
            self.battery_voltage.setText("(Battery Voltage: --V)")
            self.battery_consumed_action.setText("Battery Consumed: -- mAh")
            self.battery_temp_action.setText("Battery Temperature: --℃")
            self.battery_current_action.setText("Battery Current: --A")
            self.battery_time_action.setText("Battery Time Remaining: --s")
            self.stop_battery_monitoring()
            self.closeEvent()

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
            self.button_land.setChecked(False)
            self.button_land.setEnabled(False)
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

        except Exception as e:
            self.log(f"Moving Backward Error: {e}")

    @asyncSlot()
    async def stoping_movement(self):
        self.log("Stoping Movement...")
        try:
            await self.drone.stop_movement()
            self.log("Stoping Movement...")

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