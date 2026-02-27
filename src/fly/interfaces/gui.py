import sys
import asyncio
import re
# gui 1
from fly.core.drone import Drone
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QLabel,QDoubleSpinBox,
    QVBoxLayout, QWidget, QTextEdit, QLineEdit, QTabWidget, QGridLayout,QSizePolicy, QFileDialog, QStatusBar
)
from PyQt6.QtGui import QFont, QFontDatabase, QIcon, QAction
from qasync import asyncSlot, QEventLoop
from fly.core.mission import Mission
from fly.core.drone import Drone

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
        self.central = QWidget()
        self.main_layout = QVBoxLayout()
        self.waypoint_counter = 0
        self.next_id = 0 
        self.waypoint_x_ID = []
        self.tabs = QTabWidget()


        self.font_id = QFontDatabase.addApplicationFont("src/fly/assets/BlackOpsOne-Regular.ttf")
        if self.font_id != -1: 
            font_families = QFontDatabase.applicationFontFamilies(self.font_id)
            custom_font_name = font_families[0]  
        else:
            print("Font failed to load")
            custom_font_name = "Arial" 

        
        
        self.port_edit = QLineEdit()
        self.port_edit.setPlaceholderText("Port, e.g. udpin://0.0.0.0:14540")

        self.logo = QLabel("TCHS Aero GUI v1")
        self.logo.setStyleSheet(f"font-size:40px; font-family: {custom_font_name};")
        self.status = QLabel("Status: Disconnected")
        
        
        self.battery = QLabel("Battery:-- %")
        self.battery_action = QAction()

        self.console = QTextEdit()
        self.console.setStyleSheet("background-color: black; color: white")
        self.console.setReadOnly(True)





        #general
        self.general_widget = QWidget()
        self.general_layout = QVBoxLayout()

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

        self.general_layout.addWidget(self.port_edit)
        self.general_layout.addWidget(self.button_connect)
        self.general_layout.addWidget(self.button_takeoff)
        self.general_layout.addWidget(self.button_land)
        self.general_widget.setLayout(self.general_layout)
        self.tabs.addTab(self.general_widget, "General")





        #movement
        self.movement_widget = QWidget()
        self.movement_layout = QVBoxLayout()
        self.movement_controls_text = QLabel("Movement Controls")
        
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
        self.button_stop_movement.clicked.connect(self.stopping_movement)
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

        self.grid = QGridLayout()
        self.grid.addWidget(self.button_up, 0, 4)
        self.grid.addWidget(self.button_down, 2, 4)
        self.grid.addWidget(self.button_forward, 0, 1)
        self.grid.addWidget(self.button_backward, 2, 1)
        self.grid.addWidget(self.button_left, 1, 0)
        self.grid.addWidget(self.button_right, 1, 2)
        self.grid.addWidget(self.button_stop_movement, 1, 1)
        self.grid.addWidget(self.button_rth, 1, 4)
        
        self.velocity_text = QLabel("Velocity")
        self.velocity = QDoubleSpinBox()
        self.yaw_text = QLabel("Yaw")
        self.yaw = QDoubleSpinBox()

        self.movement_layout.addWidget(self.movement_controls_text)
        self.movement_layout.addLayout(self.grid)

        self.movement_layout.addWidget(self.velocity_text)
        self.movement_layout.addWidget(self.velocity)
        self.movement_layout.addWidget(self.yaw_text)
        self.movement_layout.addWidget(self.yaw)
        self.movement_widget.setLayout(self.movement_layout)
        self.tabs.addTab(self.movement_widget, "Movement")






        #mission
        self.mission_widget = QWidget()
        self.mission_layout = QVBoxLayout()
        self.mission_controls_text = QLabel("Mission Controls")


        self.waypoint = QLineEdit()
        self.waypoint.setPlaceholderText("enter your coordinates! Format: Lat, Lon, Alt")
        self.waypoint.setStyleSheet("color: black")
        self.waypoint.returnPressed.connect(self.parse_text)

        self.error = QLabel("oi man you left an empty parameter bro! fill that in cro!")
        self.error.setStyleSheet("color: black")
        self.error.hide()

        self.upload_mission = QPushButton("upload preset mission\n (MUST BE .json FILE)")
        self.upload_mission.setStyleSheet("background-color: grey")
        self.upload_mission.clicked.connect(self.file_upload)

        self.start_mission = QPushButton("Start the mission")
        self.start_mission.setStyleSheet("background-color: red")
        self.start_mission.clicked.connect(self.start_the_mission)

        policy_waypoint = self.waypoint.sizePolicy()
        policy_waypoint.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.waypoint.setSizePolicy(policy_waypoint)
        self.waypoint.setMinimumHeight(60)

        policy_error = self.error.sizePolicy()
        policy_error.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.error.setSizePolicy(policy_error)
        self.error.setMinimumHeight(60)

        policy_up_mis = self.upload_mission.sizePolicy()
        policy_up_mis.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.upload_mission.setSizePolicy(policy_up_mis)
        self.upload_mission.setMinimumHeight(60)

        policy_up_start_mis = self.upload_mission.sizePolicy()
        policy_up_start_mis.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.start_mission.setSizePolicy(policy_up_start_mis)
        self.start_mission.setMinimumHeight(60)
        
        self.mission_layout.addWidget(self.waypoint)
        self.mission_layout.addWidget(self.error)
        self.mission_layout.addWidget(self.upload_mission)
        self.mission_layout.addWidget(self.start_mission)
        self.mission_widget.setLayout(self.mission_layout)
        self.tabs.addTab(self.mission_widget, "Mission")

    

#Overall UI

        self.main_layout.addWidget(self.logo)
        self.main_layout.addWidget(self.status)
        self.main_layout.addWidget(self.battery)
        self.main_layout.addWidget(self.console)
        self.main_layout.addWidget(self.tabs)
        self.central.setLayout(self.main_layout)
        self.setCentralWidget(self.central)

    def log(self, msg):
        self.console.append(msg)
        print(msg)


    
    async def battery_percentage_log(self, drone):
        
        try:
            percent = await self.drone.battery_percentage()
            return percent 
        except Exception as e:
            self.log(f"Battery error in percentage_log {e}")
            return None



    
                
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

                    if not hasattr(self, "_battery_task"):
                        self._battery_task = asyncio.create_task(self._battery_watcher(self.drone))

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


    async def _battery_watcher(self, drone):
        while True:
            if self.drone is None:
                await asyncio.sleep(1.0)
                continue

            percent = await self.battery_percentage_log(drone)
            if percent:
                self.battery.setText(f"Battery: {percent:.0f} %")
            else:
                self.battery.setText("Battery: error")
            await asyncio.sleep(1.0) 


    @asyncSlot()
    async def start_the_mission(self):
        try:
            print("teto")
            for waypoint in range(len(self.mission.waypoints)):
                print('diabeto')
                i = self.mission.get_waypoint(waypoint)
                await self.drone.move_to_waypoint(self.mission.advance_next_waypoint())
        except Exception as e:
            self.log(str(e))
            print(e)


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
    async def stopping_movement(self):
        self.log("Stopping Movement...")
        try:
            await self.drone.stop_movement()
            self.log("Stopping Movement...")
            self.status.setText("Status: Stopping Movement")

        except Exception as e:
            self.log(f"Stopping Movement Error: {e}")

    @asyncSlot()
    async def return_to_launch(self):
        self.log("Starting to Return to Launch...")
        try:
            await self.drone.return_to_home()
            self.log("Returning to Launch...")
            self.status.setText("Status: Return to Launch")

        except Exception as e:
            self.log(f"Return to Launch Error: {e}")



    def parse_file_upload(self, file):
        self.mission = Mission(file) 
        #for waypoint in self.mission.waypoints:
            #lat, lon, alt = self.mission.get_waypoint(waypoint['latitude']), self.mission.get_waypoint(waypoint['longitude']), self.mission.get_waypoint(waypoint['altitude'])
        try:
            for i, waypoint_row in enumerate(self.mission.waypoints, 1):  # i=1 to 5
        
                coords = f"{waypoint_row['latitude']},{waypoint_row['longitude']},{waypoint_row['altitude']}"
            
                self.waypoint_counter+=1
                self.waypoint_coord = QLabel(self)
                self.x_label = QPushButton(icon = QIcon("~arrow/fly/src/fly/config/x.jpg"),text="" ,parent=self)
                self.x_label.setCheckable(True)
            
                
                self.x_label.setGeometry(550, (self.waypoint_counter+1)*90, 50,50 )
                self.x_label.setStyleSheet("background-color: red")
                self.x_label.resize(50,50)
                self.x_label.show()
                
                self.waypoint_coord.setGeometry(200, ((self.waypoint_counter+1)*90), 350, 50)
                self.waypoint_coord.setStyleSheet("color: black; background-color: grey")
                self.waypoint_coord.setText(f"Waypoint {self.waypoint_counter}: {coords}")
                self.waypoint_coord.show()

                row_id = self.next_id
                self.x_label.row_id, self.waypoint_coord.row_id = row_id, row_id

                self.x_label.clicked.connect(self.x_clicked)

                self.waypoint_x_ID.append({"id": row_id,"label": self.waypoint_coord,"button": self.x_label})

                self.next_id +=1 
        except Exception as e:
            self.log(str(e))
            print(e)
    
    @asyncSlot()
    async def start_the_mission(self):
        try:
            print("teto")
            for waypoint in range(len(self.mission.waypoints)):
                print('diabeto')
                i = self.mission.get_waypoint(waypoint)
                print("dance")
                await self.drone.move_to_waypoint(self.mission.advance_next_waypoint())
                print('cookie')
        except Exception as e:
            self.log(str(e))
            print(e)
    def file_upload(self):

        dialog = QFileDialog()
        dialog.setNameFilter("*.json")
        dialog.setFileMode(QFileDialog.FileMode.ExistingFile)
        dialogSuccess = dialog.exec()

        if dialogSuccess == 1:
            selectedFiles = dialog.selectedFiles()
            file = selectedFiles[0]
            print(file)
            self.parse_file_upload(file)
            
        else:
            self.log(str('user has cancelled file selection'))
            print('user cancelled file selection')


    def parse_text(self):
        text = self.waypoint.text()
        list1= [c.strip() for c in text.split(",")]
        
        if (len(list1) != 3) or any(c=="" for c in list1):
            self.error.setText("oi man you didn't fill in every parameter!/use commas to seperate them!")
            self.error.show()
            QTimer.singleShot(2000, self.error.clear) 
            return
           
        try:
            lat = float(list1[0])
            lon = float(list1[1])
            alt = float(list1[2])
            print(lat, lon, alt)
            coords = f"{lat},{lon},{alt}"
            print(coords)
            self.waypoint_counter +=1
            self.waypoint_coord = QLabel(self)
            self.x_label = QPushButton(icon = QIcon("pyqt6/x_mark.jpg"),text="" ,parent=self)
            self.x_label.setCheckable(True)
            self.x_label.hide()
            
            self.x_label.setGeometry(550, (self.waypoint_counter+1)*90, 50,50 )
            self.x_label.setStyleSheet("background-color: red")
            self.x_label.resize(50,50)
            self.x_label.show()
            
            '''self.waypoint_labels.append(self.waypoint_coord)'''
            self.waypoint_coord.setGeometry(200, ((self.waypoint_counter+1)*90), 350, 50)
            self.waypoint_coord.setStyleSheet("color: black; background-color: grey")
            self.waypoint_coord.setText(f"Waypoint {self.waypoint_counter}: {coords}")
            self.waypoint_coord.show()

            row_id = self.next_id
            self.x_label.row_id, self.waypoint_coord.row_id = row_id, row_id

            self.x_label.clicked.connect(self.x_clicked)

            self.waypoint_x_ID.append({"id": row_id,"label": self.waypoint_coord,"button": self.x_label})

            self.next_id +=1 

        except:
            self.error.setText("oi man you can only have numbers! no letters!")
            self.error.show()
            QTimer.singleShot(2000, self.error.clear) 
            return


    def x_clicked(self):
        btn = self.sender()         
        row_id = btn.row_id   
        for row in self.waypoint_x_ID:
            if row["id"] == row_id:
                row["label"].hide()
                row["label"].setParent(None)

                row["button"].hide()
                row["button"].setParent(None)

                self.waypoint_x_ID.remove(row)
                break

        self.waypoint_counter -=1


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