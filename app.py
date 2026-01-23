import csv

from mavsdk import System
from PyQt6.QtWidgets import (
    QApplication,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from qasync import asyncSlot


class Mission:
    def __init__(self, file):
        self.file = file
        self.current_index = 0
        self.waypoints = []

        self.parse_file()

    def parse_file(self):
        with open(self.file) as csvfile:
            reader = csv.DictReader(csvfile)

            for row in reader:
                for key, value in row.items():
                    try:
                        if "." in value:
                            row[key] = float(value)
                        elif value.isdigit():
                            row[key] = int(value)
                    except Exception:
                        pass

                self.waypoints.append(row)

    def get_keys(self):
        return list(self.waypoints[-1].keys())

    def create_new_waypoint(self, lat, lon, alt):
        data = (lat, lon, alt)
        new_waypoint = {}
        keys = self.get_keys()
        for i in range(len(keys)):
            try:
                new_waypoint[keys[i]] = data[i]
            except Exception:
                print(
                    "-- No new data to assign to keys or too many data per column. Defaulting to None..."
                )
                new_waypoint[keys[i]] = None

        return new_waypoint

    def append_waypoint_to_mission(self, lat, lon, alt):
        self.waypoints.append(self.create_new_waypoint(lat, lon, alt))
        return self.waypoints

    def insert_waypoint_into_mission(self, index, lat, lon, alt):
        self.waypoints.insert(index, self.create_new_waypoint(lat, lon, alt))
        return self.waypoints

    def replace_waypoint_in_mission(self, index, lat, lon, alt):
        self.waypoints[index] = self.create_new_waypoint(lat, lon, alt)
        return self.waypoints

    def get_current_waypoint(self):
        return self.waypoints[self.current_index]

    def get_waypoint(self, index):
        try:
            return self.waypoints[index]
        except Exception as e:
            print(f"-- {str(e).capitalize()}.")
            return None

    def advance_next_waypoint(self):
        try:
            self.current_index += 1
            return self.get_current_waypoint()
        except Exception:
            print("-- No more waypoints to fly through.")
            return None


# GUI application with PyQt6
class DroneControlApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Control")
        self.setGeometry(100, 100, 400, 300)
        self.init_ui()
        self.drone = Drone()  # Instance of the drone class

    def init_ui(self):
        layout = QVBoxLayout()

        # Connect Button
        self.connect_button = QPushButton("Connect to Drone", self)
        self.connect_button.clicked.connect(
            self.connect_to_drone
        )  # Connect signal to slot
        layout.addWidget(self.connect_button)

        # Current Position Label
        self.position_label = QLabel("Current Position: Not Connected", self)
        layout.addWidget(self.position_label)

        # Get Position Button
        self.position_button = QPushButton("Get Current Position", self)
        self.position_button.clicked.connect(self.get_current_position)
        self.position_button.setEnabled(False)  # Disabled until connected
        layout.addWidget(self.position_button)

        # Move to Location
        self.move_label = QLabel("Enter Latitude, Longitude, Altitude:", self)
        layout.addWidget(self.move_label)

        self.lat_input = QLineEdit(self)
        self.lat_input.setPlaceholderText("Latitude")
        layout.addWidget(self.lat_input)

        self.lon_input = QLineEdit(self)
        self.lon_input.setPlaceholderText("Longitude")
        layout.addWidget(self.lon_input)

        self.alt_input = QLineEdit(self)
        self.alt_input.setPlaceholderText("Altitude")
        layout.addWidget(self.alt_input)

        self.move_button = QPushButton("Move to Location", self)
        self.move_button.clicked.connect(self.move_to_location)
        self.move_button.setEnabled(False)  # Disabled until connected
        layout.addWidget(self.move_button)

        # Return to Home
        self.rth_button = QPushButton("Return to Home", self)
        self.rth_button.clicked.connect(self.return_to_home)
        self.rth_button.setEnabled(False)  # Disabled until connected
        layout.addWidget(self.rth_button)

        # Set central widget
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    @asyncSlot()
    async def connect_to_drone(self):
        await self.drone.connect_to_drone()
        self.position_label.setText("Drone Connected!")
        self.connect_button.setEnabled(False)
        self.position_button.setEnabled(True)
        self.move_button.setEnabled(True)
        self.rth_button.setEnabled(True)

    @asyncSlot()
    async def get_current_position(self):
        await self.drone.current_position()
        self.position_label.setText(
            f"Current Position:\nLatitude: {self.drone.latitude}\nLongitude: {self.drone.longitude}\nAltitude: {self.drone.altitude}"
        )

    @asyncSlot()
    async def move_to_location(self):
        lat = self.lat_input.text()
        lon = self.lon_input.text()
        alt = self.alt_input.text()

        if lat and lon and alt:
            try:
                lat = float(lat)
                lon = float(lon)
                alt = float(alt)
                await self.drone.move_to_location(lat, lon, alt)
            except ValueError:
                self.position_label.setText(
                    "Invalid input! Please enter valid numbers for latitude, longitude, and altitude."
                )

    @asyncSlot()
    async def return_to_home(self):
        await self.drone.return_to_home()
        self.position_label.setText("Returning to Home!")


class Drone:
    def __init__(self, default_takeoff_altitude=5):
        self.drone = System()
        self.takeoff_altitude = default_takeoff_altitude
        self.latitude = None
        self.longitude = None
        self.altitude = None

    async def connect_to_drone(self):
        await self.drone.connect(system_address="udp://:14540")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("-- Drone connected!")
                break

    async def current_position(self):
        async for position in self.drone.telemetry.position():
            self.latitude = position.latitude_deg
            self.longitude = position.longitude_deg
            self.altitude = position.relative_altitude_m
            print(
                f"Current position - Latitude: {self.latitude}, Longitude: {self.longitude}, Altitude: {self.altitude}"
            )
            break

    async def return_to_home(self):
        print("-- Returning to home location...")
        await self.drone.action.return_to_launch()

async def main():
    drone = Drone()
    await drone.connect_to_drone()

if __name__ == "__main__":
    import asyncio
    asyncio.run(main())