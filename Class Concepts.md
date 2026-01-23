# Class Concepts

In order to properly structure our code for the GUI, we are going to need to organize everything into an easy to use API. The best way to do this would be through **classes**.

### What are classes?

In python, classes are a blueprint or template for creating objects. It defines a set of properties (attributes) and functions (methods) that all objects created from that class will share, promoting code reuse and organization.

### How are they important?

We cannot simply run all of our code in a single function everytime we press a button. That will cause some really big problems down the road, so it's better to avoid that.

The way that MAVSDK has everything structured, it's actually really simple to add everything into a class.

>**Fun Fact!** \
>The "System()" that we import from MAVSDK is a class in of itself. \
>We will be creating something similar to that.

# Basic Class Structure

Let's take a look at a psudeo **Waypoint** class to handle our file parsing.

## Waypoint Class

Let's look at a basic waypoint class structure.

```python
# First, let's define a class
class Waypoint:

    # We then need to create the "initial" function that handles variable defintion for this class instance.
    # Think of class instances as different sheets of paper. They all have the same initial print, but you can add
    # to it without altering the inital ink print.
    def __init__(self, file):
        # A list containing each waypoint. We currently use tuples, but it might be better to use dictionaries instead for key-value pairs.
        self.waypoints = []

        # The current index of our waypoint.
        self.current_index = 0

        # Calling the load_coordinates function to give us all the neccessary information and save them to the waypoints list.
        self.load_coordinates(file)

    def load_coordinates(self, file):
        # This is our first function to be called no matter what.
        # Here, we will parse our csv file (or we could use a json for easier visualization) and save all the coordinate data to the waypoints list.
        # It should be structured to be able to load multiple files and append them to the same waypoints list.
        pass

    def add_waypoints(self, lat, lon, alt, mode)
        # We want to be able to update our mission on the fly, so we need some functions that can add waypoints.
        # Something like this would be needed to append waypoints to the end of our waypoints list.
        pass

    def get_current_waypoint(self):
        # This will grab the current waypoint's information.
        # This is especially useful for drone movement if not required.
        pass

    def get_waypoint(self, index):
        # This function will allows us to get the waypoint information for any waypoint in our list.
        # While not immediately useful, it can definitely help in some specific scenarioes.
        # It may benefit to create optional waypoint names for different sites.
        pass

    def set_waypoint(self, index, lat, lon, alt, mode)
        # We already have a function to append waypoints at the end of our list. However, we might need to insert them somewhere.
        # A function like this will suffice.
        pass

    def advance_next_waypoint(self):
        # Instead of automatically advancing to the next waypoint and having everything time based, it'd be much easier and practical to have something to advance instead.
        # This is a manual way of moving to the next waypoint
        pass

    def go_to_waypoint(self, index):
        # A function like this is handy when we want to choose a specific waypoint to go to, say we are at the 5th location, but we want to go to reset the timeline to the first.
        pass

    def reset_mission(self):
        # A function like this is nice when we want to completely restart the mission.
        # We could optionally call it when the mission is finished, but it'd be easier to handle if it's called manually.
        pass
```
>This is not actual production-ready code. \
>This is simply a structure that we could use. \
>Other methods and/or attributes may be needed, this is merely the basics.

## Drone Class

Another import thing to create is a class for our drone instance.
This will allow us to easily run multiple neccessary functions accross multiple scopes and scripts.
Using multiple scripts without a structure like this is impossible because you'd need a seperate drone instance every time.

Let's take a look at some simple structure.

```python
# First we must import the API
from mavsdk import System

# Create a drone class similar to how we did waypoints
class Drone:

    # Just like in our waypoints class, we will do our initial functions here. Creating our instance, connecting to the drone, and setting our default takeoff altitude.
    def __init__(self, default_takeoff_altitude = 5):
        # Initiating drone instance
        self.drone = System()
        # Connecting to the drone
        self.connect_to_drone()
        # An optional choice of changing our default takeoff altitude
        self.takeoff_altitude = default_takeoff_altitude

    def connect_to_drone(self):
        # Here will contain the main logic for connecting to the drone over UDP
        # We can use the health checks class I wrote earlier
        pass

    def current_position():
        # Grabbing the current position doesn't have to be done by calling a function every time. Instead, we can set class attributes for out instane such as
        # self.latitude,
        # self.longitude,
        # self.altitude.
        # That way we can directly reference these variables instead of unpacking a tuple from a function
        pass

    def move_to_location(self, lat, lon, alt, mode):
        # If we aren't using a waypoint that is in our mission, we can directly move to it from here.
        pass

    
    # We need to be able to control our drone, just in case something bad happens. 
    # Manual override is for that purpose, but it's nice to be able to simply move the drone with a few button presses without cancelling the mission.
    def move_left_offset(self, offset):
        pass

    def move_right_offset(self, offset):
        pass

    def move_down_offset(self, offset):
        pass

    def move_up_offset(self, offset):
        pass


    def evade(self):
        # We don't want to crash into anything during our flight. So here, we can write some logic to dodge incoming drones and obstacles.
        # MAVSDK contains a way to do this through their API
        pass

    def move_to_waypoint(waypoint):
        # This is what will allow us to move through the waypoints in our mission. We will most likely be referencing our waypoints class here.
        pass

    def begin_mission(self):
        # While probably not neccessary, we can make this function that starts automatically moving through each waypoint.
        pass

    def pause_mission(self):
        # Because we have an automated way to move through our mission, we also need a way to stop the automation.
        pass

    def resume_mission(self):
        # We also need a way to resume it.
        pass

    def return_to_home(self):
        # This can save us some time by simply always having our home position saved. This allows us to move back to it at any time.
        pass

    def fetch_drone_instance(self):
        # In case we need to do something with our drone that doesn't require a function in this class, or would just be easier without one, it's important to have a way to fetch our drone instance.
        return self.drone

```

>This is not actual production-ready code. \
>This is simply a structure that we could use. \
>Other methods and/or attributes may be needed, this is merely the basics.

# Implementation

The actual implementation itself I'm not too sure about. However, I do have a general idea of how we can go about it. I did use AI to create the comments for this one because I didn't have tiem to read the API these past few days.

```python
from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel, QVBoxLayout, QWidget, QTextEdit, QFileDialog, QMessageBox
from PyQt6.QtCore import QObject
 
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Initialize mission and drone-related instances for later access
        self.waypoint = Waypoint(file)
        self.drone = Drone()

        # Set up the main GUI window properties
        # Set the title and dimensions for the window
        self.setWindowTitle("Drone Mission Controller")
        self.setGeometry(100, 100, 800, 600)  # Define dimensions of the window
        
        # Create the layout and designate it as the central widget for the window
        # Setting up a VBox layout (vertical stacking of components)
        self.central_widget = QWidget()
        self.layout = QVBoxLayout()
        self.central_widget.setLayout(self.layout)
        self.setCentralWidget(self.central_widget)

        # Add GUI components for the mission file
        # A label to display the selected mission file path
        self.mission_file_label = QLabel("Mission File: Not loaded.")
        self.layout.addWidget(self.mission_file_label)

        # A button to trigger the file selection dialog for loading a mission file
        self.select_file_button = QPushButton("Select Mission File")
        self.select_file_button.clicked.connect(self.load_mission_file)  # Connect button click to handler
        self.layout.addWidget(self.select_file_button)

        # Add GUI components for displaying current waypoint information
        # A label to display the current waypoint data
        self.waypoint_data_label = QLabel("Current Waypoint: None")
        self.layout.addWidget(self.waypoint_data_label)

        # A button to fetch and display the current waypoint
        self.fetch_waypoint_button = QPushButton("Fetch Current Waypoint")
        self.fetch_waypoint_button.clicked.connect(self.fetch_current_waypoint)  # Connect button to handler
        self.layout.addWidget(self.fetch_waypoint_button)

        # Add buttons for controlling the mission lifecycle
        # A button to begin the mission (e.g., proceed through waypoints automatically)
        self.begin_mission_button = QPushButton("Begin Mission")
        self.begin_mission_button.clicked.connect(self.begin_mission)  # Placeholder for mission start logic
        self.layout.addWidget(self.begin_mission_button)

        # A button to manually advance to the next waypoint in the mission
        self.advance_waypoint_button = QPushButton("Advance to Next Waypoint")
        self.advance_waypoint_button.clicked.connect(self.advance_to_next_waypoint)  # Connect button to handler
        self.layout.addWidget(self.advance_waypoint_button)

        # A button to reset the mission back to the first waypoint
        self.reset_mission_button = QPushButton("Reset Mission")
        self.reset_mission_button.clicked.connect(self.reset_mission)  # Placeholder for mission reset logic
        self.layout.addWidget(self.reset_mission_button)

        # A button to trigger the drone's return-to-home functionality
        self.return_home_button = QPushButton("Return to Home")
        self.layout.addWidget(self.return_home_button)

        # Add a log output text area for display messages and status updates
        # Create a QTextEdit widget to act as a log console for status messages
        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)  # Make the log read-only
        self.layout.addWidget(self.log_output)

    def load_mission_file(self):
        # Open a file dialog to select a mission file (e.g., JSON or CSV format)
        # Use QFileDialog to allow the user to browse files
        # Update the mission_file_label and load the file into the Waypoint instance
        pass

    def fetch_current_waypoint(self):
        # Check if the Waypoint instance exists (i.e., a mission file has been loaded)
        # Retrieve the current waypoint from the Waypoint class
        # Update the waypoint_data_label to display the waypoint information
        # Log the current waypoint in the log_output area
        pass

    def begin_mission(self):
        # Check if both the Waypoint and Drone instances exist
        # Start the mission (e.g., iterate through waypoints and send commands to the drone)
        # Log the action in the log_output area
        pass

    def advance_to_next_waypoint(self):
        # Check if the Waypoint instance exists
        # Advance the Waypoint instance to the next mission point
        # Fetch and display the updated current waypoint information
        # Log the action in the log_output area
        pass

    def reset_mission(self):
        # Check if the Waypoint instance exists
        # Reset the mission to the first waypoint
        # Update the waypoint_data_label and log the action in the log_output area
        pass

if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())
```
