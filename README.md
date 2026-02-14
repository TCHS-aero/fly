# TCHS SUAS

Welcome to the TCHS Aero's main drone team repository! This project contains the core code, resources, and documentation for our team’s participation in SUAS (Small Unmanned Aerial Systems) competitions.

## Table of contents:

| Categories    | What's shown?|
| ------------- |:-------------:|
| [Setup](#quickstart) | Setup/Installation instructions. |
| [Usage](#usage) | How to use this repository. |
| [Simulation](#sim) | Guide to using the Simulation. |
| [How does SITL work?](#howdoesitwork) | Explains how the PX4 Software In The Loop Works. |
| [Used Packages](#pkgs) | All packages used in this repo. |

<a name="quickstart">

### Setup

This repository is a python module; thus, setting it up is simple.

1. Install git.
    
    <details>
      <summary>Arch Linux</summary>
        
      1. Install using pacman. Sudo is required.

            ```bash
            sudo pacman -Sy git
            ```

    </details>

    <details>
      <summary>Debian/Ubuntu</summary>
        
      1. Install using the apt package manager. Sudo is required.

            ```bash
            sudo apt install git
            ```

    </details>

    <details>
      <summary>MacOS</summary>
        
      1. Make sure you have brew installed. If not, install it.

            ```bash
            /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
            ```
    
      2. Install git.

            ```bash
            brew install git
            ```

    </details>


2. Clone this repository.
    ```bash
    git clone https://github.com/TCHS-aero/fly/ fly; cd fly
    ```
    

To use this script, it is recommended for development to install this repository as an editable python module.

```bash
pip install -e .
```

Uninstalling it as simple as removing the `fly` module.

```bash
pip uninstall fly
```

Now, you can run any script/interface by running it's binary. It should automatically be aliased.

```bash
# Here is an example of running the CLI.
aero_cli
```

```bash
# Here is how to run our GUI
aero_gui
```

Alternatively, you can run it using the module path.

```bash
python -m fly.interfaces.cli #for running the cli
python -m fly.interfaces.gui #to run the gui
```

For the sake of simplicity, the rest of the examples will be using the former method.

> [!CAUTION]
> When using an operating system such as Arch Linux, install your pip packages through a [virtual environment](https://docs.python.org/3/tutorial/venv.html) to avoid system-wide python conflicts. While rare, it does happen because Arch uses python for some of it's core scripts. Virtual environments must be initiated to use their installed pip packages.

<a name="usage">

### Usage


The CLI and GUI is compatible with the simulation by default with very little configuration, but some additional setup is requried for real flights.
Simply running the [drone simulation](#sim) will forward two ports, udp://0.0.0.0:14540 and udp://0.0.0.0:14550. All you need to do is connect to one of these ports.

```bash
# The "--port" flag is an optional setting, and is mainly used for actual drones. Ignoring it defaults the connection to 14540.
aero_cli connect --port udpin://0.0.0.0:14540
```

> [!NOTE]
> Port 14550 can be used to talk to the drone, however it is recommended to use 14540 instead, as 14550 is the default port that QGC listens to. You cannot have two connections bound the same port at a time.

On the actual flight, it is not as simple as connecting to 14540. You must find the port specifically connected to your PixHawk configuration on your flight device.
This varies depending on what telemetry radio you are using.

<details open=true>
<summary>Unix Filesystems</summary>

You can find plugged USB's with the command `lsusb -t`.
    
```bash
# Replace "ttyUSB0" with whatever USB port your telemetry radio is plugged in.
# If you have a differently configured BAUD rate for your pixhawk, change "57600" to said rate.
aero_cli connect --port serial:///dev/ttyUSB0:57600
```

</details>

<details open=true>
<summary>Windows</summary>
    
```bash
# Replace "COM3" with whatever USB port your telemetry radio is plugged in. You can find your COM port via Device Manager.
# If you have a differently configured BAUD rate for your pixhawk, change "57600" to said rate.
aero_cli connect --port serial://COM3:57600 
```

</details>

> [!NOTE]
> Typically you'd use 57600 BAUD for offboard control, and 921600 BAUD for Ground Control or helper apps such as QGroundControl

<a name="sim">

### Simulation

This repository uses PX4 and MAVSDK-python to connect to the drone. In order to use these scripts in a simulation, you must first install the neccessary software. There are multiple simulators we can choose from, but for this project we use JMavSim, a drone simulator built in Java. JMavSim comes with plenty of keyboard bindings for altering things such as the weather conditions, as documented [here](https://github.com/PX4/jMAVSim).

1. Clone the PX4-Autopilot repository.
    ```bash
    git clone https://github.com/PX4/PX4-Autopilot; cd PX4-Autopilot
    ```

2. Execute the setup script for your operating system. This should automatically install all required dependencies.
   
    <details>
      <summary>Arch Linux</summary>

      1. This script installs applications through pacman, meaning you need **sudo** privilages.

            ```bash
            bash Tools/setup/arch.sh
            ```

    </details>
    
    <details>
      <summary>Debain/Ubuntu Linux</summary>
      
      1. Install dependencies.
            ```bash
            sudo apt-get install ant
            ```
      3. This script installs applications through apt, meaning you need **sudo** privilages.

            ```bash
            bash Tools/setup/ubuntu.sh
            ```
    > The Ubuntu setup script sometimes skips over the ant install silently, so make sure you have both ant and atleast openjdk-17-jdk installed.


    </details>
    
    <details>
      <summary>MacOS</summary>
      
      1. The `--sim-tools` flag must be specified when using the MacOS setup script.

            ```bash
            bash Tools/setup/macos.sh --sim-tools
            ```

    </details>

> [!IMPORTANT]
> This script also installs python packages through pip. It will use the system-wide version of python if you do not have a virtual environment enabled. If you have a virtual environment running in the same shell as you run this script, it will use the virtual environment instead. This may be preferred for some arch users.

3. Restart your computer.

4. Make PX4 SITL (Software In The Loop) and JMavSim
   
    ```bash
    make px4_sitl jmavsim
    ```
You may also want to use your simulation alongside an application such as QGroundControl. All you have to do is run it, and it will automatically connect.

> [!IMPORTANT]
> If you previously ran the make command before installing dependencies, it may have caused a build error, resulting in px4_sitl or JMavSim not being found. Make sure to run ```make distclean``` to revert the build before making again.

> [!CRITICAL]
> The simulation is not required to use any other scripts in this repository; it is simple a replacement for testing if you do not have access to a real drone.

<a name="howdoesitwork">

### How does SITL work?

PX4 SITL is a way to run the PX4 flight-control firmware on your computer instead of on a real drone. PX4 runs as a normal program, and the simulator (like Gazebo or jMAVSim) gives physics and sensor readings such as GPS and IMU data. PX4 interprets this, computes any processes or calculations that need to be done, then sends it back to the simulation for the next update. SITL and the simulator talk using MAVLink over UDP, which also allows tools like QGroundControl to connect just like they would to a real drone. This makes it safe and easy to test flight behavior, missions, and code changes without needing any physical hardware or risking a crash.

<a name="pkgs">

## Packages
* [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
* [QGroundControl](https://github.com/mavlink/qgroundcontrol)
* [JMavSim](https://github.com/PX4/jMAVSim)
* [Gazebo](https://gazebosim.org/home)
* [Python](https://www.python.org/)
* [MAVSDK-python](https://github.com/mavlink/MAVSDK-Python)
* [PyQt6](https://pypi.org/project/PyQt6/)
* [qasync](https://github.com/CabbageDevelopment/qasync)
* [asyncclick](https://pypi.org/project/asyncclick/)