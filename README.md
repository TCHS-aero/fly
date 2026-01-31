# TCHS SUAS

Welcome to the TCHS Aero's main drone team repository! This project contains the core code, resources, and documentation for our team’s participation in SUAS (Small Unmanned Aerial Systems) competitions.

## Table of contents:

| Categories    | What's shown?|
| ------------- |:-------------:|
| [Setup](#quickstart) | Setup/Installation instructions. |
| [How does SITL work?](#howdoesitwork) | Explains how the PX4 Software In The Loop Works. |
| [Usage](#usage) | How to use this repository. |
| [Used Packages](#pkgs) | All packages used in this repo. |

<a name="quickstart">

## Setup

### Python Scripts

Setting up the individual scripts is simple.

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

3. Install all dependencies.
    ```bash
    python -m pip install -r requirements.txt
    ```

> [!CAUTION]
> When using an operating system such as Arch Linux, install your pip packages through a [virtual environment](https://docs.python.org/3/tutorial/venv.html) to avoid system-wide python conflicts. While rare, it does happen because Arch uses python for some of it's core scripts. Virtual environments must be initiated to use their installed pip packages.

### Simulation

This repository uses PX4 and MAVSDK-python to connect to the drone. In order to use these scripts in a simulation, you must first install the neccessary software. There are multiple simulators we can choose from, but for this project we use JMavSim, a drone simulator built in Java. JMabSim comes with plenty of keyboard bindings for altering things such as the weather conditions, as documented [here](https://github.com/PX4/jMAVSim).

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
      
      1. This script installs applications through apt, meaning you need **sudo** privilages.

            ```bash
            bash Tools/setup/ubuntu.sh
            ```

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

4. Make PX4 SITL (Software In The Loop) and JMavSim
   
    ```bash
    make px4_sitl jmavsim
    ```
You may also want to use your simulation alongside an application such as QGroundControl. All you have to do is run it, and it will automatically connect.

<a name="usage">

### Usage

As of now, we only have a limited CLI to interface with the drone with. This is scheduled to change soon.
The CLI is compatible with the simulation by default, but some additional setup is requried for real flights.

**Simulation**

Simply running SITL will forward two ports, udp://0.0.0.0:14540 and udp://0.0.0.0:14550. All you need to do is connect to one of these ports.

```bash
# The "--port" flag is an optional setting, and is mainly used for actual drones. Ignoring it defaults the connectiong to 14540.
python cli_app.py connect --port udpin://0.0.0.0:14540
```

> [!NOTE]
> Port 14550 can be used to talk to the drone, however it is recommended to use 14540 instead, as 14550 is the default port that QGC listens to.

**Actual Flight**

On the actual flight, it is not as simpple as connecting to 14540. You must find the port specifically connected to your PixHawk configuration on your flight device.
This varies depending on what telemtry radio you are using.

<details open=true>
<summary>Unix Filesystems</summary>

You can find plugged USB's with the command `lsusb -t`.
    
```bash
# Replace "ttyUSB0" with whatever USB port your telemetry radio is plugged in.
# If you have a differently configured BAUD rate for your pixhawk, change "921600" to said rate.
python cli_app.py connect --port serial:///dev/ttyUSB0:921600
```

</details>

<details open=true>
<summary>Windows</summary>
    
```bash
# Replace "COM3" with whatever USB port your telemetry radio is plugged in. You can find your COM port via Device Manager.
# If you have a differently configured BAUD rate for your pixhawk, change "921600" to said rate.
python cli_app.py connect --port serial://COM3:921600
```

</details>

> [!NOTE]
> Typically you'd use 921600 BAUD for offboard control, and 57600 BAUD for Ground Control.

<a name="howdoesitwork">

### How does SITL work?

PX4 SITL is a way to run the PX4 flight-control software on your computer instead of on a real drone. PX4 runs as a normal program, and the simulator (like Gazebo or jMAVSim) gives physics and sensor readings such as GPS and IMU data. PX4 interprets this, computes any processes or calculations that need to be done, then sends it back to the simulation for the next update. SITL and the simulator talk using MAVLink over UDP, which also allows tools like QGroundControl to connect just like they would to a real drone. This makes it safe and easy to test flight behavior, missions, and code changes without needing any physical hardware or risking a crash, which is definitely something that none of us want to handle ourselves.

<a name="pkgs">

## Packages
* [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
* [JMavSim](https://github.com/PX4/jMAVSim)
* [Gazebo](https://gazebosim.org/home)
* [Python](https://www.python.org/)
* [MAVSDK-python](https://github.com/mavlink/MAVSDK-Python)
