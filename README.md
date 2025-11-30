# TCHS SUAS

Welcome to the TCHS Aero's main drone team repository! This project contains the core code, resources, and documentation for our team’s participation in SUAS (Small Unmanned Aerial Systems) competitions.

## Table of contents:

| Categories    | What's shown?|
| ------------- |:-------------:|
| [Setup](#quickstart) | Setup/Installation instructions. |
| [How does SITL work?](#howdoesitwork) | Explains how the PX4 Software In The Loop Works |
| [Used Packages](#pkgs) | All packages used in this repo. |

<a name="quickstart">

## Setup

### Python Scripts

Setting up the individual scripts is simple.

1. Install git.
    <details>
    <summary>Arch Linux</summary>
    <pre>sudo pacman -Sy git</pre>
    </details>
    <details>
    <summary>Debain/Ubuntu Linux</summary>
    <pre>sudo apt install git</pre>
    </details>
    <details>
    <summary>MacOS</summary>
    1a. Make sure you have brew installed. If not, install it.
    <pre>/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"</pre>
    1b. Install git.
    <pre>brew install git</pre>
    </details>

2. Clone this repository.
    ```bash
    git clone https://github.com/TCHS-aero/fly/ && cd fly
    ```

3. Install mavsdk using pip.
    ```bash
    python -m pip install mavsdk
    ```
>When using an operating system such as Arch Linux, install your pip packages through a [virtual environment](https://docs.python.org/3/tutorial/venv.html). Virtual environments must be initiated to use their installed pip packages.

### Simulation

This repository uses PX4 and MAVSDK-python to connect to the drone. In order to use these scripts in a simulation, you must first install the neccessary software. There are multiple simulators we can choose from, but for this project we use JMavSim, a drone simulator built in Java.

1. Clone the PX4-Autopilot repository.
    ```bash
    git clone https://github.com/PX4/PX4-Autopilot; cd PX4-Autopilot
    ```

2. Execute the setup script for your operating system. This should automatically install all required dependencies.
    <details>
    <summary>Arch Linux</summary>
    <pre>bash Tools/setup/arch.sh</pre>
    </details>
    <details>
    <summary>Debain/Ubuntu Linux</summary>
    <pre>bash Tools/setup/ubuntu.sh</pre>
    </details>
    <details>
    <summary>MacOS</summary>
    <pre>bash Tools/setup/macos.sh</pre>
    </details>

3. Make PX4 SITL (Software In The Loop) and JMavSim
    ```bash
    make px4_sitl jmavsim
    ```
You may also want to use your simulation alongside an application such as QGroundControl. All you have to do is run it, and it will automatically connect.

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
