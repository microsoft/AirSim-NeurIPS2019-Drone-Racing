# Game of Drones Test Environment v0
Say hello to the first release of test environments for the NeurIPS 2019 competition, [Game of Drones](https://www.microsoft.com/en-us/research/academic-program/game-of-drones-competition-at-neurips-2019/).
 Here you will find 3 different worlds with prebuilt tracks to test and refine the autonomous functionality of your drones.
## Environments
This competition marks the start of some new functionality in airsim, which we think you will enjoy. Instead of having a series of individual executables for each environment, we have compacted all of the relevant airsim content and functionality into a single binary, and created DLC packages which can loaded and unloaded into the main binary at will. Adding DLC content to your airsim executable is simple:
1. Navigate to [the releases page](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases)
2. Download the AirSim executable relevant to your operating system ([linux](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/download/v0.0-linux/Airsim-Linux.tar.gz)/[windows](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/download/v0.0-windows/AirSim-Windows.zip))
3. Download your desired DLC package(s)
4. Copy your downloaded `pak` file, and paste it into `Airsim-{your distribution}/AirSimExe/Content/Paks`
5. The contents in that level pak will now be linked to your airsim binary!

## Airsim
To control your drone and get information from the environment, you will need to use the airsim client API, which is accessible via Python or C++.
### Python installation
To install the Python API via pip, simply open a command window (if on a Windows OS), or a terminal window (if on a Unix OS), and run the following command:
```
pip install airsim
```
To download and from source: 
1. Clone the [airsim git repository](https://github.com/Microsoft/AirSim.git)
2. Navigate to `Airsim/PythonClient`, open a command window, and enter the following command: ```pip install airsim```

### C++ installation
To install the client API c++ library, follow the instructions listed under [How to Get It](https://github.com/Microsoft/AirSim#how-to-get-it)

## Changing Levels
There are two ways to swap between levels, either via API or by the UI menu.
### API usage
``` client.simLoadLevel("{level_name}")```
### UI menu
1. press `F10` to toggle the level menu
2. Click your desired level

## List of Levels by Difficulty (Least to Most Difficult)
1. Soccer Field - A simple outdoors environment with few obstacles, and an easy to follow course.
2. ZhangJiaJie - A mountainous landscape based on a national park in the Hunan province of China.
3. Building99 - A tight race course designed inside one of Microsoft's very own buildings.

