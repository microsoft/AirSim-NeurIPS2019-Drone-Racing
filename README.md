# Game of Drones: A NeurIPS 2019 Competition
- [Competition website](https://www.microsoft.com/en-us/research/academic-program/game-of-drones-competition-at-neurips-2019/)
- [Competition guidelines](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/blob/master/docs/competition_guidelines.md)
- [Linux Binaries](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/tag/v0.0-linux)
- [Windows Binaries](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/tag/v0.0-windows)

![Alt Text](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/blob/master/docs/images/neurips_b99_3_drones.gif?raw=true)

## Unreal Environments
This competition marks the advent of a new release process for AirSim, in which we have separated out the AirSim plugin from environment content.    
Instead of having [a series of individual executables](https://github.com/microsoft/AirSim/releases/tag/v.1.2.2) for each environment, we have compacted all of the relevant AirSim content and API into a single binary (`AirSimExe`).    
Unreal environments containing race courses are released as separate downloadable content (DLC) packages, in the form of `.pak` files, which can be loaded and unloaded into the main binary as needed.    

### Downloading AirSimExe and Unreal Environments 
- Navigate to [the releases page](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases).
- Download the relevant AirSim executable:
	- [Linux](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/download/v0.0-linux/AirSim-Linux.tar.gz)
	- [Windows](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/download/v0.0-windows/AirSim-Windows.zip)
- Download your desired environments (pakfiles) - `Building99.pak` / ` SoccerField.pak` / `ZhangJiaJie.pak`.
- Move the environment pakfile into `AirSim/AirSimExe/Content/Paks`. 
- The contents in the environment pakfile will now be linked to your AirSim binary!

### Running the Executable

- Linux
	- Open a terminal window, `cd` to `AirSim/` directory, and enter the following command:
		```
		./AirSimExe.sh -windowed
		```

- Windows
	- Navigate to the `AirSim/` directory, and double-click `run.bat`


## AirSim API
To control your drone and get information from the environment, you will need to use the AirSim client API, which is accessible via Python or C++.

### Python
- Pip package
	To install the Python API via pip:
		```
		pip install AirSim
		```

- Or else, install from source: 
	- Clone the [AirSim git repository](https://github.com/Microsoft/AirSim):
		```
		git clone https://github.com/Microsoft/AirSim.git
		```
	- Navigate to `AirSim/PythonClient`, open a command window, and enter the following command: 
		```
		python setup.py install --user
		```

### C++
- To use the C++ API, you'll have to build AirSim from source:
	- [Linux](https://microsoft.github.io/AirSim/docs/build_linux/)
	- [Windows](https://microsoft.github.io/AirSim/docs/build_windows/)
- The C++ API is [documented here](https://microsoft.github.io/AirSim/docs/apis_cpp/) 

## Changing Environments
There are two ways to swap between levels, either via AirSIm API or by the UI menu.
- API 
	```python
	client.simLoadLevel("{level_name}")
	```
- UI Menu
	- Press `F10` to toggle the level menu
	- Click your desired level. (Note: the UI lists all the pakfiles in the `AirSim/AirSimExe/Content/Paks` directory. Ensure you downloaded the pakfile, if you are not able to see a particular environment)

## List of Environments in Increasing Order of Difficulty 
- Soccer Field: A simple outdoors environment with few obstacles, and an easy to follow course.
- ZhangJiaJie: A mountainous landscape based on a national park in the Hunan province of China.
- Building99: A tight race course designed inside one of Microsoft's very own buildings.
