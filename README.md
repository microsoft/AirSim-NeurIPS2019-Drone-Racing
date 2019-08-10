# Game of Drones: A NeurIPS 2019 Competition

## Quickstart
- [Competition website](https://www.microsoft.com/en-us/research/academic-program/game-of-drones-competition-at-neurips-2019/)
- [Competition guidelines](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/blob/master/docs/competition_guidelines.md)
- [Linux and Windows Binaries](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases)
- Python API
   - [airsimneurips @PyPI](https://pypi.org/project/airsimneurips/)
   - [airsimneurips API doc](https://microsoft.github.io/AirSim-NeurIPS2019-Drone-Racing/)


<img src="https://github.com/madratman/airsim_neurips_gifs/blob/master/imgs/neurips_b99_3_drones.gif?raw=true" width="285"> <img src="https://github.com/madratman/airsim_neurips_gifs/blob/master/imgs/neurips_soccer_field_8_drones.gif?raw=true" width="285"> <img src="https://github.com/madratman/airsim_neurips_gifs/blob/master/imgs/neurips_zhangjiajie_4_drones.gif?raw=true" width="285">

## Unreal Environments
This competition marks the advent of a new release process for AirSim, in which we have separated out the AirSim plugin from environment content. Instead of having a series of individual executables for each environment, we have compacted all of the relevant AirSim content and API into a single binary (`AirSimExe`).    
Unreal environments containing race courses are released as separate downloadable content (DLC) packages, in the form of `.pak` files, which can be loaded and unloaded into the main binary as needed.    

### Downloading AirSimExe and Unreal Environments 
- Navigate to [the releases page](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases).
- Download the latest Windows or Linux `AirSim.zip`. 
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
To control your drone and get information from the environment, you will need the `airsimneurips` API, which is accessible via Python. 

### Installation
- To install the Python API for the Neurips competition, please use:
	```
	pip install airsimneurips
	```
	Corollary: Do not do a `pip install airsim`, as we will have a few custom APIs specific to this competition. 
- Resources 
  	- [airsimneurips API doc](https://microsoft.github.io/AirSim-NeurIPS2019-Drone-Racing/)
	- [AirSim upstream API](https://microsoft.github.io/AirSim/docs/apis/) and [examples](https://github.com/microsoft/AirSim/tree/master/PythonClient)    
	(Note that this is not used in the competition, however is a good learning resource)

## Changing Environments
There are two ways to swap between levels, either via AirSIm API or by the UI menu.
- API 
	- We have added a new API `simLoadLevel(level_name="MainMenu")` to change Unreal environments on the fly.   
	 Possible values for `level_name` are : `"Soccer_Field_Easy"`, `"Soccer_Field_Medium"`, `"ZhangJiaJie_Medium"`, `"Building99_Hard"`. 
	Here's a quick snippet to iterate throught them all. Before trying this, please ensure you've downloaded the 	corresponding pak files from [our releases page](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases). 

	```python
	import airsimneurips as airsim
	client = airsim.MultirotorClient()
	client.confirmConnection()
	client.simLoadLevel('Soccer_Field_Easy')	
	client.simLoadLevel('Soccer_Field_Medium')	
	client.simLoadLevel('ZhangJiaJie_Medium')
	client.simLoadLevel('Building99_Hard')
	```
- UI Menu
	- Press `F10` to toggle the level menu
	- Click your desired level. (Note: the UI lists all the pakfiles in the `AirSim/AirSimExe/Content/Paks` directory. Ensure you downloaded the pakfile, if you are not able to see a particular environment)

## Baselines
 - Plan and move on minimum jerk trajectory using gate ground truth poses:
    - Generate an AirSim settings.json file
	 ```shell
	$ cd baselines;
	$ python generate_settings_file.py
	```
    - Start the AirSim Neurips binary, [as explained above](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing#running-the-executable)
    - Run the code!  
      See all the [baseline arguments here](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/blob/master/baselines/baseline_racer.py#L184-#L188) 
	```shell
	$ python baseline_racer.py \
		--viz_traj \
		--plot_transform \
		--planning_baseline_type all_gates_at_once \
		--planning_and_control_api moveOnSpline \
		--level_name ZhangJiaJie_Medium 
	```

	
## List of Environments in Increasing Order of Difficulty 
- Soccer Field: A simple outdoors environment with few obstacles, and an easy to follow course.
- ZhangJiaJie: A mountainous landscape based on a national park in the Hunan province of China.
- Building99: A tight race course designed inside one of Microsoft's very own buildings.

## Questions
Please open a Github Issue on **this** repository (not [AirSim](https://github.com/microsoft/AirSim)) for any technical questions w.r.t. the Neurips competition. 
