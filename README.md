# Game of Drones: A NeurIPS 2019 Competition


## Quickstart
- [Website](https://microsoft.github.io/AirSim-NeurIPS2019-Drone-Racing/)
- [Register](https://www.microsoft.com/en-us/research/academic-program/game-of-drones-competition-at-neurips-2019/)
- [Competition guidelines](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/blob/master/docs/competition_guidelines.md)
- [Linux and Windows Binaries](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases)
- [Python API](https://microsoft.github.io/AirSim-NeurIPS2019-Drone-Racing/api.html), [airsimneurips PyPI package](https://pypi.org/project/airsimneurips/)

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
	If you are having texture problems with the gates or seeing black shadows on the ground, please try running the binary with the openGL option : `./AirSimExe.sh -windowed -opengl4`. 

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
  	- [airsimneurips API doc](https://microsoft.github.io/AirSim-NeurIPS2019-Drone-Racing/api.html)
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
    
## Submitting Results and Leaderboard
- We have opened a validation submission pipeline so that participants can see how they are performing w.r.t. others, and for us to get feedback on how easy or hard the race tracks are currently.
- Please note that this leaderboard is not for the qualification rounds. 
This is effectively a dry run for you to gauge your performance and for us to gather feedback. 
The qualification binaries with new race tracks will be released next month

- How to generate logfiles for each tier:
	- Calling simStartRace with the desired tier level generates the appropriate log files.    
	As soon as simStartRace is called, `drone_2` (MSR opponent racer) will start flying.    
	Note that `simGetObjectPose()` will return noisy gate poses if tier=2 of tier=3 is passed to `simStartRace`   
	See `baseline_racer.py` for sample code. Note that if the `--race_tier` argument to `baseline_racer.py` is 2 or 3, `drone_2` will follow the noisy waypoints.  
	```
	client.simStartRace(tier=1/2/3)
	```

- To submit your results to the leaderboard:
	- Navigate to the [submission site](https://microsoft.github.io/AirSim-NeurIPS2019-Drone-Racing/upload.html), enter your team name in the proper field, and upload any number of [race logs](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/blob/master/docs/competition_guidelines.md#race-monitoring).   
It's ok to make a submission for as little as a single track and/or a single tier.   
You can find race logs inside of `AirSimExe/Saved/Logs/RaceLogs` in your downloaded binary folder.   
Please read [the race monitoring section](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/blob/master/docs/competition_guidelines.md#race-monitoring) in the competition guidelines for more details. 
	- The leaderboard will publish the results of a drone that is named `drone_1` (call [`generate_settings_file.py`](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/blob/master/baselines/generate_settings_file.py) to generate an AirSim settings file, as done for the `baseline_racer` below. 
	- At this point in time, the `report` and `Team ID` fields are optional, so you can leave them blank.   
	You'd be require to submit a report when the qualification round opens, in order to validate the legitimacy of the submissions.    
	We'll email Team IDs to each team, after which the `Team ID` will be required.   
 	For now, feel free to leave that field blank.   
	- The [validation leaderboard](https://microsoft.github.io/AirSim-NeurIPS2019-Drone-Racing/leaderboard.html) is updated once per day at 2100 PST.   
	If you do not see your results after 24 hours, please [email us](mailto:neuripsdronecontestinfo@gmail.com) with your team name and submitted log files.
	- This round of submissions is for **validation** and **performance comparison** with other teams only.  
 	Submissions at this time will not be used to determine which teams qualify for the live event.
	
## Baselines
 - Plan and move on minimum jerk trajectory using gate ground truth poses:
    - Generate an AirSim settings.json file
	 ```shell
	$ cd baselines;
	$ python generate_settings_file.py
	```
    - Start the AirSim Neurips binary, [as explained above](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing#running-the-executable)
    - Run the code!  
      See all the [baseline arguments here](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/blob/master/baselines/baseline_racer.py#L260-#L265) 
	```shell
	$ python baseline_racer.py \
		--enable_viz_traj \
		--enable_viz_image_cv2 \
		--planning_baseline_type all_gates_at_once \
		--planning_and_control_api moveOnSpline \
		--level_name ZhangJiaJie_Medium 
		--race_tier 3 \
	```

## List of Environments in Increasing Order of Difficulty 
- Soccer Field: A simple outdoors environment with few obstacles, and an easy to follow course.
- ZhangJiaJie: A mountainous landscape based on a national park in the Hunan province of China.
- Building99: A tight race course designed inside one of Microsoft's very own buildings.

## Questions
Please open a Github Issue on **this** repository (not [AirSim](https://github.com/microsoft/AirSim)) for any technical questions w.r.t. the Neurips competition. 
