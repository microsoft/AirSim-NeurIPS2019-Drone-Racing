# Game of Drones: A NeurIPS 2019 Competition


## Quickstart
- [Website](https://microsoft.github.io/AirSim-NeurIPS2019-Drone-Racing/)
- [Register](https://www.microsoft.com/en-us/research/academic-program/game-of-drones-competition-at-neurips-2019/)
- [Competition guidelines](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/blob/master/docs/competition_guidelines.md)
- [Linux and Windows Binaries](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases)
- [Python API](https://microsoft.github.io/AirSim-NeurIPS2019-Drone-Racing/api.html), [airsimneurips PyPI package](https://pypi.org/project/airsimneurips/)

<img src="https://github.com/madratman/airsim_neurips_gifs/blob/master/imgs/neurips_b99_3_drones.gif?raw=true" width="285"> <img src="https://github.com/madratman/airsim_neurips_gifs/blob/master/imgs/neurips_soccer_field_8_drones.gif?raw=true" width="285"> <img src="https://github.com/madratman/airsim_neurips_gifs/blob/master/imgs/neurips_zhangjiajie_4_drones.gif?raw=true" width="285">

### Downloading and running AirSim Binaries
#### Downloading
- Qualifier binaries and environments (v1.0)
	- tl;dr:
		- [Linux] Use the [download_qualification_binaries.sh](download_qualification_binaries.sh) script
	- Long version:
		- Download the v1.0 [Linux](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/tag/v1.0-linux) or [Windows](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/tag/v1.0-windows) `AirSim.zip`, and unzip it. 
		- Download your qualifier environments (shipped in pakfiles) - `Qual_Tier_1_and_Tier_3.pak`  and ` Qual_Tier_2.pak`.
		- Move the environment pakfiles into `AirSim/AirSimExe/Content/Paks`. 
		- Download and move the `settings.json` file to `~/Documents/AirSim/setting.json`. 

- Training binaries and environments (v0.3):
	- tl;dr: 
		- [Linux] Use the [download_training_binaries.sh](download_training_binaries.sh) script
	- Long version:
		- Download the v0.3 [Linux](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/tag/v0.3.0-linux) or [Windows](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/tag/v0.3.0) `AirSim.zip`, and unzip it. 
		- Download training  environments (shipped in pakfiles) - `Soccer_Field.pak`, `ZhangJiaJie.pak`, and `Building99.pak`. 
		- Move the environment pakfiles into `AirSim/AirSimExe/Content/Paks`. 
		- Download and move the `settings.json` file to `~/Documents/AirSim/setting.json`. 

Notes:
-  `Source code (zip)` or `Source code (tar.gz)` might not be up-to-date with the master branch of this repository. It can be lagging by `n commits to master since this release`, specified on the released page.   
	For the code on this repository, it's best to just `git clone`.  

- List of disabled APIs in qualification binaries: The following APIs on the server side in the qualification binaries. You should see an error message pop up in the terminal message when you call these. They do work in the training binaries:
	- `simSetObjectPose`
	- `simSetVehiclePose`
	- `simSetObjectScale`
	- `simGetObjectScale`
	- `simSetSegmentationObjectID`
	- `simGetSegmentationObjectID`
	- `simPause`
	- `simContinueForTime`

#### Running
- Linux
	- Open a terminal window, `cd` to `AirSim_Training/` or `AirSim_Qualification` directory, and enter the following command:
		```
		./AirSimExe.sh -windowed -opengl4
		```

- Windows
	- Navigate to the `AirSim/` directory, and double-click `run.bat` (or `AirSimExe.exe -windowed`)

## AirSim API
- To control your drone and get information from the environment, you will need the `airsimneurips` API, which is accessible via Python.   
We recommend you used python >= 3.6. Python 2.7 will go [out of support soon](https://pythonclock.org/)

- To install the Python API, do a :
	```
	pip install airsimneurips
	```

- The API is documented at [airsimneurips API doc](https://microsoft.github.io/AirSim-NeurIPS2019-Drone-Racing/api.html)

- Resources 
	- Going through both open and closed issues in this repository might answer some of your questions. The search bar on top left can prove useful.    
	- [AirSim upstream API](https://microsoft.github.io/AirSim/docs/apis/) and [examples](https://github.com/microsoft/AirSim/tree/master/PythonClient) can also be of use. However, please note that the main AirSim repo's API is not used in the competition (there's some overlap and some differences), however is a good learning resource. 
    
## Submitting Results and Leaderboard
- For the qualification round, we have one race track for each tier. The relevant binaries (v1.0) are available for [linux](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/tag/v1.0-linux) and [windows](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/tag/v1.0-windows)
	- Tier 1: This is in the Soccer Field environment.    
	THe race track is in the `Qual_Tier_1_and_Tier_3.pak` pakfile
	- Tier 2: This is in the ZhangJiaJie environment.    
	The race track is in the `Qual_Tier_2.pak` pakfile. 
	- Tier 3: This is again in the Soccer Field environment.    
	The race track is in the `Qual_Tier_1_and_Tier_3.pak` pakfile. 

- How to generate logfiles for each tier:
	- Loading level and starting race:
		- Please update your airsimneurips pythonclient (should be >=1.0.0). 
		- Calling `simStartRace(race_tier=1, 2, or 3)` generates the appropriate log files. 
		- Tier 1: 
			```python
				airsim_client.simLoadLevel('Qualifier_Tier_1')
				airsim_client.simStartRace(1)
			```

		- Tier 2: 
			```python
				airsim_client.simLoadLevel('Qualifier_Tier_2')
				airsim_client.simStartRace(2)
			```

		- Tier 3: 
			```python
				airsim_client.simLoadLevel('Qualifier_Tier_3')
				airsim_client.simStartRace(3)
				```
	- As Tier 2 focuses on perception and Tier 3 focuses on both perception and planning, note that `simGetObjectPose` returns noisy gate poses, after `simStartRace(2)` and `simStartRace(3)` is called. 

	- As soon as `simStartRace(1)`  or `simStartRace(3)` is called, `drone_2` (MSR opponent racer) will start flying. 

	- See `baseline_racer.py` for sample code. The previous bullet points are being called in wrapper functions in the following snippet in `baseline_racer.py`:
	```python
		baseline_racer.load_level(args.level_name)
		if args.level_name == "Qualifier_Tier_1":
		    args.race_tier = 1
		if args.level_name == "Qualifier_Tier_2":
		    args.race_tier = 2
		if args.level_name == "Qualifier_Tier_3":
		    args.race_tier = 3
		baseline_racer.start_race(args.race_tier)
	```

- To submit your results to the leaderboard:
	- Navigate to the [submission site](https://microsoft.github.io/AirSim-NeurIPS2019-Drone-Racing/upload.html), enter your team name in the proper field, and upload any number of [race logs](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/blob/master/docs/competition_guidelines.md#race-monitoring).   
It's ok to make a submission for as little as a single track and/or a single tier.   
You can find race logs inside of `AirSimExe/Saved/Logs/RaceLogs` in your downloaded binary folder.   
Please read [the race monitoring section](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/blob/master/docs/competition_guidelines.md#race-monitoring) in the competition guidelines for more details. 
	- The leaderboard will publish the results of a drone that is named `drone_1` (call [`generate_settings_file.py`](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/blob/master/baselines/generate_settings_file.py) to generate an AirSim settings file, as done for the `baseline_racer` below. 
	- Please submit a PDF file in the `report` section to help us verify the honesty of your submission for the Nov 21st deadline. Please summarize your approach for all tiers you make a submission for, with appropriate citations. The report PDF size should not exceed 10 MB, and should be a maximum of 4 pages in length. We leave the exact format of the report to your descrition, but the [IEEE template](https://ras.papercept.net/conferences/support/tex.php) is a good choice.  
	- We have emailed you a private key, which should be entered in the `Team ID` field. This helps us verify it was your team who indeed made the submission.   
	- The [leaderboard](https://microsoft.github.io/AirSim-NeurIPS2019-Drone-Racing/leaderboard.html) is updated once per day at 2100 PST.   
	If you do not see your results after 24 hours, please [email us](mailto:neuripsdronecontestinfo@gmail.com) with your team name and submitted log files.
	
## Sample code
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
		--race_tier 1 \
	```

## Quick API overview 
- Changing unreal environments  
	There are two ways to swap between environments / "unreal level", either via AirSIm API or by the UI menu.
	- Python API 
	Use `simLoadLevel(level_name="MainMenu")` to change Unreal environments on the fly.   
	 Possible values for `level_name` are : `"Soccer_Field_Easy"`, `"Soccer_Field_Medium"`, `"ZhangJiaJie_Medium"`, `"Building99_Hard"`. 
	Here's a quick snippet to iterate throught all the training environments.   
	Before trying this, please ensure you've downloaded the corresponding training (v0.3) / qualifier (v1.0) binaries, [as described above](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing#downloading-airsimexe-and-unreal-environments)

		```python
		import airsimneurips as airsim
		client = airsim.MultirotorClient()
		client.confirmConnection()

		# use this for training environments (v0.3)

		client.simLoadLevel('Soccer_Field_Easy')	
		client.simLoadLevel('Soccer_Field_Medium')	
		client.simLoadLevel('ZhangJiaJie_Medium')
		client.simLoadLevel('Building99_Hard')

		# use this for qualification environments (v1.0)
		client.simLoadLevel('Qualification_Tier_1')	
		client.simLoadLevel('Qualification_Tier_2')	
		client.simLoadLevel('Qualification_Tier_3')	

		```
	- UI Menu
		- Press `F10` to toggle the level menu
		- Click your desired level. (Note: the UI lists all the pakfiles in the `AirSim/AirSimExe/Content/Paks` directory. Ensure you downloaded the pakfile, if you are not able to see a particular environment)

## Questions
Please open a Github Issue on **this** repository (not [AirSim](https://github.com/microsoft/AirSim)) for any technical questions w.r.t. the Neurips competition. 
