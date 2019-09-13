# Competition Guidelines
The challenge aims to encourage the NeurIPS research community to push the boundaries of building competitive autonomous systems through head-to-head drone races. To broaden the audience and bring people from different disciplines to drones, all racing will be done in simulation.
## Challenge Description
The competition will be held in two phases: a qualification phase and a live racing phase. The qualification phase is to help the teams push their solution to be competitive and ready for the live races. The second phase will be held live at the NeurIPS 2019 conference, where teams will be competing against each other.

The competition will also be held across three tiers, with each tier focusing on a different aspect of autonomous systems.
The races will be carried out in three test tracks of increasing difficulty. Test tracks in different environments are available for download [here](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases). Tracks are marked by a series of gates through which the drone must pass. Additionally, we will be publishing a reference solution for all the three tiers that the participants can use to get started.

### Tier I: Planning Only
Race an opponent drone to the finish line given the poses of self and the opponent drone in real time. The task is to plan the most time-optimal path that completes the course while avoiding collisions with the opponent drone as well as the track gates. Poses of the gates are also provided.

- Challenge: Finish the track quicker than your opponent, without crashing.
- Focus: Competitive planning algorithms, adversarial race tactics.

### Tier II: Perception Only
Equipped with a forward-facing RGB camera and given the ground truth pose of your drone - complete the racecourse as fast as possible while avoiding collisions with the track gates. The approximate gate poses will be provided at the start of the race, but the actual gates poses will be slightly perturbed from the approximate poses.
- Challenge: Use perception to sense the spatial whereabouts and order of gates in the track.
- Focus: Vision based perception.

### Tier III: Full Autonomy 
This is the combination of Tiers I and II. Given a forward-facing RGB camera and the ground truth pose of your own drone, complete the racecourse as fast as possible while avoiding collisions with the gates as well as the opponent drone. You will again be provided with approximate gate poses before the race starts. The actual gate poses will be perturbed versions of the approximate ones.

- Challenge: Use perception both to locate the gates as well as your opponent, and finish the race quicker than your opponent.
- Focus: Full stack autonomy, integrating perception with competitive planning. 

## How to Participate in the Challenge
Participants can download binaries to experiment with the drone racing simulation environment using the AirSim API. The binaries are located [here](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases). The simulation environment will later contain a low-level trajectory planner and feedback controller to control the dynamics of the drone to achieve the specified waypoint and velocity, thereby simplifying the interface as much as possible.

The exact form of submission is to be determined.  The participants are expected to develop everything in Python using typical packages (numpy, scipy, etc.) and the provided AirSim API.

### Environments
There are three environments that will house race tracks for this competition.
1. **SoccerField**: 
A simple outdoors environment with few obstacles consisting of a green soccer field surrounded by buildings and trees.
2. **ZhangJiaJie**: 
 A mountainous landscape modeled after the national park with the same name in the Hunan province of China.
3. **Microsoft Research Building 99**: 
 A complex indoor environment with varying lighting conditions in one of Microsoft’s very own buildings. 

## Definitions
**Reference Submission.** A reference submission is developed by Microsoft Research and the Stanford Multi-robot Systems Lab (which will not be disclosed). During the qualification, all participants in Tiers I and III are evaluated against the same reference submission, which acts as the opponent drone. 

**Mirrored Race Pairs.** To ensure fairness of evaluation, every race is always carried out twice. For tiers I and III, which involve two drones, the start positions are switched between runs.

**Round.** For every submission a set of 2 mirrored races (a total of 4 races) is carried out for each of 3 different track environments. Each race has different starting positions for the drones. The average score in each environment counts toward overall points. Runs without disqualifications take precedence.

**Start Positions.** The start positions are specified for each track and in general consists of two non-colliding positions before the first gate.

**Chasing drone.** At any point of the race, the chasing drone is the drone behind. This either means it has passed less gates than the other drone, or in case the same number of gates have been passed, the following drone is the drone with larger Euclidean distance to the next gate’s center. The other drone is referred to as the leading drone.
It is the job of the chasing drone to avoid collisions with the leading drone. This means, that the leading drone can make use of its lead by blocking the pathway of the chaser.

### Gates
The gates are rectangular and marked using a "race-tape" checker pattern with different colors on each side.
- The front side is colored green.
- The (inner and outer) sides are blue.
- The back side of the gate is colored red.

## Tracks
Α race is carried out on a track consisting of a series of gates that must be traversed in order and in the correct direction. There are three tracks with increasing difficulties, which meet the following specifications.

1. **Easy**
   - All gates are at the same altitude above the ground (2D tracks).
   - All gates are the same dimensions.
   - Gates are only rotated about the z-axis.
   - When passing through one gate, the next gate is always fully in the field-of-view (FOV) of the drone's camera.
2. **Medium**
   - Gates are at varying heights (2.5D tracks).
   - Gate types are scalable.
   - Gates are only rotated about the z-axis.
   - The thickness of the gates differs (i.e. the ratio of the internal and external dimensions of the gates will not be consistent).
   - When passing through a gate, the next gate is always at least partially in the camera FOV. 
3. **Hard**
   - Gates are arbitrarily oriented (3D tracks).
   - Gates are arbitrarily sized and proportioned.
   - When passing through a gate, the next gate is not necessarily in the camera FOV.
   - There may be obstacles in between gates that have to be detected (i.e. walls, chairs, etc.).

## Qualification Rules
The qualifier binary will contain three tracks classified into three categories: Easy, Medium, and Hard. In each race, each drone must complete the racecourse by passing through a series of gates in a given order without colliding with the environment or the opponent drone (Tier I and III). Missing a gate and colliding with the environment incurs a penalty. Violating the safety radius incurs a penalty as well, and a collision with the other drones will disqualify the drone causing the collision. See [Penalties and Disqualifications](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/blob/master/docs/competition_guidelines.md#penalties) in the following for details.

To determine a leaderboard among all submissions during the qualification phase, every submission is evaluated across the three tracks. To ensure fairness, each track is required to have results from two runs. The score for each track is determined by averaging the metrics of the two runs. As there are two drones in Tier I and III, the two drones will start at offset positions. When the race is repeated, the start positions of the participant and the opponent drones will be mirrored.

The summed score over all tracks determines the overall score in the leaderboard (see [Metrics and Scoring](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/blob/master/docs/competition_guidelines.md#metrics-and-scoring) below).

The exact conditions for a team to qualify for the live tournament at the NeurIPS 2019 conference depends on the number of participants and feedback we gather during the qualification and will be detailed before 15th October, 2019. Qualifier binaries will also be released during this timeframe.

## Metrics and Scoring

### Nummber of gates passed (**G**)
This is the primary metric in the scoring strategy. The drone that navigates through the most number of gates in a 2-drone race will always be the winner.

G = Number of gates passed / Total number of gates

### Lag time (**T**)
For tiers I and III, in order to measure how well the participants perform with respect to the reference drone, another aspect of the score is determined by a *lag time*. At any instant, the lag time (or split time) is the difference between the participant drone’s lap time and the reference drone's lap time. If the participant drone is in second place, the lag time will be positive. The scoring metric only considers the lag time at the end of the race - when the chasing drone has crossed the final gate.

### Maximal lap time

For every racetrack, there is a maximum lap time ***t***<sub>max</sub>. If the participant drone is unable to complete the track within this timeframe, it is disqualified. 

### Collision Penalties  

**Drone-Drone collision (Tier I and III)**  
It is the job of the chasing drone to avoid the leading drone. In the case of a collision, penalties are always incurred by the chasing drone.

For collision purposes, the drones are modeled as simple boxes. A collision is registered any time the distance between the two drones falls below **d**. One drone-drone collision is allowed, for which the chasing drone incurs a flat time penalty of ***t***<sub>c</sub>. Any subsequent collisions will disqualify the chasing drone, and the other drone is allowed to finish without an opponent.

**Collision with environment/gates**  
Every collision with other objects in the environment such as the gates is penalized with a time delay represented by ***t***<sub>***e***</sub>. The drone-environment collisions are checked over a specific window of time ***t***<sub>cw</sub>. Multiple collisions registered under the window of ***t***<sub>cw</sub> are treated as a single collision.

**Values**

***NOTE: AS OF NOW, THESE VALUES ARE ONLY NOMINAL AND ARE SUBJECT TO CHANGE.***

| Quantity | Description                                                             | Value      |
|----------|-------------------------------------------------------------------------|------------|
| ***t***<sub>***max***</sub>    | Maximal lap time, within which the drones should complete the track    | 100 seconds
| ***d***        | Distance threshold for collision detection                              | 0.3 meters |
| ***t***<sub>***c***</sub>      | Time penalty incurred by chasing drone for colliding with leading drone | 3 seconds  |
| ***t***<sub>***e***</sub>     | Time penalty incurred by drone for colliding with environment           | 3 seconds  |
| ***t***<sub>***cw***</sub>     | Time window for collision checking                                      | 1 second   |

### Overall Score
During a two-drone race, the drone that passed through the most number of gates automatically wins. If both drones cross the same number of gates, the winner is determined by the lag time. As the drones navigate the race track, any time penalties incurred (as described above) are added to the current lap time of the drone responsible, which in turn affects the lag time between the drones. 

### Disqualifications
There are two reasons for a disqualification: Timeouts and multiple drone-drone collisions.

1. **Timeout**  
If a drone does not finish within the maximal lap time of a track, it is disqualified.
2. **Collision with another drone**  
If a drone-drone collision happens more than once, the chasing drone is disqualified.

In case of a disqualification, the score is equivalent to achieving the maximal lap time.

## Race Monitoring

As a drone is navigating the racetrack, the race progress is streamed to a local log file, which can be found at ``Saved/Logs/RaceLogs/[TimeStamp]_[Level]_tier_[Tier#]_[Race#].log``. This log is updated with data such as the current odometry of the drone, number of gates passed/missed, times etc. It may be useful to continually process this log file client-side. The format of the data generated in this log file can be seen through this example:

```
A odometry_XYZRPY (75.000,-200.000,2882.102,0.000,0.000,90.000)
A gates_passed 0
A gates_missed 0
A collision_count 0
A time 8
A penalty 0
A disqualified 0
A finished 0
```

We have an example python script ([`scripts/logging/log_monitor.py`](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/blob/master/scripts/logging/log_monitor.py)) to help you analyze the log file. This script demonstrates how one can detect when to start a new race, which may be useful for training scenarios. 

Please note that in the qualification round, participants will be required to submit these generated race logs for evaluation.

## Live Tournament Rules
The rules for the live tournament carried out during the NeuRIPS 2019 conference will be published after feedback from the qualification round is gathered.

