# Competition Guidelines
The challenge aims to encourage the NeurIPS research community to push the boundaries of building competitive autonomous systems through head-to-head drone races. To broaden the audience and bring people to drones from all kinds of disciplines. All racing will be done in simulation.
## What is the Challenge?
The competition will be held in two different phases: Qualification phase and Live races. The qualification phase is aimed to help the teams push their solution to be competitive and ready for the Live races. The second phase will be held live at the NeurIPS 2019 conference, where teams will be competing against each other.

The competition will be held across three different tiers, where each tier will focus on a different aspect of autonomous systems.
The races will be carried out in three different environments. Test tracks in these environments are available for download [here](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases). Tracks are delineated by a series of gates through which the drone must pass. Additionally, we will be publishing a reference solution for all the three tiers that the participants can use to get started.

### Tier I: Planning Only
Race an opponent drone to the finish line given both ego and opponent’s poses in real time. The task is to plan the most time-optimal path to complete the course while avoiding collisions with the opponent drone and the track gates. There is no perceptual uncertainty in this tier.

- The Challenge: Finish the track quicker than your opponent, without crashing.
- The Focus: Competitive planning algorithms, adversarial race tactics.

### Tier II: Perception Only
Equipped with two RGB cameras – one facing forward and the other downward – and given the ground truth pose of your drone, complete the racecourse as fast as possible while avoiding collisions with the track gates. The approximate gate poses will be provided at the race start time, but the true race gates will be perturbed from these approximate poses.
- The Challenge: Use perception to sense the spatial whereabouts and order of gates in the track.
- The Focus: Vision based perception.

### Tier III: Full Autonomy 
This is the combination of Tiers I and II. Given forward and downward facing RBD cameras and the ground truth pose of your own drone, complete the racecourse as fast as possible while avoiding collisions with the gates and the opponent drone. You will again be provided with approximate gate poses before the race starts. The actual gate poses will be a perturbed version of the given gate poses.
- The Challenge: Use perception both to locate the gates and your opponent, and finish the race quicker than your opponent.
- The Focus: Full stack autonomy, incorporating on board perception coupled with competitive planning. 

## How to participate in the Challenge?
Participants can download the binaries for the drone racing simulation environment [here](https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases). Participants interface with the simulation by supplying, at each time step, a target waypoint and velocity for the drone. The target waypoint is the point in 3D space where the user wants the drone to be located at the next available time step, and the target velocity is the 3D velocity it will be travelling at when it reaches that point. The simulation environment will already contain a low-level trajectory planner and feedback controller to control the dynamics of the drone to achieve the specified waypoint and velocity, thereby simplifying the interface as much as possible. Technical details of the interface can be found [here](//link).

The exact form of submission is to be determined.  The participants are expected to develop everything in Python using typical packages (numpy, scipy, etc.) and the provided AirSim API.

## Qualification Rules
To determine a leaderboard among all submissions during the qualification phase, every submission is evaluated across three rounds − one for each track − consisting of two races. Of the two races, only the best result is counted towards a participant’s score.
As there are two drones in Tier I and III, the two drones will start at offset positions. To ensure fairness, every race is repeated with mirrored start positions and the score is determined by averaging the lap times of the two runs. 

The qualifier binary contains three tracks of increasing difficulty levels: Easy, Medium, and Hard. In each race each drone must complete the racecourse by passing through a series of gates in a given order without colliding with the environment or the opponent drone (Tier I and III). Missing a gate and colliding with the environment incurs a penalty. Violating the safety radius incurs a penalty as well, and a collision with the other drones will disqualify the drone causing the collision. See [Penalties and Disqualifications](//link) in the following for details.
The summed score over all tracks determines the overall score in the leaderboard (see [Metrics and Scoring](//link) below).

The exact conditions for a team to qualify for the live tournament at the NeurIPS 2019 conference depends on the number of participants and feedback we gather during the qualification and will be detailed before 15th October, 2019. Qualifier binaries will also be released during this timeframe.

## Definitions
**Reference Submission.** During the qualification, all participants in Tiers I and III are evaluated against the same reference submission. The reference submission is developed by Microsoft Research and the Stanford Multi-robot Systems Lab and will not be disclosed.

**Mirrored Race Pairs.** To ensure fairness of evaluation, every race is always carried out twice with switched start positions.

**Round.** For every submission a set of 2 mirrored races (4 races total) is carried out for each of 3 different track environments. Each race has different starting positions for the drones. The best run in each environment counts toward overall points. Runs without disqualifications take precedence.

**Chasing drone.** At any point of the race, the chasing drone is the drone behind. This either means it has passed less gates than the other drone, or in case the same number of gates have been passed, the following drone is the drone with larger Euclidean distance to the next gate’s center. The other drone is referred to as the leading drone.
It is the job of the chasing drone to avoid collisions with the leading drone. This means, that the leading drone can make use of its lead by blocking the pathway of the chaser.

## Tracks
Α race is carried out on a track consisting of a series of gates that must be traversed in order and in the correct direction.

### Start Positions
The start positions are specified for each track and in general consists of two non-colliding positions before the first gate.

### Environments
There are three environments that will house race tracks for this competition.
1. **SoccerField**, Easy  
A simple outdoors environment with few obstacles consisting of a green soccer field surrounded by buildings and trees.
2. **ZhangJiaJie**, Medium  
 A mountainous landscape modeled after the national park with the same name in the Hunan province of China.
3. **Microsoft Research Building 99**, Hard  
 A complex indoor environment with varying lighting conditions in one of Microsoft’s very own buildings. 

### Difficulty
The three tracks have increasing difficulties and meet the following specifications.

1. **Easy**
   - All gates are at the same altitude above the ground (2D tracks).
   - All gates are the same dimensions (inner rectangle 1.6x1.6 m).
   - Gates are only rotated about the z-axis.
   - The next gate is always fully in the field-of-view (FOV).
2. **Medium**
   - Gates are at varying heights (2.5D tracks).
   - Gate types are scalable.
   - Gates are only rotated about the z-axis.
   - The next gate is always at least partially in the FOV. 
   - The thickness of the gates differs (i.e. the ratio of the internal and external dimensions of the gates will not be consistent).
3. **Hard**
   - Gates are arbitrarily oriented (3D tracks).
   - Gates are arbitrarily sized.
   - The next gate is not necessarily in the FOV.
   - There may be obstacles in between gates that have to be detected (i.e. walls, chairs, etc.). Holes and borders of varying size for gates.

### Gates
The gates are rectangular and marked using a "race-tape" checker pattern with different colors on each side.
- The front side is colored green
- The (inner and outer) sides blue and
- The back side of the gate is colored red.

Example [picture of gate].

## Metrics and Scoring

### Lap time (Lap-T)
The lap time is the time needed for a lap, that is, the time needed to pass all gates in order and right direction. The lap time is capped at a certain maximal lap time, which is given for every racetrack. In case of a disqualification the lap time is set to be this maximal lap time.

### Lag time (Lag-T)
For Tier I and III to measure how well the participants performs with respect to the reference drone, the score is determined by the *lag time*.

The lag time is the difference of the reference drone’s lap time minus the participants lap time.
This is measured with both drones competing at the same time on the same track.

### Overall Score
The sum of all lag times (Tier II: lap times) plus point deduction (penalties, see later) across all tracks and all counted runs determine the overall score.

### Collision Avoidance. (Tier I and III)
It is the job of the chasing drone to avoid the leading drone. In the case of a collision, penalties are always incurred to the chasing drone.
There are two radii to limit the number of collisions and resulting disqualifications.
1. **Collision Radius**  
For collision purposes, the drones are modeled simple boxes. In case the distance between the two drones deceeds 0.6m a (simulated) collision occurs. Such a collision removes the violating drone from the game, and the other drone is allowed to finish without an opponent.
2. **Safety Radius**  
The safety radius is set to 0.4m. If the distance between the two drones deceeds 0.8m, a linear penalty term is incurred for violating this constraint.

### Penalties
There are the following three types of penalties.
1. **Missed gates**  
For each gate, the penalty incurred is some fixed time, dependent on the location of that particular gate.
2. **Violation of safety radius**  
At every time step the safety radius is violated by *x* meters, a (10 *x dt*) penalty is incurred. The penalty is only incurred to the following drone.
3. **Collision with environment/gate**  
Every collision instance incurred within a 1 second window is penalized with a three second time delay.

### Disqualifications
There are two reasons for a disqualification: Timeout and Drone on Drone Collision.

1. **Timeout**  
If a drone does not finish within the maximal lap time of a track, it is disqualified.
2. **Collision with another drone**  
If a collision happens (i.e. the collision radius is deceeded), the following drone is disqualified.

In case of a disqualification, the score is equivalent to achieving the maximal lap time.

***Example 1***: *On a racetrack with maximal lap time of 100s, the reference drone finishes the track within 20s. If the participant’s drone does not complete the track within 100s, it is disqualified,* and the score is 100s-20s = 80s.

***Example 2***: *If the reference drone collides with the participant’s drone from behind, it is removed from the game immediately.* The score is computed as if the participant's drone finished within the maximal lap time.

## Live Tournament Rules
The rules for the live tournament carried out during the NeuRIPS 2019 conference will be published after feedback from the qualification round is gathered.

