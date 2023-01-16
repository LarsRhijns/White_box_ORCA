# Introduction
This repository contains an implementation of ORCA. This implementation mainly focuses on being easy to configure for 
various scenarios and gathering useful information about them. The ORCA solution is not perfect yet but in most cases
shows the validity of the idea behind the algorithm. 

This project was part of the RO47005 course of the TU Delft in 2022. 

### Getting started: environment setup
Run the code below to initialize the repository. Run it in the location you want. Currently the environment is working only on Linux. To work with this repository on windows please install the packages mentioned below the commands manually. <b>Note</b> that the step of creating an environment takes around 5 minutes.

```
    git clone git@github.com:LarsRhijns/White_box_ORCA.git
    cd White_box_ORCA
    git submodule init
    git submodule update
    conda env create -f environment.yml
    conda activate orca
```

Conda can be very slow and get stuck, so if the above environment creation does not work, please manually install numpy, 
matplotlib, pybullet, gym, scipy, shapely, pandas and [urdfenvs](https://github.com/maxspahn/gym_envs_urdf/issues/136)
(urdfenvs requires a specific installation to work, please use the link)

### Running the solution
After the environment is set up and activated, make sure your IDE is using the orca environment as python interpreter and simply run either one of the two main scripts described below.

### How to use White_box_ORCA
There are two mains, namely in `Main_single_run.py` and `Main_simulations.py`. The first one is used to run a single 
setting, while the latter is used to gather information about the solution in many simulations with different settings. 
The results are then stored in a seperate folder (will be created upon running) called `data`. 

Within `Main_single_run.py` there are several variables at the beginning of the script to set before a run:
<ul>
    <li><code>tau</code> is the time horizon the robots will use.</li>
    <li><code>simulation_cycle</code> is the timestep used for the simulation</li>
    <li><code>robot_amount</code> for the amount of robots in the simulation</li>
    <li><code>circle_radius</code> is the radius of the circle where the robots will be initialized on</li>
    <li><code>total_time</code> is the maximal time for the simulation</li>
    <li><code>obstacle_run</code> for if you want a run with obstacles or not</li>
    <li><code>obstacle_radius</code> to define the radius of obstacles globally</li>
    <li><code>obstacle_location</code> to set a single location of an obstacle globally</li>
    <li><code>plot_velocities</code> for if you want to see the paths of the robots during the simulation (slows down simulation)</li>
</ul>

Furthermore, there are several things that can easily be changed to better fit your use-case:
<ul>
    <li><b>Robot start and goal position initialization:</b> within <code>run_point_robot</code> the robot start and 
        goal are initialized on a circle with the <code>circle_radius</code> mentioned before. These locations can of course
        be initialized to your liking</li>
    <li><b>Changing the robot model: </b> right now this is defined by a holonomic point robot urdf from the
    <code>gym_envs_urdfs</code> package, but feel free to use any other model, either made by yourself or from the 
        mentioned package</li>
    <li><b>Ideal velocity calculation:</b> currently we use a very simple control method to calculate the ideal velocity
        for each robot. This can be changed to any control sequence matching your robot within <code>velocity_reference.py</code> 
        by changing the function <code>calculate_vref</code></li>
    <li><b>Static obstacles: </b> currently only a single static obstacle is used with a given radius and location. 
        You can change the amount and way this is initialized within <code>run_point_robot</code> and the urdf variable
        <code>obst1Dict</code> at the top of the script</li>
    <li><b>Simulation environment: </b> besides the robots and static obstacles you can also change the environment as 
        a whole in <code>run_point_robot</code>, e.g. add walls or other obstacles. Keep in mind that you also need
        a representation for the `Observation` instance to consider the static/dynamic obstacle within the ORCA cycles
        as well</li>
</ul>

Within `Main_simulations.py` you will find the same variables as mentioned before to configure your simulation, however
in the main method you will also find `robots_amount` and `taus` with multiple amounts and taus in them, which will be 
run one after another. You can also set `render` to False to speed up the process.

### Contributors
<ul>
    <li>Steven Mitchell Tan</li>
    <li>Kane de Roodt</li>
    <li>Guido Dumont</li>
    <li>Lars Rhijnsburger</li>
</ul>


