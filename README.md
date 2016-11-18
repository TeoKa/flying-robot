# System description

A flying robot is considered as a simple point translating in the environment (3 DOF). This robot has a maximum velocity but we suppose that an infinite acceleration is possible and the system has no delay. The environment imposes no constraint, so robot movements in the world perfectly reflect the command. At start, the robot is at a predefined initial position in the environment and it does not move.

# System output

- at program launch time, the user to set the initial position of the robot in the world, its maximum velocity and the time step of the simulation.
The user can then either:
- input a target position where the robot has to go to. The robot then starts moving from its current position to that target position.
- ask for exiting the program. The program immediately ends.

If the robot moves it prints its current position (X, Y, Z, |v(t)|, time) in the terminal (at every iteration). Once it has reached the target position, the robot stops and alerts the user with a message in the terminal.


## Run the program
<!-- alt+9 = ` -->
```
cd build
cmake ..
make
./flyingRobot
```
Have Fun!



## References

Polynomial trajectory inspired by:

[Trajectory Planning of an End-Effector for Path with Loop](http://ojs.sv-jme.eu/index.php/sv-jme/article/viewFile/sv-jme.2014.1965/707)

Segment in 3D space:

[Robotics: modelling, planning and control](https://books.google.it/books?hl=it&lr=&id=jPCAFmE-logC&oi=fnd&pg=PR8&dq=robotics+siciliano&ots=3UPr3jLvDt&sig=OfaLJUdGFDj4bFuGryUhKAHT7X4#v=onepage&q=robotics%20siciliano&f=false)
