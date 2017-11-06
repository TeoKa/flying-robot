//
//  main.cpp
//  flying_robot_project
//
//  Created by Matteo Ciocca on 17/11/16.
//  Copyright © 2016 Matteo Ciocca. All rights reserved.
//



#include "robot.hpp"
#include "trajectory_planning.hpp"

#include <iostream>
#include <cmath>
#include <stdio.h>
#include <iomanip>


// The algorithm uses the 9th order polynomial trajectory presented in [1] and the parametric representation of a segment in space as in [2].
// In the code, the references will be cited where needed.

// [1] Boryga, Marek. "Trajectory Planning of End-Effector for Path with Loop." Strojniški vestnik-Journal of Mechanical Engineering 60.12 (2014): 804-814.
// [2] Siciliano, Bruno, et al. Robotics: modelling, planning and control. Springer Science & Business Media, 2010.


// Dispaly the position of the flying machine.
void display(machine::robot& flying_machine,double t_current){

    // Display robot's status and more
    std::cout<<std::setprecision(3)<< "x  : " << flying_machine.x()
                         << " y : " << flying_machine.y()
                         << " z : "<<flying_machine.z();
    std::cout<<std::setprecision(3)<< " | abs_vel: " << flying_machine.absVelocity()
                         <<" | time : " << t_current << "\n";

    return;
}

int main(int argc, const char * argv[]) {

    double xt,yt,zt;                        // target position coordinates
    double xi,yi,zi;                        // initial position coordinates
    double v_max=-1, t_end, p;              // polynomial parameters
    double dt=-1,t_current;                 // time parameters
    int quit_program=-1;                    // handle the simulation


    std::cout<<"\n\n";
    std::cout<<"======== Flying machine Program ======="<< "\n";

    // positioning in space the flying machine
    std::cout << "Initial x position of the flying machine (real number): ";
    std::cin  >> xi;
    std::cout << "Initial y position of the flying machine (real number): ";
    std::cin  >> yi;
    std::cout << "Initial z position of the flying machine (real number): ";
    std::cin  >> zi;


    // Creating an object of robot and trajectory_planner class
    machine::robot flying_machine(xi,yi,zi,0,0,0);
    planner::trajectory_planner planner;


    std::cout << "=== Flying machine placed in space ====" << "\n";

    while( v_max <= 0 )
    {
      // Define the Max velocity
      std::cout << "Set maximum velocity of the flying machine (positive number): ";
      std::cin  >> v_max;
    }

    while( dt <= 0 )
    {
      // Define the Time Step
      std::cout << "Set time step of the simulation (positive number): ";
      std::cin  >> dt;
    }

    std::cout << "=== Selected max vel and time step ====" << "\n";

    std::cout << "Max Velocity: " << v_max << " | Time Step: " << dt << "\n";
    std::cout << "Flying machine state: " << "\n";
    display(flying_machine,0);


    // Begin of the simulation

    while(1)
    {
        // reset the simulation time to 0.0
        t_current = 0.0;
        quit_program = -1;
        std::cout << "=========== New Simulation ============" << "\n";

        while (quit_program!=0 && quit_program!=1)
        {
              // Declare what-to-do
              std::cout << "Quit the program (1) or Move the robot (0): ";
              std::cin  >> quit_program;
        }

        // Condition: Exit
        if(quit_program == 1)
        {
            std::cout << "Exit the program..\n..Thanks for playing\n\n";
            break;
        }

        // Get initial position: position of the flying machine
        planner.SetXYZi(flying_machine.x(),flying_machine.y(),flying_machine.z());

        // Define a Target position in the 3D space
        std::cout << "Set target x position of the flying machine: ";
        std::cin  >> xt;
        std::cout << "Set target y position of the flying machine: ";
        std::cin  >> yt;
        std::cout << "Set target z position of the flying machine: ";
        std::cin  >> zt;

        // Dispaly Information
        std::cout << "Current Position: (" << flying_machine.x() << ";"
            << flying_machine.y() << ";" << flying_machine.z() << ")"
            << " | Target Position: (" << xt << ";" << yt << ";" << zt << ")" << "\n";

        planner.SetXYZf(xt,yt,zt);

        // Equation (6) of [1]. Terms used to generate
        // a 9th order polynomial trajectory in the 3D space.
        t_end   = 1.6406 * (planner.deltaSpace()/v_max);
        p       = 6144   * v_max/pow(t_end,8);

        // Show polynomial parameter
        std::cout << "Distance: " << planner.deltaSpace()
            << " | Time to travel: " << t_end << " | p coefficient: " << p << "\n";

        // Running the simulation
        std::cout << "======== Display Iterations ===========" << "\n";
        while(t_current < t_end)
        {
            flying_machine.move(t_current,t_end,p,planner);
            display(flying_machine,t_current);
            t_current += dt;
        }

        // show the results.
        std::cout << "============ Results ==================" << "\n";
        std::cout << "Initial Position: (" << xi << ";" << yi << ";" << zi << ")"
        << "| Current Position: " << "(" << flying_machine.x()
        << ";" << flying_machine.y() << ";" << flying_machine.z() << ")"
        << " | Target Position: " << "(" << xt << ";" << yt << ";"<< zt << ")" << "\n";

        std::cout << "Max Velocity: " << v_max << " | Time Step: " << dt << "\n";
    }
    return 0;

}
