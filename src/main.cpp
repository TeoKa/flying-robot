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
void display(robot* flying_machine, trajectory_planner* planner, double t_current){

    // Display robot's status and more
    double vel_robot;

    vel_robot = planner->abs_velocity(flying_machine->get_dx(),flying_machine->get_dy(),flying_machine->get_dz());
    cout<<setprecision(3)<<"x  : "<<flying_machine->get_x()<<" y : "<<flying_machine->get_y()<<" z : "<<flying_machine->get_z();
//    cout<<setprecision(3)<<" | dx : "<<flying_machine->get_dx()<<" dy: "<<flying_machine->get_dy()<<" dz: "<<flying_machine->get_dz()<<" | abs_vel: "<<vel_robot<<" | time : "<<t_current<<"\n";
    cout<<setprecision(3)<<" | abs_vel: "<<vel_robot<<" | time : "<<t_current<<"\n";

    return;
}

int main(int argc, const char * argv[]) {

    double xt,yt,zt;                        // target position coordinates
    double xi,yi,zi;                        // initial position coordinates
    double v_max=-1, dS, t_end, p;          // polynomial parameters
    double dt=-1,t_current;                 // time parameters
    int quit_program=-1;                    // handle the simulation



    cout<<"\n\n";
    cout<<"======== Flying machine Program ======="<<endl;

    // positioning in space the flying machine
    cout<<"Initial x position of the flying machine (real number): ";
    cin>>xi;
    cout<<"Initial y position of the flying machine (real number): ";
    cin>>yi;
    cout<<"Initial z position of the flying machine (real number): ";
    cin>>zi;


    // Creating an object of robot and trajectory_planner class
    robot machine(xi,yi,zi,0,0,0);
    robot* flying_machine = &machine;
    trajectory_planner planner_mind;
    trajectory_planner* planner = &planner_mind;

    cout<<"=== Flying machine placed in space ===="<<endl;

    while(v_max<=0){
    // Define the Max velocity
    cout<<"Set maximum velocity of the flying machine (positive number): ";
    cin>>v_max;
    }
    while(dt<=0){
    // Define the Time Step
    cout<<"Set time step of the simulation (positive number): ";
    cin>>dt;
    }
    cout<<"=== Selected max vel and time step ===="<<endl;

    cout<<"Max Velocity: "<<v_max<<" | Time Step: "<<dt<<endl;
    cout<<"Flying machine state: "<<endl;
    display(flying_machine,planner,0);


    // Begin of the simulation

    while(1){
        // reset the simulation time to 0.0
        t_current = 0.0;
        quit_program = -1;
        cout<<"=========== New Simulation ============"<<endl;

        while (quit_program!=0 && quit_program!=1) {
        //        Declare what-to-do
            cout<<"Quit the program (1) or Move the robot (0): ";
            cin>>quit_program;
        }
        //        Condition: Exit
        if(quit_program == 1){
            cout<<"Exit the program\n\n\n";
            break;
        }

        // Get initial position: position of the flying machine
        xi = flying_machine->get_x();
        yi = flying_machine->get_y();
        zi = flying_machine->get_z();

        // Define a Target position in the 3D space
        cout<<"Set target x position of the flying machine: ";
        cin>>xt;
        cout<<"Set target y position of the flying machine: ";
        cin>>yt;
        cout<<"Set target z position of the flying machine: ";
        cin>>zt;

        // Dispaly Information
        cout<<"Current Position: "<<"("<<xi<<";"<<yi<<";"<<zi<<")"<<" | Target Position: "<<"("<<xt<<";"<<yt<<";"<<zt<<")"<<endl;

        // Equation (6) of [1]. Terms used to generate a 9th order polynomial trajectory in the 3D space.
        dS      = planner->delta_space_f_i(xt,yt,zt,xi,yi,zi);
        t_end   = 1.6406 * (dS/v_max);
        p       = 6144   * v_max/pow(t_end,8);

        // Show polynomial parameter
        cout<<"Distance: "<<dS<<" | Time to travel: "<<t_end<<" | p coefficient: "<<p<<endl;


        // Running the simulation
        cout<<"======== Display Iterations ==========="<<endl;
        while(t_current < t_end){
            flying_machine->move(t_current,t_end,p,xt,yt,zt,xi,yi,zi,planner);
            display(flying_machine,planner,t_current);
            t_current += dt;
        }

        // show the results.
        cout<<"============ Results =================="<<endl;
        cout<<"Initial Position: "<<"("<<xi<<";"<<yi<<";"<<zi<<")"<<"| Current Position: "<<"("<<flying_machine->get_x()<<";"<<flying_machine->get_y()<<";"<<flying_machine->get_z()<<")"<<" | Target Position: "<<"("<<xt<<";"<<yt<<";"<<zt<<")"<<endl;
        cout<<"Max Velocity: "<<v_max<<" | Time Step: "<<dt<<endl;

    }
    return 0;

}
