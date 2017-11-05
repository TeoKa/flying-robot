//
//  robot.hpp
//  flying_robot_project
//
//  Created by Matteo Ciocca on 17/11/16.
//  Copyright © 2016 Matteo Ciocca. All rights reserved.
//

#ifndef robot_hpp
#define robot_hpp

#include "trajectory_planning.hpp"

#include <stdio.h>
#include <iostream>


using namespace std;

class robot {

public:

    // Get the target and call polynomial_position to move the robot: set his new coordinate
    void move(double dt,double t_end,double p,double xt,double yt,double zt,double xi,double yi,double zi,trajectory_planner* planner);


    // Place (initialize) the robot in the space
    robot(double x,double y,double z,double dx,double dy,double dz)
    : pos_x(x), pos_y(y), pos_z(z), vel_x(dx), vel_y(dy), vel_z(dz) {}


    //    In order to access to the private members of robot
    double get_x();
    double get_y();
    double get_z();
    double get_dx();
    double get_dy();
    double get_dz();

private:
    double pos_x;
    double pos_y;
    double pos_z;
    double vel_x;
    double vel_y;
    double vel_z;
};

#endif /* robot_hpp */


//    int polynomial_acceleration(int dt,int ddx);
