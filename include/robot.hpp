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

    // @brief Constructor: place (initialize) the robot in the space
    robot(double x,double y,double z,double dx,double dy,double dz);

    // @brief Deconstructor
    virtual ~robot() {}

    // Get the target and call polynomial_position to move the robot:
    // set his new coordinate
    void move(double dt,double t_end,double p,trajectory_planner& planner);


    // Get Methods
    double x();
    double y();
    double z();
    double dx();
    double dy();
    double dz();
    double absVelocity();

    // Set Methods
    void SetPosition(double x, double y, double z);
    void SetVelocity(double dx, double dy, double dz);

private:
    double x_;
    double y_;
    double z_;
    double dx_;
    double dy_;
    double dz_;
};

#endif /* robot_hpp */
