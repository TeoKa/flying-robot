//
//  trajectory_planning.hpp
//  fying_machine_better
//
//  Created by Matteo Ciocca on 18/11/16.
//  Copyright © 2016 Matteo Ciocca. All rights reserved.
//

// #ifndef trajectory_planning_hpp
// #define trajectory_planning_hpp

#pragma once

#include <stdio.h>
#include <iostream>



namespace planner
{

class trajectory_planner {

public:

    // @brief Constructor: place (initialize) the robot in the space
    trajectory_planner();

    // @brief Deconstructor
    virtual ~trajectory_planner() {}

    // Compute a single step of position, given a time step dt.
    double deltaPolynomialPosition(double dt,double t_end,double p);

    // Compute a single step of velocity, given a time step dt.
    double deltaPolynomialVelocity(double dt,double t_end,double p);

    double deltaSpace();

    // Get waypoints
    double Xi();
    double Yi();
    double Zi();
    double Xf();
    double Yf();
    double Zf();

    // Set waypoints
    void SetXYZi(double x, double y, double z);
    void SetXYZf(double x, double y, double z);

private:
    double xi_;
    double yi_;
    double zi_;
    double xf_;
    double yf_;
    double zf_;

};


// #endif /* trajectory_planning_hpp */


} /* planner */
