//
//  robot.cpp
//  flying_robot_project
//
//  Created by Matteo Ciocca on 17/11/16.
//  Copyright © 2016 Matteo Ciocca. All rights reserved.
//

#include "robot.h"
#include "trajectory_planning.h"


#include <iostream>
#include <cmath>
#include <stdio.h>
#include <iomanip>


namespace machine
{

robot::robot(double x,double y,double z,double dx,double dy,double dz)
: x_(x), y_(y), z_(z), dx_(dx), dy_(dy), dz_(dz) {}

void robot::move(double dt,double t_end,double p,
                 planner::trajectory_planner& planner)
{

    double d_pos;
    double d_vel;

    d_pos = planner.deltaPolynomialPosition(dt,t_end,p);
    d_vel = planner.deltaPolynomialVelocity(dt,t_end,p);

    // From a polynomial time-law to a trajectory in the 3D space,
    // w.r.t. three components (px,py,pz). In this case a rectilinear path
    // in the 3D space (pag. 182 of [2]).

    // position
    SetPosition(planner.Xi() + d_pos*(planner.Xf()-planner.Xi())/planner.deltaSpace(),
                planner.Yi() + d_pos*(planner.Yf()-planner.Yi())/planner.deltaSpace(),
                planner.Zi() + d_pos*(planner.Zf()-planner.Zi())/planner.deltaSpace());

    // velocity
    SetVelocity(d_vel*(planner.Xf()-planner.Xi())/planner.deltaSpace(),
                d_vel*(planner.Yf()-planner.Yi())/planner.deltaSpace(),
                d_vel*(planner.Zf()-planner.Zi())/planner.deltaSpace());

}


// Get functions
double robot::x(){
    return x_;
}
double robot::y(){
    return y_;
}
double robot::z(){
    return z_;
}
double robot::dx(){
    return dx_;
}
double robot::dy(){
    return dy_;
}
double robot::dz(){
    return dz_;
}

double robot::absVelocity()
{
    // Calculate the absolute velocity: |v(t)| < v_max
    return sqrt( pow(dx(),2) + pow(dy(),2) + pow(dz(),2) );
}


// Set Methods
void robot::SetPosition(double x, double y, double z)
{
    x_ = x;
    y_ = y;
    z_ = z;
}

void robot::SetVelocity(double dx, double dy, double dz)
{
  dx_ = dx;
  dy_ = dy;
  dz_ = dz;
}

}
