//
//  robot.cpp
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



robot::robot(double x,double y,double z,double dx,double dy,double dz)
: x_(x), y_(y), z_(z), dx_(dx), dy_(dy), dz_(dz) {}

void robot::move(double dt,double t_end,double p,double xt,double yt,double zt,double xi,double yi,double zi,trajectory_planner& planner){

    double d_pos;
    double d_vel;
    double abs_f_i;

    d_pos = planner.polynomial_position(dt,t_end,p);
    d_vel = planner.polynomial_velocity(dt,t_end,p);

    abs_f_i = planner.delta_space_f_i(xt,yt,zt,xi,yi,zi);

    // From a polynomial time-law to a trajectory in the 3D space, w.r.t. three components (px,py,pz). In this case a rectilinear path in the 3D space (pag. 182 of [2]).

    // position
    SetPosition(xi + d_pos*(xt-xi)/abs_f_i,
                yi + d_pos*(yt-yi)/abs_f_i,
                zi + d_pos*(zt-zi)/abs_f_i);

    // velocity
    SetVelocity(d_vel*(xt-xi)/abs_f_i,
                d_vel*(yt-yi)/abs_f_i,
                d_vel*(zt-zi)/abs_f_i);

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
