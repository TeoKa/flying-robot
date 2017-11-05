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



double robot::get_x(){
    return x_;
}
double robot::get_y(){
    return y_;
}
double robot::get_z(){
    return z_;
}
double robot::get_dx(){
    return dx_;
}
double robot::get_dy(){
    return dy_;
}
double robot::get_dz(){
    return dz_;
}


robot::robot(double x,double y,double z,double dx,double dy,double dz)
: x_(x), y_(y), z_(z), dx_(dx), dy_(dy), dz_(dz) {}

void robot::move(double dt,double t_end,double p,double xt,double yt,double zt,double xi,double yi,double zi,trajectory_planner* planner){

    double d_pos;
    double d_vel;
    double abs_f_i;

    d_pos = planner->polynomial_position(dt,t_end,p);
    d_vel = planner->polynomial_velocity(dt,t_end,p);

    abs_f_i = planner->delta_space_f_i(xt,yt,zt,xi,yi,zi);

    // From a polynomial time-law to a trajectory in the 3D space, w.r.t. three components (px,py,pz). In this case a rectilinear path in the 3D space (pag. 182 of [2]).

    // position
    x_ = xi + d_pos*(xt-xi)/abs_f_i;
    y_ = yi + d_pos*(yt-yi)/abs_f_i;
    z_ = zi + d_pos*(zt-zi)/abs_f_i;

    // velocity
    dx_ = d_vel*(xt-xi)/abs_f_i;
    dy_ = d_vel*(yt-yi)/abs_f_i;
    dz_ = d_vel*(zt-zi)/abs_f_i;

    return;

}
