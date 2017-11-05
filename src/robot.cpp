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
    return pos_x;
}
double robot::get_y(){
    return pos_y;
}
double robot::get_z(){
    return pos_z;
}
double robot::get_dx(){
    return vel_x;
}
double robot::get_dy(){
    return vel_y;
}
double robot::get_dz(){
    return vel_z;
}

// Used to place (initialize) the robot in the space
void robot::place(double x,double y,double z,double dx,double dy,double dz)
  : pos_x(x), pos_y(y), pos_z(z), vel_x(dx), vel_y(dy), vel_z(dz) {}


void robot::move(double dt,double t_end,double p,double xt,double yt,double zt,double xi,double yi,double zi,trajectory_planner* planner){

    double d_pos;
    double d_vel;
    double abs_f_i;

    d_pos = planner->polynomial_position(dt,t_end,p);
    d_vel = planner->polynomial_velocity(dt,t_end,p);

    abs_f_i = planner->delta_space_f_i(xt,yt,zt,xi,yi,zi);

    // From a polynomial time-law to a trajectory in the 3D space, w.r.t. three components (px,py,pz). In this case a rectilinear path in the 3D space (pag. 182 of [2]).

    // position
    pos_x = xi + d_pos*(xt-xi)/abs_f_i;
    pos_y = yi + d_pos*(yt-yi)/abs_f_i;
    pos_z = zi + d_pos*(zt-zi)/abs_f_i;

    // velocity
    vel_x = d_vel*(xt-xi)/abs_f_i;
    vel_y = d_vel*(yt-yi)/abs_f_i;
    vel_z = d_vel*(zt-zi)/abs_f_i;

    return;

}
