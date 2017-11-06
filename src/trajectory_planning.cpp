//
//  trajectory_planning.cpp
//  fying_machine_better
//
//  Created by Matteo Ciocca on 18/11/16.
//  Copyright © 2016 Matteo Ciocca. All rights reserved.
//

#include "trajectory_planning.h"



#include <iostream>
#include <cmath>
#include <stdio.h>
#include <iomanip>


namespace planner
{


trajectory_planner::trajectory_planner() : xi_(0), yi_(0), zi_(0),
                                           xf_(0), yf_(0), zf_(0) {}


double trajectory_planner::deltaPolynomialPosition(double dt,double t_end,double p){

    // 9th order polynomial trajectory as in [1] Eq. (3)

    double d_pos,coef1,coef2,coef3,coef4,coef5,coef6,coef7,coef8;

    coef1 = 72;coef2 = 16;coef3 = 19;coef4 = 168;
    coef5 = 5;coef6 = 48;coef7 = 20;coef8 = 96;

    coef1 = 1/coef1;coef2 = 1/coef2;coef3 = coef3/coef4;
    coef4 = coef5/coef6;coef5 = 1/coef7;coef6 = 1/coef8;


    d_pos = -p*(coef1*pow(dt,9)-coef2*t_end*pow(dt,8)+coef3*pow(t_end,2)*pow(dt,7)-coef4*pow(t_end,3)*pow(dt,6)+coef5*pow(t_end,4)*pow(dt,5)-coef6*pow(t_end,5)*pow(dt,4));

    return d_pos;
}

double trajectory_planner::deltaPolynomialVelocity(double dt,double t_end,double p){

    // 8th order polynomial velocity as in [1] Eq. (2)

    double d_vel,coef1,coef2,coef3,coef4,coef5,coef6,coef7,coef8;

    coef1 = 8;coef2 = 2;coef3 = 19;coef4 = 24;
    coef5 = 5;coef6 = 8;coef7 = 4;coef8 = 24;

    coef1 = 1/coef1;coef2 = 1/coef2;coef3 = coef3/coef4;
    coef4 = coef5/coef6;coef5 = 1/coef7;coef6 = 1/coef8;

    d_vel = -p*(coef1*pow(dt,8)-coef2*t_end*pow(dt,7)+coef3*pow(t_end,2)*pow(dt,6)-coef4*pow(t_end,3)*pow(dt,5)+coef5*pow(t_end,4)*pow(dt,4)-coef6*pow(t_end,5)*pow(dt,3));

    return d_vel;
}

double trajectory_planner::deltaSpace(){

    return sqrt( pow(xf_-xi_,2)+pow(yf_-yi_,2)+pow(zf_-zi_,2) );
}


// Get waypoints
double trajectory_planner::Xi()
{
  return xi_;
}
double trajectory_planner::Yi()
{
  return yi_;
}
double trajectory_planner::Zi()
{
  return zi_;
}
double trajectory_planner::Xf()
{
  return xf_;
}
double trajectory_planner::Yf()
{
  return yf_;
}
double trajectory_planner::Zf()
{
  return zf_;
}

// Set waypoints
void trajectory_planner::SetXYZi(double x, double y, double z)
{
  xi_ = x;
  yi_ = y;
  zi_ = z;
}
void trajectory_planner::SetXYZf(double x, double y, double z)
{
  xf_ = x;
  yf_ = y;
  zf_ = z;
}

} /* planner */
