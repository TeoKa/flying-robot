//
//  trajectory_planning.hpp
//  fying_machine_better
//
//  Created by Matteo Ciocca on 18/11/16.
//  Copyright © 2016 Matteo Ciocca. All rights reserved.
//

#ifndef trajectory_planning_hpp
#define trajectory_planning_hpp

#include <stdio.h>
#include <iostream>


using namespace std;

class trajectory_planner {
    
public:
    
    // Compute a single step of position, given a time step dt.
    double polynomial_position(double dt,double t_end,double p);
    
    // Compute a single step of velocity, given a time step dt.
    double polynomial_velocity(double dt,double t_end,double p);
    
    // Compute abs velocity given (vx,vy,vz).
    double abs_velocity(double vx, double vy,double vz);
    
    double delta_space_f_i(double xf, double yf,double zf,double xi,double yi,double zi);

};


#endif /* trajectory_planning_hpp */
