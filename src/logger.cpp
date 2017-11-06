


#include "robot.h"
#include "logger.h"

#include <iomanip>


namespace visualization
{

// Dispaly the position of the flying machine.
void display(machine::robot& flying_machine, double t_current)
{

    // Display robot's status and more
    std::cout << std::setprecision(3) << "x  : " << flying_machine.x()
                         << " y : " << flying_machine.y()
                         << " z : " <<flying_machine.z();
    std::cout << std::setprecision(3) << " | abs_vel: " << flying_machine.absVelocity()
                         <<" | time : " << t_current << "\n";

    return;
}

} /* visualization */
