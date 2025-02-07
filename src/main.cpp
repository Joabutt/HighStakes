#include "auton.hpp"
#include "drive.hpp"
#include "robot.hpp"

int main(void)
{
    vex::competition comp = vex::competition();
    comp.autonomous(auton);
    comp.drivercontrol(auton);
    
    return EXIT_SUCCESS;
}
