#include "drive.hpp"

#include "helper.hpp"
#include "robot.hpp"

// Function to update the robot's coordinates
/*
void updateCoordinates() {
    // Call the function to update odometry and coordinates
    updateOdometry(); // Ensure this updates the robotPosition
}
*/

[[noreturn]] void drive(void)
{

    robo_g.pnch[0].setBrake(vex::brakeType::brake);
    /*    robo_g.ctrl.ButtonL1.pressed([] { // toggle reverse mode
            robo_g.screen.setCursor(0, 0);
            robo_g.screen.print((cfg.rev ^= 1) ? "rev" : "   ");
        });
    */

    /*    robo_g.ctrl.ButtonA.pressed([] { // toggle brake mode
            vex::brakeType b;

            cfg.brk ^= 1;
            b = cfg.brk ? vex::brakeType::brake : vex::brakeType::coast;
            for (uint8_t i = 0; i < MOTORS_BASE; i++)
                robo_g.base[i].setBrake(b);

            robo_g.screen.setCursor(0, 5);
            robo_g.screen.print(cfg.brk ? "brk" : "   ");
        });
    */
    robo_g.ctrl.ButtonB.pressed([] { // toggle wings
        robo_g.wing.set(robo_g.wing.value() ^ 1);
    });

    robo_g.ctrl.ButtonX.pressed([] { // toggle hang
        robo_g.hang.set(robo_g.hang.value() ^ 1);
    });

    while (true)
    {
        double fwd, side;
        double left, right;

        // the messy calculation allows branchless programming
        // reverses the controls if dir is set
        fwd = robo_g.ctrl.Axis3.position() * (cfg.rev ? -SPEED : SPEED);
        side = robo_g.ctrl.Axis1.position() * SPEED;

        /*
                count++;
                if (count > 100) {
                    vex::brain().Screen.setCursor(200, 200);
                    vex::brain().Screen.clearLine(200);
                    vex::brain().Screen.print("rotation: %fº", robo_g.gyro.rotation());
                    count = 0;
                }
                */

        // Update the robot's coordinates
        // updateCoordinates();

        left = fwd + side;
        right = fwd - side;

        for (uint8_t i = 0; i < MOTORS_BASE; i++)
            robo_g.base[i].spin(vex::directionType::fwd,
                                i < drive_motors::RF ? left : right,
                                vex::velocityUnits::pct);

        bool isR1Pressed = robo_g.ctrl.ButtonR1.pressing();
        bool isR2Pressed = robo_g.ctrl.ButtonR2.pressing();

        if (isR2Pressed)
        {
            double rollerVel = 100; 
            for (uint8_t i = 0; i < MOTORS_INTK; i++)
                robo_g.intk[i].spin(vex::directionType::rev, rollerVel, vex::percentUnits::pct);
        }
        else if (isR1Pressed)
        {
            double rollerVel = 100; 
            for (uint8_t i = 0; i < MOTORS_INTK; i++)
                robo_g.intk[i].spin(vex::directionType::fwd, rollerVel, vex::percentUnits::pct);
        }
        else
        {
            for (uint8_t i = 0; i < MOTORS_INTK; i++)
                robo_g.intk[i].stop(); // Stop roller motor when neither ButtonR1 nor ButtonR2 is pressed
        }
        /*bool p = robo_g.ctrl.ButtonL2.pressing();
        double vel = p * 100;
        for (uint8_t i = 0; i < MOTORS_PNCH; i++)
            robo_g.pnch[i].spin(vex::directionType::fwd, vel,
                                vex::velocityUnits::pct);
        */

        // Control pnch motor based on ButtonUp and ButtonDown
        bool down = robo_g.ctrl.ButtonL1.pressing();
        bool up = robo_g.ctrl.ButtonL2.pressing();

        double vel = (up ? 100 : (down ? -100 : 0)); // Clockwise if ButtonUp, counterclockwise if ButtonDown
        if (vel != 0)
        {
            robo_g.pnch[0].spin(vex::directionType::fwd, vel, vex::velocityUnits::pct);
        }
        else
        {
            robo_g.pnch[0].stop(); // Stops the motor and holds position due to the hold brake mode
        }

        vex::task::sleep(20);
    }
}