#include "drive.hpp"

#include "helper.hpp"
#include "robot.hpp"


[[noreturn]] void drive(void)
{

    robo_g.pnch[0].setBrake(vex::brakeType::brake);
    robo_g.ctrl.ButtonB.pressed([] { // toggle mogo
        robo_g.mogo.set(robo_g.mogo.value() ^ 1);
    });

    robo_g.ctrl.ButtonX.pressed([] { // toggle dink
        robo_g.dink.set(robo_g.dink.value() ^ 1);
    });

    while (true)
    {
        double fwd, side;
        double left, right;

        fwd = robo_g.ctrl.Axis3.position() * (cfg.rev ? -SPEED : SPEED);
        side = robo_g.ctrl.Axis1.position() * SPEED;

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