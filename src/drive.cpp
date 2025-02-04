#include "drive.hpp"
#include <vector>
#include "helper.hpp"
#include "robot.hpp"
#include <stdint.h>

// Define stages for the "lady" motor
std::vector<double> stages = {0, -60, -300}; // Rotation values for each stage
uint8_t currentStage = 0;                   // Start at stage 0
bool toggleL1 = false;                      // State for L1 toggle
bool toggleL2 = false;                      // State for L2 toggle


void move_to_stage(uint8_t targetStage) {
    if (targetStage >= stages.size()) {
        return; // Prevent invalid stage access
    }

    double targetRotation = stages[targetStage];
    double currentRotation = robo_g.rotation.position(vex::rotationUnits::deg);

    // Correct direction since there's no gear inversion
    double direction = targetRotation > currentRotation ? 1 : -1;
    double velocity = direction * 50; // Adjust speed as needed

    // Move the motor to the target stage
    robo_g.lady[0].spinToPosition(targetRotation, vex::rotationUnits::deg, velocity, vex::velocityUnits::pct, true);

    currentStage = targetStage; // Update current stage
    if (currentStage == 0)
    {
        robo_g.rotation.resetPosition(); // Reset the rotation sensor when reaching stage 0
    }
    robo_g.screen.clearScreen();
}


[[noreturn]] void drive(void)
{
    robo_g.screen.setCursor(0, 0);
    robo_g.screen.print("uno dos");
    robo_g.ctrl.ButtonB.pressed([] { // toggle mogo
        robo_g.mogo.set(robo_g.mogo.value() ^ 1);
    });

    robo_g.ctrl.ButtonA.pressed([] { // toggle dinker
        robo_g.dink.set(robo_g.dink.value() ^ 1);
    });

    robo_g.ctrl.ButtonY.pressed([] { // toggle intake piston
        robo_g.intkp.set(robo_g.intkp.value() ^ 1);
    });

    // Brake mode for lady motor
    robo_g.lady[0].setBrake(vex::brakeType::hold);

    // Button handlers for stage movement
    robo_g.ctrl.ButtonL1.pressed([]
        {
            if (toggleL1) {
                if (currentStage == 0) {
                    move_to_stage(1);
                } else if (currentStage == 2) {
                    move_to_stage(1);
                } else if (currentStage == 1){
                    move_to_stage(2);
                }
            } 
            toggleL1 = !toggleL1; // Toggle the state
        });

    robo_g.ctrl.ButtonL2.pressed([]
                                 {
                                     if (toggleL2)
                                     {
                                         move_to_stage(0); // Move to stage 0
                                     }
                                     toggleL2 = !toggleL2; // Toggle the state
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

        bool isR1Pressed = robo_g.ctrl.ButtonR2.pressing();
        bool isR2Pressed = robo_g.ctrl.ButtonR1.pressing();

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


        vex::task::sleep(20);
    }
}
