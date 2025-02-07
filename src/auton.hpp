#ifndef AUTON_HPP
#define AUTON_HPP

/**
 * Moves the robot by a fixed distance, spinning each side at specified speeds.
 *
 * @param degree    number of degrees to rotate the faster side. A negative
 * degree would negate the velocities for each side.
 * @param left      velocity of motors on the left side.
 * @param right     velocity of motors on the right side.
 * @param invert    whether left and right should be inverted.
 */
void move(double degree, double left, double right, bool invert = false);
void turn_until(double degree, double leftSpeed, double rightSpeed, bool invert, double calibrationFactor = 1.05);
void lady(double stage);
void auton(void);
void skill(void);


#endif