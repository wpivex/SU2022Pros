#include "Subsystems/RobotBuilder.h"
#include "Subsystems/Localizer/Odometry.h"
#include "Subsystems/Flywheel/TBHFlywheel.h"

Robot getRobot() {

    Robot robot;

    robot.drive.reset(new Drive(
        {-11, 12, -14, 15}, // left motor ports
        {17, 18, -19, 20}, // right motor ports
        pros::E_MOTOR_GEAR_600, // internal gear ratio
        4.0/3.0, // external gear ratio
        2.75, // wheel diameter in inches
        10 // track width in inches
    ));

    robot.localizer.reset(new Odometry(
        7, // imu port
        {'E','F'}, // left encoder port
        {'C', 'D'}, // right encoder port
        {'A', 'B'}, // back encoder port
        1.625, // odom wheel diameter in inches
        5.5 // distance from center of rotation to back encoder in inches
    ));

    robot.flywheel.reset(new TBHFlywheel(
        {1, -2}, // ports
        0.0002, // tbh constant
        { // volt to rpm data
            {1615, 5},
            {1966, 6},
            {2306, 7},
            {2646, 8},
            {3054, 9},
            {3416, 10},
            {3751, 11},
            {4141, 12}
        },
        0 // start speed
    ));

    robot.intake.reset(new pros::MotorGroup({-13, 16}));

    robot.indexer.reset(new pros::ADIDigitalOut('G'));

    return robot;

}