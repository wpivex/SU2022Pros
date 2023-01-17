#include "Subsystems/RobotBuilder.h"
#include "Subsystems/Localizer/Odometry.h"
#include "Subsystems/Flywheel/TBHFlywheel.h"
#include "pros/motors.h"

Robot getRobot15() {

    Robot robot;

    robot.drive.reset(new Drive(
        {-11, 12, -14, 15}, // left motor ports
        {17, 18, -19, -20}, // right motor ports
        pros::E_MOTOR_GEAR_600, // internal gear ratio
        3.0/4.0, // external gear ratio
        2.74, // wheel diameter in inches
        14.25//15.2 // track width in inches
    ));

    robot.localizer.reset(new Odometry(
        8, // imu port
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
    robot.intake->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

    robot.indexer.reset(new pros::ADIDigitalOut('G'));

    robot.roller.reset(new pros::Motor(10));

    robot.shooterFlap.reset(new pros::ADIDigitalOut('H'));

    return robot;

}

Robot getRobot18() {

    Robot robot;

    robot.drive.reset(new Drive(
        {1, 3, -4,-5}, // left motor ports
        {-16, -17, 18, 19}, // right motor ports
        pros::E_MOTOR_GEAR_600, // internal gear ratio
        3.0/4.0, // external gear ratio
        2.74, // wheel diameter in inches
        15.2 // track width in inches
    ));

    robot.localizer.reset(new Odometry(
        2, // imu port
        {'E','F'}, // left encoder port
        {'C', 'D'}, // right encoder port
        {'A', 'B'}, // back encoder port
        1.625, // odom wheel diameter in inches
        5.5 // distance from center of rotation to back encoder in inches
    ));

    robot.flywheel.reset(new TBHFlywheel(
        {17, 18}, // ports
        0.0002, // tbh constant
        { // volt to rpm data
            {1570, 5},
            {1970, 6},
            {2315, 7},
            {2650, 8},
            {3050, 9},
            {3400, 10},
            {3740, 11},
            {4130, 12}
        },
        0 // start speed
    ));

    robot.intake.reset(new pros::MotorGroup({9, 10}));
    robot.intake->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

    robot.indexer.reset(new pros::ADIDigitalOut('G'));

    return robot;

}