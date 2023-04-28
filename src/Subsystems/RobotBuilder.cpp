#include "Subsystems/RobotBuilder.h"
#include "Subsystems/Localizer/Odometry.h"
#include "Subsystems/Localizer/IMULocalizer.h"
#include "Subsystems/Localizer/Localizer.h"
#include "Subsystems/Flywheel/TBHFlywheel.h"
#include "Subsystems/Flywheel/VoltageFlywheel.h"

#include "pros/motors.h"

// flywheel
Robot getRobot15(bool isSkills) {

    Robot robot;

    // left -13, -14, 15, 17
    // right -9, 18, -20, 21

    robot.drive.reset(new Drive(
        {-13, -14, 15, 17}, // left motor ports
        {-9, 18, -20, 21}, // right motor ports
        pros::E_MOTOR_GEAR_600, // internal gear ratio
        3.0/4.0, // external gear ratio
        2.73, // wheel diameter in inches
        14.25//15.2 // track width in inches
    ));

    robot.localizer.reset(new IMULocalizer(
        *robot.drive, // reference to drive object
        1,// relabled to 1, 2, // imu port A
        3 // imu port B
    ));


    

    robot.flywheel.reset(new TBHFlywheel(
        {-4, 8}, // ports
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
        { // rpm to distance data FOR FLAP DOWN
            {2450, 56},
            {2425, 61},
            {2475, 66},
            {2550, 71},
            {2575, 76},
            {2700, 81},
            {2800, 86},
            {2800, 91},
            {2850, 96},
            {2925, 101},
            {3050, 106},
            {3187, 111},
            {3225, 116},
            {3350, 121}
            
            
        },
        { // rpm to distance data FOR FLAP UP
            {2450, 56},
            {2425, 61},
            {2475, 66},
            {2550, 71},
            {2575, 76},
            {2700, 81},
            {2800, 86},
            {2800, 91},
            {2850, 96},
            {2925, 101},
            {3050, 106},
            {3187, 111},
            {3225, 116},
            {3350, 121}
        },
        0, // start speed
        0.00005 // tbh constant
    ));

    robot.intake.reset(new pros::MotorGroup({-11, 16}));
    robot.intake->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

    robot.indexer.reset(new pros::ADIDigitalOut('A'));

    robot.roller.reset(new pros::Motor(10, pros::E_MOTOR_GEAR_100));
    robot.roller->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

    robot.shooterFlap.reset(new pros::ADIDigitalOut('H'));

    robot.endgame.reset(new pros::ADIDigitalOut('B'));

    return robot;

}

// cata
Robot getRobot18(bool isSkills) {

    
    Robot robot;

    robot.drive.reset(new Drive(
        {11, 12, -13, -14}, // left motor ports
        {1, 2, -3, -4}, // right motor ports 
        pros::E_MOTOR_GEAR_600, // internal gear ratio
        3.0/4.0, // external gear ratio
        2.74, // wheel diameter in inches
        14.25//15.2 // track width in inches
    ));

    robot.localizer.reset(new IMULocalizer(
        *robot.drive, // reference to drive object
        8, // imu port A
        9 // imu port B
    ));

    robot.cata.reset(new pros::MotorGroup({16, -17}));
    robot.cata->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

    robot.limitSwitch.reset(new pros::ADIDigitalIn('A'));

    robot.intake.reset(new pros::MotorGroup({-19, 20}));
    robot.intake->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

    robot.roller.reset(new pros::Motor(18, pros::E_MOTOR_GEAR_100));
    robot.roller->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);


    robot.endgame.reset(new pros::ADIDigitalOut('C'));

    return robot;

}