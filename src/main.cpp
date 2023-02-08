#include "main.h"

#include "Subsystems/RobotBuilder.h"
#include "Programs/Driver.h"
#include "Programs/Autonomous.h"
#include "TuneFlywheel.h"

#define IS_FIFTEEN // uncomment for 15, comment for 18
bool isSkills = false;


#ifdef IS_FIFTEEN
    #define IS_TWO_TILE
    Robot robot = getRobot15(isSkills);
    Driver driver(robot, TANK_DRIVE, 2400); 
#else
    #define IS_THREE_TILE
    Robot robot = getRobot18(isSkills);
    Driver driver(robot, TANK_DRIVE, 2400);
#endif


// #define RUN_TEST
// #define RUN_AUTON // uncomment to run auton, comment to run teleop / actual comp
//#define TUNE_FLYWHEEL // uncomment to run flywheel tuning program intsead, comment to disable this

using namespace pros;

bool centerButtonReady = false;

void ready() {
    centerButtonReady = true;
}

void initialize() {

    pros::lcd::initialize();
    pros::lcd::register_btn1_cb (ready);

    robot.shooterFlap->set_value(true); // start flap up

	
	if (robot.localizer) robot.localizer->init();
    robot.localizer->setHeading(getRadians(0));
	pros::lcd::print(0, "initialized.");

    #ifdef RUN_AUTON
    #ifndef TUNE_FLYWHEEL
    while (!centerButtonReady) {
        pros::lcd::print(1, "Heading (deg): %f", robot.localizer->getHeading() * 180 / 3.1415);
        pros::delay(10);
    }
    #endif
    #endif

}

  
void disabled() {}


void competition_initialize() {}


void autonomous() {  

    robot.shooterFlap->set_value(false); // flap down  

    if (true && robot.flywheel) {
        pros::Task taskFlywheel([&] {
            robot.flywheel->maintainVelocityTask();
        });
    }

    if (true && robot.localizer) {
        pros::Task taskOdometry([&] {
            robot.localizer->updatePositionTask();
        });
    }

    try {

        #ifdef RUN_TEST
        testAuton(robot);
        return;
        #endif

        #ifdef IS_THREE_TILE
        if (isSkills) threeTileSkills(robot);
        else threeTileAuton(robot);
        #endif

        #ifndef IS_THREE_TILE
        if (isSkills) twoTileSkills(robot);
        else twoTileAuton(robot);
        #endif

    } catch (std::runtime_error &e) {
        pros::lcd::clear();
        pros::lcd::print(0, "IMU disconnect, force shutdown.");
        robot.drive->stop();
        robot.intake->brake();
    }

    
}




void opcontrol() {

    #ifdef TUNE_FLYWHEEL
    tuneFlywheel(robot, driver.controller);
    return;
    #endif



	#ifdef RUN_AUTON
	autonomous();
	return;
	#endif


	driver.runDriver();
	
}
