#include "main.h"

#include "Subsystems/RobotBuilder.h"
#include "Programs/Driver.h"
#include "Programs/Autonomous.h"


Robot robot = getRobot15();
Driver driver(robot, TANK_DRIVE);

#define RUN_AUTON

using namespace pros;

bool centerButtonReady = false;

void ready() {
    centerButtonReady = true;
}

void initialize() {

    pros::lcd::initialize();
    pros::lcd::register_btn1_cb (ready);

	
	if (robot.localizer) robot.localizer->init();
    robot.localizer->setHeading(getRadians(0));
	pros::lcd::print(0, "initialized.");

    while (!centerButtonReady) pros::delay(10);

}

void disabled() {}


void competition_initialize() {}


void autonomous() {

	matchAutonIMUOnly(robot);

}


void opcontrol() {

	
    if (true && robot.flywheel) {
        pros::Task taskFlywheel([&] {
            robot.flywheel->maintainVelocityTask();
        });
    }

    if (false && robot.localizer) {
        pros::Task taskLocalizer([&] {
            robot.localizer->updatePositionTask();
        });
    }

	#ifdef RUN_AUTON
	autonomous();
	return;
	#endif

	driver.runDriver();
	
}
