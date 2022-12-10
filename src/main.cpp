#include "main.h"

#include "Subsystems/RobotBuilder.h"
#include "Programs/Driver.h"
#include "AutonomousFunctions/DriveFunctions.h"
#include "Algorithms/SingleBoundedPID.h"
#include "Algorithms/SimplePID.h"
#include "Algorithms/DoubleBoundedPID.h"

Robot robot = getRobot();
Driver driver(robot, TANK_DRIVE);

#define RUN_AUTON

using namespace pros;

void initialize() {
	pros::lcd::initialize();
	if (robot.localizer) robot.localizer->init();
	pros::lcd::print(0, "initialized");
}

void disabled() {}


void competition_initialize() {}


void autonomous() {
	#define GFU_DIST(maxSpeed) SingleBoundedPID({0.1, 0, 0, 0.1, maxSpeed})
	#define GFU_DIST_PRECISE(maxSpeed) DoubleBoundedPID({0.1, 0, 0, 0.1, maxSpeed}, 0.2, 3)
	#define GFU_TURN SimplePID({1, 0, 0.1, 0.0, 1})
	#define GTU_TURN DoubleBoundedPID({1, 0, 0.1}, getRadians(1.5), 3)

	robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

	//goForwardU(robot, GFU_DIST_PRECISE(0.8), GFU_TURN, 24);

	goTurnU(robot, GTU_TURN, getRadians(180));

	pros::lcd::print(0, "done");

}


void opcontrol() {

	
    if (false && robot.flywheel) {
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
