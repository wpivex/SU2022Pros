#include "Subsystems/RobotBuilder.h"
#include "Programs/Driver.h"
#include "AutonomousFunctions/DriveFunctions.h"
#include "Algorithms/SingleBoundedPID.h"
#include "Algorithms/SimplePID.h"
#include "Algorithms/DoubleBoundedPID.h"
#include "misc/ProsUtility.h"

#define GFU_DIST(maxSpeed) SingleBoundedPID({0.1, 0, 0, 0.1, maxSpeed})
#define GFU_DIST_PRECISE(maxSpeed) DoubleBoundedPID({0.1, 0, 0, 0.1, maxSpeed}, 0.2, 3)
#define GFU_TURN SimplePID({1, 0, 0.1, 0.0, 1})
#define GTU_TURN DoubleBoundedPID({1, 0, 0.1}, getRadians(1.5), 3)

void matchAutonIMUOnly(Robot& robot) {

	robot.localizer->setPosition(0, 0, getRadians(334.3));
	robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	robot.flywheel->setVelocity(3000);

    setEffort(*robot.intake, 1);
	goForwardU(robot, GFU_DIST_PRECISE(0.8), GFU_TURN, 14);
    pros::delay(1000);

    robot.indexer->set_value(true);
    pros::delay(500);

    // wait for spinup
    setEffort(*robot.intake, 0);
    pros::delay(2000);

    // shoot
    setEffort(*robot.intake, -0.5);
    pros::delay(2000);
    setEffort(*robot.intake, 0);


	//goTurnU(robot, GTU_TURN, getRadians(180));


	pros::lcd::print(0, "done");

}

void skillsAutonIMUOnly(Robot& robot) {

}