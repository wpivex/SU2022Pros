#include "Subsystems/RobotBuilder.h"
#include "Programs/Driver.h"
#include "AutonomousFunctions/DriveFunctions.h"
#include "Algorithms/SingleBoundedPID.h"
#include "Algorithms/SimplePID.h"
#include "Algorithms/DoubleBoundedPID.h"
#include "Algorithms/Alternator.h"
#include "misc/ProsUtility.h"
#include "pros/llemu.hpp"

#define GFU_DIST(maxSpeed) SingleBoundedPID({0.1, 0, 0, 0.1, maxSpeed})
#define GFU_DIST_PRECISE(maxSpeed) DoubleBoundedPID({0.1, 0, 0, 0.1, maxSpeed}, 0.2, 3)
#define GFU_TURN SimplePID({1, 0, 0.1, 0.0, 1})
#define GTU_TURN DoubleBoundedPID({1, 0, 0.1}, getRadians(1.5), 3)
#define GCU_CURVE SimplePID({1, 0, 0})


void startIntake(Robot& robot) {
    pros::delay(300);
    setEffort(*robot.intake, 1);
}

// shoot a 3-burst round. First two rounds are short burst (110ms with 220ms break), third is longer (300ms)
void shoot(Robot& robot) {

    setEffort(*robot.intake, -1);
    pros::delay(110);
    robot.intake->brake();
    robot.flywheel->setVelocity(3300);
    pros::delay(280);

    setEffort(*robot.intake, -1);
    pros::delay(110);
    robot.intake->brake();
    pros::delay(280);

    setEffort(*robot.intake, -1);
    pros::delay(300);
    robot.intake->brake();
}

void testCurve(Robot& robot) {
    robot.flywheel->setVelocity(0);
    robot.localizer->setPosition(0, 0, getRadians(0));
    goCurveU(robot, GFU_DIST_PRECISE(0.5), GCU_CURVE, getRadians(0), getRadians(90), 24);
}

void matchAutonIMUOnly(Robot& robot) {

	
	robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    testCurve(robot);
    pros::lcd::print(1, "Done curve");
    return;

    robot.localizer->setPosition(0, 0, getRadians(334.3));
	robot.flywheel->setVelocity(3250);

    // initial goal rush
    //pros::Task([&] { startIntake(robot); }); // start intake 300ms after goForwardU
	//goForwardU(robot, GFU_DIST_PRECISE(0.4), GFU_TURN, 14);

    // testing only
    setEffort(*robot.intake, 1);
    pros::delay(500);

    pros::delay(1000);

    // cock the gun
    robot.indexer->set_value(true);
    pros::delay(500);

    // wait for spinup
    setEffort(*robot.intake, 0);
    while (!robot.flywheel->atTargetVelocity()) pros::delay(10);

    // shoot
    shoot(robot);


	//goTurnU(robot, GTU_TURN, getRadians(180));


	pros::lcd::print(0, "done");

}

void skillsAutonIMUOnly(Robot& robot) {

}