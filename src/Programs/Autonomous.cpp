#include "Subsystems/RobotBuilder.h"
#include "Programs/Driver.h"
#include "AutonomousFunctions/DriveFunctions.h"
#include "Algorithms/SingleBoundedPID.h"
#include "Algorithms/SimplePID.h"
#include "Algorithms/DoubleBoundedPID.h"
#include "Algorithms/Alternator.h"
#include "misc/MathUtility.h"
#include "misc/ProsUtility.h"
#include "pros/llemu.hpp"

#define GFU_DIST(maxSpeed) SingleBoundedPID({0.1, 0, 0, 0.1, maxSpeed})
#define GFU_DIST_PRECISE(maxSpeed) DoubleBoundedPID({0.1, 0, 0, 0.1, maxSpeed}, 0.2, 3)
#define GFU_TURN SimplePID({1, 0, 0.1, 0.0, 1})
#define GTU_TURN DoubleBoundedPID({1, 0, 0.1}, getRadians(1.5), 3)
#define GTU_TURN_PRECISE DoubleBoundedPID({0.8, 0, 0.1}, getRadians(0.75), 5)
#define GCU_CURVE SimplePID({1.7, 0, 0})


void startIntake(Robot& robot) {
    pros::delay(300);
    setEffort(*robot.intake, 1);
}

// shoot a 3-burst round. First two rounds are short burst (110ms with 220ms break), third is longer (300ms)
void shoot(Robot& robot) {

    // cock gun
    setEffort(*robot.intake, 1);
    robot.indexer->set_value(true);
    pros::delay(500);

    // wait for spinup
    if (!robot.flywheel->atTargetVelocity()) {
        setEffort(*robot.intake, 0);
        while (!robot.flywheel->atTargetVelocity()) pros::delay(10);
    }

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
    robot.indexer->set_value(false);
}

void testCurve(Robot& robot) {
    robot.flywheel->setVelocity(0);
    goCurveU(robot, GFU_DIST_PRECISE(0.5), GCU_CURVE, getRadians(0), getRadians(90), 24);
    goCurveU(robot, GFU_DIST_PRECISE(0.5), GCU_CURVE, getRadians(90), getRadians(0), 24);
    goCurveU(robot, GFU_DIST_PRECISE(0.5), GCU_CURVE, getRadians(0), getRadians(90), -24);
    goCurveU(robot, GFU_DIST_PRECISE(0.5), GCU_CURVE, getRadians(90), getRadians(0), -24);
}

void matchAutonIMUOnly(Robot& robot) {

	
	robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    robot.localizer->setPosition(0, 0);
	robot.flywheel->setVelocity(3250);

    double startTheta = getRadians(342);

    // initial goal rush
    pros::Task([&] { startIntake(robot); }); // start intake 300ms after goForwardU
	goForwardU(robot, GFU_DIST_PRECISE(0.4), GFU_TURN, 14, startTheta);

    pros::delay(1000);

    // shoot
    shoot(robot);
    robot.flywheel->setVelocity(3350); // next shot is a bit further away

    // Get to roller
    goCurveU(robot, GFU_DIST_PRECISE(0.5), GCU_CURVE, startTheta, getRadians(67), -15);
    goTurnU(robot, GTU_TURN, 0);
    goForwardTimedU(robot, GFU_TURN, 1, -0.5, 0);

    // Collect disc at (1,1)
    setEffort(*robot.intake, 1);
    goCurveU(robot, GFU_DIST_PRECISE(0.5), GCU_CURVE, 0, getRadians(-55), 14);
    pros::delay(500);

    // Collect disc at (0.5, 0.5)
    goTurnU(robot, GTU_TURN, getRadians(-115));
    goForwardU(robot, GFU_DIST_PRECISE(0.8), GFU_TURN, 16, getRadians(-115));
    pros::delay(500);

    // Shoot two discs
    goTurnU(robot, GTU_TURN_PRECISE, 0);
    shoot(robot);

    // Collect preloads
    goTurnU(robot, GTU_TURN, getRadians(80));
    setEffort(*robot.intake, 1);
    goForwardU(robot, GFU_DIST_PRECISE(0.8), GFU_TURN, 24, getRadians(80));

    // Shoot preloads
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(-20));
    shoot(robot);




    // Do roller [TODO]



    

	pros::lcd::print(0, "done");

}

void skillsAutonIMUOnly(Robot& robot) {

}