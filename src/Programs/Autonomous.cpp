#include "Subsystems/RobotBuilder.h"
#include "Programs/Driver.h"
#include "AutonomousFunctions/DriveFunctions.h"
#include "Algorithms/SingleBoundedPID.h"
#include "Algorithms/SimplePID.h"
#include "Algorithms/DoubleBoundedPID.h"
#include "Algorithms/NoPID.h"
#include "Algorithms/Alternator.h"
#include "misc/MathUtility.h"
#include "misc/ProsUtility.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

#define GFU_DIST(maxSpeed) SingleBoundedPID({0.1, 0, 0, 0.1, maxSpeed})
#define GFU_DIST_PRECISE(maxSpeed) DoubleBoundedPID({0.1, 0, 0, 0.1, maxSpeed}, 0.2, 3)
#define GFU_TURN SimplePID({1, 0, 0.1, 0.0, 1})
#define GTU_TURN DoubleBoundedPID({1.25, 0, 0.1, 0.12, 1}, getRadians(1.5), 3)
#define GTU_TURN_PRECISE DoubleBoundedPID({1.25, 0, 0.1, 0.12, 1}, getRadians(0.75), 3)
#define GCU_CURVE SimplePID({1.7, 0, 0})

// don't stop motors at end
#define NO_SLOWDOWN(maxSpeed) NoPID(maxSpeed)

void startIntake(Robot& robot) {
    pros::delay(300);
    setEffort(*robot.intake, 1);
}

void delayResetIndexer(Robot& robot) {
    pros::delay(500);
    robot.indexer->set_value(false);
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

        constexpr uint32_t TIMEOUT_MS = 5000; // maximum time to wait for spinup to proper velocity

        setEffort(*robot.intake, 0);
        uint32_t start = pros::millis();
        while (!robot.flywheel->atTargetVelocity() && pros::millis() - start < TIMEOUT_MS) {
            pros::delay(10);
        }
    }

    // first shot
    setEffort(*robot.intake, -1);
    pros::delay(110);
    robot.intake->brake();
    robot.flywheel->setVelocity(3300);
    pros::delay(280);

    // second shot
    setEffort(*robot.intake, -1);
    pros::delay(110);
    robot.intake->brake();
    pros::delay(280);

    // third shot (longer)
    setEffort(*robot.intake, -1);
    pros::delay(300);
    robot.intake->brake();

    // reset indexer after 500ms, nonblocking
    pros::Task([&] {delayResetIndexer(robot); });
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

    // get disc at (1,1)
    goTurnU(robot, GTU_TURN, getRadians(10));
    goForwardU(robot, GFU_DIST(1), GFU_TURN, -12, getRadians(10));
    goTurnU(robot, GTU_TURN, getRadians(-45));
    goForwardU(robot, GFU_DIST(1), GFU_TURN, 6, getRadians(-45));
    pros::delay(500);

    // get disc at (0.5,0,5)
    goTurnU(robot, GTU_TURN, getRadians(0));
    goForwardU(robot, GFU_DIST(1), GFU_TURN, -6, getRadians(0));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(-90));
    goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 12, getRadians(-90));
    pros::delay(500);

    // Shoot disc and align to roller
    goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, -12, getRadians(-90));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(-2));
    shoot(robot);

    // back up till pressed with roller
    goForwardTimedU(robot, GFU_TURN, 1, -0.5, 0);

    // Collect preloads
    goCurveU(robot, GFU_DIST(0.5), GCU_CURVE, getRadians(0), getRadians(90), 16);
    pros::delay(500);

    // Shoot preloads
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(-10));
    shoot(robot);

    // Get other triple stack
    goTurnU(robot, GTU_TURN, getRadians(55));
    goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 8, getRadians(55));
    pros::delay(500);

    // Shoot triple stack
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(-15));
    shoot(robot);
    

	pros::lcd::print(0, "Final theta: %f", getDegrees(robot.localizer->getHeading()));

}

void skillsAutonIMUOnly(Robot& robot) {

}