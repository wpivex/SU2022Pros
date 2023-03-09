#include "Subsystems/RobotBuilder.h"
#include "Programs/Driver.h"
#include "AutonomousFunctions/DriveFunctions.h"
#include "Algorithms/SingleBoundedPID.h"
#include "Algorithms/SimplePID.h"
#include "Algorithms/DoubleBoundedPID.h"
#include "Algorithms/NoPID.h"
#include "Algorithms/Alternator.h"
#include "Algorithms/Shooter.h"
#include "Algorithms/ConversionData.h"
#include "misc/MathUtility.h"
#include "misc/ProsUtility.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

#define GFU_DIST(maxSpeed) SingleBoundedPID({0.1, 0, 0, 0.12, maxSpeed})

#define GFU_DIST_PRECISE(maxSpeed) DoubleBoundedPID({0.123, 0, 0.027, 0.12, clamp(maxSpeed,-0.8,0.8), 0.03}, 0.075, 3)

#define GFU_TURN SimplePID({1, 1.5, 0, 0.0, 1})
#define GTU_TURN DoubleBoundedPID({1.25, 0.00, 0.095, 0.15, 1}, getRadians(1.5), 1)

#define GTU_TURN_PRECISE DoubleBoundedPID({1.25, 0.005, 0.13, 0.17, 1}, getRadians(0.5), 3)

#define GCU_CURVE SimplePID({2.5/*2.25*//*1.7*/, 0, 0})

#define NO_CORRECTION SimplePID({0,0,0})

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


// blocking function to move rollers some degrees
// speed between -1 to 1
void moveRollerDegrees(Robot& robot, double degrees, double speed) {

    double startPosition = robot.roller->get_position();
    robot.roller->move_relative(degrees, speed * 100);
    while (fabs(robot.roller->get_position() - startPosition) < 5) {
        pros::delay(10);
    }
}

// blocking function to move rollers for some time
void moveRollerTime(Robot& robot, int timeMs, double speed) {
    robot.roller->move_velocity(speed * 100);
    double startTime = pros::millis();
    while (pros::millis() - startTime < timeMs) {
        pros::delay(10);
    }
    robot.roller->brake();
}

void setShootDistance(Robot& robot, double distanceToGoal, double rpmCorrection, bool flapUp) {
    double rpm;
    if (flapUp) rpm = voltToRpm(robot.flywheel->rpmDistanceUp, distanceToGoal);
    else rpm = voltToRpm(robot.flywheel->rpmDistanceDown, distanceToGoal);
    robot.flywheel->setVelocity(rpm + rpmCorrection);
}

// shoot a 3-burst round. First two rounds are short burst (110ms with 220ms break), third is longer (300ms)
void shoot(Robot& robot) {

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

    uint32_t start = pros::millis();
    Shooter shooter;

    while (pros::millis() - start < 4000) {
        setEffort(*robot.intake, shooter.tickIntakeShootingSpeed(robot));
        pros::delay(50);
    }

    // reset indexer after 500ms, nonblocking
    pros::Task([&] {delayResetIndexer(robot); });
}

void threeTileAuton(Robot& robot) {

    #include "ThreeTileAuton.txt"
    
}

void twoTileAuton(Robot& robot) {// GENERATED C++ CODE FROM PathGen 3.4.3

    #include "TwoTileAuton.txt"

}

void threeTileSkills(Robot& robot) {

    #include "ThreeTileSkills.txt"
    
}

void twoTileSkills(Robot& robot) {// GENERATED C++ CODE FROM PathGen 3.4.3

    #include "TwoTileSkills.txt"

}

void testAuton(Robot& robot) {
    
    #include "TestAuton.txt"
    
    
}