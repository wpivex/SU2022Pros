#include "Subsystems/RobotBuilder.h"
#include "Programs/Driver.h"
#include "AutonomousFunctions/DriveFunctions.h"
#include "Algorithms/SingleBoundedPID.h"
#include "Algorithms/SimplePID.h"
#include "Algorithms/DoubleBoundedPID.h"
#include "Algorithms/NoPID.h"
#include "Algorithms/Alternator.h"
#include "Algorithms/Shooter.h"
#include "misc/MathUtility.h"
#include "misc/ProsUtility.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

#define GFU_DIST(maxSpeed) SingleBoundedPID({0.1, 0, 0, 0.12, maxSpeed})
#define GFU_DIST_PRECISE(maxSpeed) DoubleBoundedPID({0.1, 0, 0, 0.12, maxSpeed}, 0.1, 3)
#define GFU_TURN SimplePID({1, 0, 0.1, 0.0, 1})
#define GTU_TURN DoubleBoundedPID({1.25, 0, 0.095, 0.15, 1}, getRadians(1.5), 3)
#define GTU_TURN_PRECISE DoubleBoundedPID({1.25, 0, 0.095, 0.15, 1}, getRadians(0.75), 3)
#define GCU_CURVE SimplePID({2.5/*2.25*//*1.7*/, 0, 0})

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
    while (pros::millis() - start < 6000) {
        setEffort(*robot.intake, shooter.tickIntakeShootingSpeed(robot));
    }

    // reset indexer after 500ms, nonblocking
    pros::Task([&] {delayResetIndexer(robot); });
}

void threeTileAuton(Robot& robot) {
    
    // GENERATED C++ CODE FROM PathGen 3.4.3
    // Exported: Sun Jan 15 19:15:33 2023

    // Robot assumes a starting position of (18.5,91.2) at heading of 0.0 degrees.
    robot.flywheel->setVelocity(3115); // Preemptively set speed for next shot
    setEffort(*robot.intake, 1); // Start running intake immediately
    robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    goTurnU(robot, GTU_TURN_PRECISE, getRadians(44.72));
    goForwardU(robot, NO_SLOWDOWN(0.8), GFU_TURN, 8.41, getRadians(44.72));
    goForwardU(robot, GFU_DIST_PRECISE(0.48), GFU_TURN, 8.44, getRadians(44.72));
    pros::delay(1000);
    goForwardU(robot, GFU_DIST_PRECISE(0.52), GFU_TURN, -4.23, getRadians(404.71));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(314.65));
    goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, -19.71, getRadians(314.65));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(360.01));
    goForwardU(robot, GFU_DIST_PRECISE(0.48), GFU_TURN, -3.48, getRadians(360.01));
    goForwardU(robot, GFU_DIST_PRECISE(0.55), GFU_TURN, 3.62, getRadians(359.99));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(6.69));

    shoot(robot);
    robot.flywheel->setVelocity(3018); // Preemptively set speed for next shot

    goTurnU(robot, GTU_TURN_PRECISE, getRadians(310.5));
    setEffort(*robot.intake, 1);
    goForwardU(robot, NO_SLOWDOWN(0.77), GFU_TURN, 21.52, getRadians(310.5));
    goForwardU(robot, GFU_DIST_PRECISE(0.49), GFU_TURN, 10.24, getRadians(310.5));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(23.5));

    shoot(robot);
    robot.flywheel->setVelocity(3018); // Preemptively set speed for next shot

    setEffort(*robot.intake, 1);
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(23.71));
    goForwardU(robot, GFU_DIST_PRECISE(0.45), GFU_TURN, 6.19, getRadians(23.71));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(396.28));
    goForwardU(robot, GFU_DIST_PRECISE(0.55), GFU_TURN, -6.14, getRadians(396.28));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(213.89));
    setEffort(*robot.intake, 1);
    goForwardU(robot, GFU_DIST_PRECISE(0.77), GFU_TURN, 13.65, getRadians(213.89));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(25.16));

    shoot(robot);
    robot.flywheel->setVelocity(3018); // Preemptively set speed for next shot

    goTurnU(robot, GTU_TURN_PRECISE, getRadians(276.17));
    goForwardU(robot, GFU_DIST_PRECISE(0.82), GFU_TURN, 21.81, getRadians(276.17));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(215.88));
    goForwardU(robot, GFU_DIST_PRECISE(0.8), GFU_TURN, -11.33, getRadians(215.88));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(271.55));
    setEffort(*robot.intake, 1);
    goForwardU(robot, GFU_DIST_PRECISE(0.58), GFU_TURN, 7.23, getRadians(271.55));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(222.34));
    goForwardU(robot, GFU_DIST_PRECISE(0.62), GFU_TURN, -11.89, getRadians(222.34));
    setEffort(*robot.intake, 1);
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(271.43));
    goForwardU(robot, GFU_DIST_PRECISE(0.55), GFU_TURN, 7.81, getRadians(271.43));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(36.79));

    shoot(robot);

    // ================================================

}

void twoTileAuton(Robot& robot) {// GENERATED C++ CODE FROM PathGen 3.4.3
// GENERATED C++ CODE FROM PathGen 3.4.3

// Robot assumes a starting position of (88.0,15.9) at heading of 42.48 degrees.
// GENERATED C++ CODE FROM PathGen 3.4.3

// Robot assumes a starting position of (88.0,15.9) at heading of 42.48 degrees.
robot.flywheel->setVelocity(3148); // Preemptively set speed for next shot
setEffort(*robot.intake, 1); // Start running intake immediately
robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

goForwardU(robot, GFU_DIST_PRECISE(0.51), GFU_TURN, 20.68, getRadians(42.48));
pros::delay(1000);
goTurnU(robot, GTU_TURN_PRECISE, getRadians(485.77));
goForwardU(robot, GFU_DIST_PRECISE(0.51), GFU_TURN, -18.01, getRadians(485.77));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(151.49));
goForwardU(robot, GFU_DIST_PRECISE(0.52), GFU_TURN, 19.8, getRadians(151.49));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(72.16));

shoot(robot);
robot.flywheel->setVelocity(3000); // Preemptively set speed for next shot

goTurnU(robot, GTU_TURN_PRECISE, getRadians(133.26));
goForwardU(robot, GFU_DIST_PRECISE(0.51), GFU_TURN, 44.84, getRadians(133.26));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(48.36));

shoot(robot);
robot.flywheel->setVelocity(3000); // Preemptively set speed for next shot

goTurnU(robot, GTU_TURN_PRECISE, getRadians(356.35));
goForwardU(robot, GFU_DIST_PRECISE(0.52), GFU_TURN, 10.12, getRadians(356.35));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(483.32));
goForwardU(robot, GFU_DIST_PRECISE(0.49), GFU_TURN, -18.77, getRadians(483.32));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(37.88));
goForwardU(robot, GFU_DIST_PRECISE(0.51), GFU_TURN, 4.9, getRadians(37.88));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(64.42));

shoot(robot);
robot.flywheel->setVelocity(3000); // Preemptively set speed for next shot

goTurnU(robot, GTU_TURN_PRECISE, getRadians(240.85));
goForwardU(robot, GFU_DIST_PRECISE(0.52), GFU_TURN, 25.58, getRadians(240.85));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, -24.92, getRadians(240.84));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(64.4));

shoot(robot);
robot.flywheel->setVelocity(3300); // Preemptively set speed for next shot

goTurnU(robot, GTU_TURN_PRECISE, getRadians(207.8));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 28.88, getRadians(207.8));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(241.69));
goForwardU(robot, GFU_DIST_PRECISE(0.52), GFU_TURN, -10.6, getRadians(241.69));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(184.18));
goForwardU(robot, GFU_DIST_PRECISE(0.52), GFU_TURN, 8.83, getRadians(184.18));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(224.28));
goForwardU(robot, GFU_DIST_PRECISE(0.52), GFU_TURN, -12.0, getRadians(224.28));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(186.0));
goForwardU(robot, GFU_DIST_PRECISE(0.52), GFU_TURN, 8.21, getRadians(186.0));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(57.22));
goForwardU(robot, GFU_DIST_PRECISE(0.51), GFU_TURN, 15.08, getRadians(57.22));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(49.19));

shoot(robot);

// ================================================
}
