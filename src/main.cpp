#include "main.h"
#include "autoSelect/selection.h"
#include "okapi/api.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <any>

using namespace okapi;

/**
 * Testing ground for Okapilib
 *
 * Mostly to see if it's best to use Okapilib for Odom and chassis setup or to make your own.
 */

// auto chassis = ChassisControllerBuilder()
// 				   .withMotors({10, -9, -18}, {11, 12, -20})
// 				   //    .withGains(
// 				   // 	   {0.005, 0.03, 0.0005},  // Distance controller gains
// 				   // 	   {0.012, 0.001, 0.0001}, // Turn controller gains
// 				   // 	   {0.001, 0.001, 0.000}   // Angle controller gains (helps drive straight)
// 				   // 	   )
// 				   .withDimensions({AbstractMotor::gearset::blue}, {{3.25_in, 15_in}, imev5BlueTPR})
// 				   .withOdometry()
// 				   .buildOdometry();

auto chassis = ChassisControllerBuilder()
				   .withMotors({10, -9, -18}, {11, 12, -20})
				   .withGains(
					   {0.002, 0.001, 0.00025}, // Distance controller gains
					   {0.003, 0.001, 0.0001},	// Turn controller gains
					   {0.001, 0.001, 0.000}	// Angle controller gains (helps drive straight)
					   )
				   .withDimensions({AbstractMotor::gearset::blue, (48.0 / 36.0)}, {{3.25_in, 13_in}, imev5BlueTPR})
				   .withOdometry()
				   .buildOdometry();

Controller controller;

pros::Motor frontIntake(8);
pros::Motor backIntake(-7);

pros::ADIDigitalOut pneumatics('b');
pros::ADIDigitalOut midgoal('d');
pros::ADIDigitalOut wings('c');

pros::Imu imu(6);

void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Drivetrain (Odom)");
	selector::init();
}

long driveDistance(double distance)
{
	long double pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679;
	long double mm_to_inches = (distance / 25.4);
	long double inches_to_rotations = mm_to_inches / (3.25 * pi);
	long double rotations_to_degrees = inches_to_rotations * 360;
	return rotations_to_degrees;
}

/**
 *
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

QLength dilateDistance(QLength distance)
{

	return (distance * (4.8 / 2.0));
}

QAngle dilateRotation(QAngle rotation)
{
	return (rotation * 1.8 * (160 / 90));
}

void autonomous()
{
	if (selector::auton == 1)
	{
		chassis->getOdometry()->setState({77_in,
										  6_in,
										  0_deg});

		imu.reset();

		pros::delay(500);

		chassis->setMaxVelocity(300);
		chassis->getModel()->setBrakeMode(AbstractMotor::brakeMode::coast);

		// chassis->moveDistance(2_ft);
		// chassis->turnAngle(90_deg);
		// chassis->waitUntilSettled();
		// chassis->turnAngle(-90_deg);
		// chassis->moveDistance(-2_ft);

		chassis->moveDistance(3_ft);
		chassis->turnAngle(-90_deg);
		pneumatics.set_value(true);
		frontIntake.move(-127 * 2);
		pros::delay(300);
		chassis->setMaxVelocity(200);
		chassis->moveDistanceAsync(1.5_ft);
		pros::delay(1100);
		if (chassis->isSettled() != true)
		{
			chassis->stop();
		}

		// pros::delay(500);

		// chassis->setMaxVelocity(300);
		// chassis->getModel()->setBrakeMode(AbstractMotor::brakeMode::coast);

		// chassis->moveDistance(2_ft);
		// chassis->turnAngle(360_deg); // 90 degrees
		// chassis->waitUntilSettled();
		// chassis->moveDistance(-2_ft);
		// chassis->turnAngle(-90_deg); // 90 degrees
		// chassis->moveDistance(-2_ft);

		// chassis->moveDistance(4.8_ft); // 2 ft
		// chassis->turnAngle(160_deg); // 90 degrees

		// pros::delay(500);

		// chassis->moveDistance(dilateDistance(10_in)); // 10 in
		// chassis->turnAngle(dilateRotation(90_deg));	  // 90 degress
		// frontIntake.move(-127 * 2);
		// chassis->setMaxVelocity(200);
		// chassis->moveDistance(dilateDistance(52_in)); // 52 in
		// chassis->waitUntilSettled();
		// chassis->turnAngle(dilateRotation(-145_deg)); //-145 degrees
		// // pros::delay(550);
		// frontIntake.move(0);
		// backIntake.move(0);
		// // chassis->setMaxVelocity(250);
		// pneumatics.set_value(true);
		// chassis->moveDistance(70_in);
		// // chassis->setMaxVelocity(150);
		// chassis->turnAngle(dilateRotation(-35_deg));
		// pros::delay(100);
		// frontIntake.move(-127 * 2);
		// // chassis->moveDistance(1.5_ft);
		// chassis->moveDistanceAsync(6_ft);
		// pros::delay(1000);
		// if (chassis->isSettled() != true)
		// {
		// 	chassis->stop();
		// }

		// chassis->moveDistance(-80_in);
		// frontIntake.move(-127 * 2);
		// backIntake.move(-127 * 2);
		// pros::delay(10000);
		// ------------------------------

		// pros::delay(400);
		// chassis->moveDistance(12_in);
		// frontIntake.move(-127 * 2);
		// chassis->driveToPoint({7_ft, 5_ft}); // +X is forward, +Y is right
		// // chassis->driveToPoint({4_ft, 1_ft}); // +X is forward, +Y is right
		// chassis->waitUntilSettled();
		// frontIntake.move(0);
		// chassis->driveToPoint({9.30_ft, 1.5_ft});
		// pros::delay(300);
		// chassis->turnAngle(-40_deg);
		// pneumatics.set_value(true);
		// frontIntake.move(-127 * 2);
		// pros::delay(400);
		// chassis->setMaxVelocity(150);
		// chassis->moveDistanceAsync(1.4_ft);
		// pros::delay(1000);
		// if (chassis->isSettled() != true)
		// {
		// 	chassis->stop();
		// }
		// chassis->setMaxVelocity(150);

		// // chassis->driveToPoint({7.90_ft, -3.5_ft});

		// // chassis->driveToPoint({1_ft, 2.5_ft});

		// chassis->moveDistanceAsync(-7.5_ft);
		// pros::delay(2000);
		// if (chassis->isSettled() != true)
		// {
		// 	chassis->stop();
		// }
		// // chassis->driveToPoint({9.55_ft, 3.5_ft}, true);
		// chassis->setMaxVelocity(600);
		// // chassis->driveToPoint({9.79_ft, 4.5_ft}, true);
		// chassis->setMaxVelocity(150);
		// // chassis->setMaxVelocity(150);
		// frontIntake.move(-127 * 2);
		// backIntake.move(-127 * 2);
		// pros::delay(1500);
		// frontIntake.move(0);
		// backIntake.move(0);
		// frontIntake.move(-127 * 2);
		// backIntake.move(-127 * 2);
		// frontIntake.move(-127 * 2);
		// backIntake.move(-127 * 2);
		// frontIntake.move(-127 * 2);
		// backIntake.move(-127 * 2);
		// frontIntake.move(-127 * 2);
		// backIntake.move(-127 * 2);
		// frontIntake.move(-127 * 2);
		// backIntake.move(-127 * 2);
		// frontIntake.move(-127 * 2);
		// backIntake.move(-127 * 2);
		// frontIntake.move(-127 * 2);
		// backIntake.move(-127 * 2);
		// frontIntake.move(-127 * 2);
		// backIntake.move(-127 * 2);
		// frontIntake.move(-127 * 2);
		// backIntake.move(-127 * 2);
		// frontIntake.move(-127 * 2);
		// backIntake.move(-127 * 2);
		// pros::delay(1000);
	}
	if (selector::auton == 2)
	{
		chassis->getOdometry()->setState({77_in,
										  6_in,
										  0_deg});
		chassis->setMaxVelocity(170);
		chassis->getModel()->setBrakeMode(AbstractMotor::brakeMode::coast);
		pros::delay(300);
		chassis->moveDistance(12_in);
		frontIntake.move(-127 * 2);
		chassis->driveToPoint({7_ft, 5_ft}); // +X is forward, +Y is right
		// chassis->driveToPoint({4_ft, 1_ft}); // +X is forward, +Y is right
		chassis->waitUntilSettled();
		frontIntake.move(0);
		chassis->driveToPoint({9.35_ft, 1.5_ft});
		pros::delay(100);
		chassis->turnAngle(-45_deg);
		pneumatics.set_value(true);
		frontIntake.move(-127 * 2);
		pros::delay(400);
		chassis->setMaxVelocity(200);
		chassis->moveDistance(1.5_ft);
		chassis->setMaxVelocity(150);

		// chassis->driveToPoint({7.90_ft, -3.5_ft});
		pros::delay(1000);
		// chassis->driveToPoint({1_ft, 2.5_ft});
		chassis->waitUntilSettled();
		chassis->moveDistanceAsync(-7.5_ft);
		pros::delay(2500);
		if (chassis->isSettled() != true)
		{
			chassis->stop();
		}
		// chassis->driveToPoint({9.55_ft, 3.5_ft}, true);
		chassis->setMaxVelocity(600);
		// chassis->driveToPoint({9.79_ft, 4.5_ft}, true);
		chassis->setMaxVelocity(150);
		// chassis->setMaxVelocity(150);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		pros::delay(1500);
		frontIntake.move(0);
		backIntake.move(0);

		// chassis->moveDistance(-2_in);
		// chassis->moveDistance(2_in);
		// chassis->moveDistance(-2_in);
		// chassis->moveDistance(2_in);
		// chassis->moveDistance(-2_in);
		// chassis->moveDistance(2_in);
		// chassis->turnAngle(5_deg);
		// chassis->moveDistance(-3.5_ft);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		pros::delay(2000);
		// PART 2
		frontIntake.move(0);
		backIntake.move(0);
		chassis->setMaxVelocity(150);
		chassis->getModel()->setBrakeMode(AbstractMotor::brakeMode::coast);
		pros::delay(300);
		chassis->moveDistance(1.5_ft);
		chassis->turnAngle(-100_deg);
		pros::delay(100);
		// chassis->moveDistance(-2_ft);
		chassis->moveDistanceAsync(-2_ft);
		pros::delay(2000);
		if (chassis->isSettled() != true)
		{
			chassis->stop();
		}
		chassis->getOdometry()->setState({158_in,
										  36_in,
										  180_deg});
		chassis->driveToPoint({12_in, 36_in}); // chassis->moveDistance(12.5_ft);
											   // chassis->turnAngle(100_deg);
											   // // chassis->moveDistance(-1_ft);
											   // chassis->moveDistanceAsync(-2.5_ft);
											   // pros::delay(2000);
											   // if (chassis->isSettled() != true)
											   // {
											   // 	chassis->stop();
											   // }
											   // pneumatics.set_value(true);
											   // frontIntake.move(-127 * 2);

		// chassis->moveDistance(4.00_ft);
		// chassis->setMaxVelocity(350);
		// pros::delay(100);
		// chassis->moveDistance(0.4_ft);
		// chassis->setMaxVelocity(150);
		// pros::delay(2500);
		// frontIntake.move(0);
		// backIntake.move(0);
		// // chassis->turnAngle(3.5_deg);
		// // chassis->moveDistance(-3.3_ft);
		// chassis->moveDistanceAsync(-3.5_ft);
		// pros::delay(2000);
		// if (chassis->isSettled() != true)
		// {
		// 	chassis->stop();
		// }
		// frontIntake.move(127 * 2);
		// pros::delay(100);
		// frontIntake.move(-127 * 2);
		// backIntake.move(-127 * 2);
		// pros::delay(2500);
		// frontIntake.move(0);
		// backIntake.move(0);
		// chassis->moveDistance(1_ft);
		// pneumatics.set_value(false);
		// chassis->turnAngle(-125_deg);
		// chassis->setMaxVelocity(350);
		// chassis->moveDistance(-6_ft);
		// pros::delay(300);
		// chassis->turnAngle(-100_deg);
		// chassis->moveDistance(-2_ft);
		// pros::delay(20000);
		// {

		// 	// For SKILLS
		// 	chassis->getOdometry()->setState({77_in,
		// 									6_in,
		// 									0_deg});
		// 	chassis->setMaxVelocity(170);
		// 	chassis->getModel()->setBrakeMode(AbstractMotor::brakeMode::coast);
		// 	pros::delay(300);
		// 	chassis->moveDistance(12_in);
		// 	frontIntake.move(-127 * 2);
		// 	chassis->driveToPoint({7_ft, 5_ft}); // +X is forward, +Y is right
		// 	// chassis->driveToPoint({4_ft, 1_ft}); // +X is forward, +Y is right
		// 	chassis->waitUntilSettled();
		// 	frontIntake.move(0);
		// 	chassis->driveToPoint({9.40_ft, 1.5_ft});
		// 	pros::delay(100);
		// 	chassis->turnAngle(-45_deg);
		// 	pneumatics.set_value(true);
		// 	frontIntake.move(-127 * 2);
		// 	pros::delay(1000);
		// 	chassis->setMaxVelocity(200);
		// 	chassis->moveDistance(1.4_ft);
		// 	chassis->setMaxVelocity(150);

		// 	// chassis->driveToPoint({7.90_ft, -3.5_ft});
		// 	pros::delay(100);
		// 	// chassis->driveToPoint({1_ft, 2.5_ft});
		// 	chassis->waitUntilSettled();
		// 	chassis->driveToPoint({9.55_ft, 3.5_ft}, true);
		// 	chassis->setMaxVelocity(600);
		// 	// chassis->driveToPoint({9.79_ft, 4.5_ft}, true);
		// 	chassis->setMaxVelocity(150);
		// 	// chassis->setMaxVelocity(150);
		// 	frontIntake.move(-127 * 2);
		// 	backIntake.move(-127 * 2);
		// 	pros::delay(1500);
		// 	frontIntake.move(0);
		// 	backIntake.move(0);
		// }
	}
	if (selector::auton == 3)
	{
		chassis->getOdometry()->setState({77_in,
										  6_in,
										  0_deg});
		chassis->setMaxVelocity(150);
		chassis->getModel()->setBrakeMode(AbstractMotor::brakeMode::coast);
		pros::delay(500);
		chassis->moveDistance(12_in);
	}
	if (selector::auton == -1)
	{
		/*Code auton for Near Blue */
		chassis->getOdometry()->setState({77_in,
										  6_in,
										  0_deg});
		chassis->setMaxVelocity(170);
		chassis->getModel()->setBrakeMode(AbstractMotor::brakeMode::coast);
		pros::delay(400);
		chassis->moveDistance(12_in);
		frontIntake.move(-127 * 2);
		chassis->driveToPoint({3_ft, 5_ft}); // +X is forward, +Y is right
		// chassis->driveToPoint({4_ft, 1_ft}); // +X is forward, +Y is right
		chassis->waitUntilSettled();
		frontIntake.move(0);
		chassis->driveToPoint({0.65_ft, 1.5_ft});
		pros::delay(300);
		chassis->turnAngle(45_deg);
		pneumatics.set_value(true);
		frontIntake.move(-127 * 2);
		pros::delay(400);
		chassis->setMaxVelocity(150);
		chassis->moveDistanceAsync(1.4_ft);
		pros::delay(1000);
		if (chassis->isSettled() != true)
		{
			chassis->stop();
		}
		chassis->setMaxVelocity(150);

		// chassis->driveToPoint({7.90_ft, -3.5_ft});

		// chassis->driveToPoint({1_ft, 2.5_ft});

		chassis->moveDistanceAsync(-7.5_ft);
		pros::delay(2500);
		if (chassis->isSettled() != true)
		{
			chassis->stop();
		}
		// chassis->driveToPoint({9.55_ft, 3.5_ft}, true);
		chassis->setMaxVelocity(600);
		// chassis->driveToPoint({9.79_ft, 4.5_ft}, true);
		chassis->setMaxVelocity(150);
		// chassis->setMaxVelocity(150);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		pros::delay(1500);
		frontIntake.move(0);
		backIntake.move(0);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		pros::delay(1000);
	}
	if (selector::auton == -2)
	{
		/*Code auton for Far Blue */
	}
	if (selector::auton == -3)
	{
		/*Code auton for Blue Nothing (Leave Blank) */
	}
	if (selector::auton == 0)
	{
		chassis->getOdometry()->setState({77_in,
										  6_in,
										  0_deg});
		chassis->setMaxVelocity(170);
		chassis->getModel()->setBrakeMode(AbstractMotor::brakeMode::coast);
		pros::delay(400);
		chassis->moveDistance(12_in);
		frontIntake.move(-127 * 2);
		chassis->driveToPoint({7_ft, 5_ft}); // +X is forward, +Y is right
		// chassis->driveToPoint({4_ft, 1_ft}); // +X is forward, +Y is right
		chassis->waitUntilSettled();
		frontIntake.move(0);
		chassis->driveToPoint({9.35_ft, 1.5_ft});
		pros::delay(300);
		chassis->turnAngle(-45_deg);
		pneumatics.set_value(true);
		frontIntake.move(-127 * 2);
		pros::delay(400);
		chassis->setMaxVelocity(200);
		chassis->turnAngle(5_deg);
		chassis->moveDistance(1.4_ft);
		chassis->setMaxVelocity(150);

		// chassis->driveToPoint({7.90_ft, -3.5_ft});
		pros::delay(1000);
		// chassis->driveToPoint({1_ft, 2.5_ft});
		chassis->waitUntilSettled();
		chassis->moveDistanceAsync(-7.5_ft);
		pros::delay(2500);
		if (chassis->isSettled() != true)
		{
			chassis->stop();
		}
		// chassis->driveToPoint({9.55_ft, 3.5_ft}, true);
		chassis->setMaxVelocity(600);
		// chassis->driveToPoint({9.79_ft, 4.5_ft}, true);
		chassis->setMaxVelocity(150);
		// chassis->setMaxVelocity(150);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		pros::delay(1500);
		frontIntake.move(0);
		backIntake.move(0);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		pros::delay(2000);
		// PART 2
		frontIntake.move(0);
		backIntake.move(0);
		chassis->setMaxVelocity(150);
		chassis->getModel()->setBrakeMode(AbstractMotor::brakeMode::coast);
		pros::delay(300);
		chassis->moveDistance(1.75_ft);
		chassis->turnAngle(-100_deg);
		pros::delay(100);
		// chassis->moveDistance(-2_ft);
		chassis->moveDistanceAsync(-8.25_ft);
		pros::delay(2000);
		if (chassis->isSettled() != true)
		{
			chassis->stop();
		}
		chassis->moveDistance(13.5_ft);
		chassis->turnAngle(100_deg);
		// chassis->moveDistance(-1_ft);

		chassis->moveDistanceAsync(-3.0_ft);
		pros::delay(3000);
		if (chassis->isSettled() != true)
		{
			chassis->stop();
		}
		chassis->setMaxVelocity(350);
		chassis->moveDistanceAsync(-2.0_ft);
		pros::delay(500);
		if (chassis->isSettled() != true)
		{
			chassis->stop();
		}

		chassis->setMaxVelocity(150);

		pneumatics.set_value(true);
		frontIntake.move(-127 * 2);

		chassis->moveDistance(4.00_ft);
		chassis->setMaxVelocity(350);
		pros::delay(100);
		chassis->moveDistance(0.4_ft);
		chassis->setMaxVelocity(150);
		pros::delay(2500);
		frontIntake.move(0);
		backIntake.move(0);
		chassis->turnAngle(5_deg);
		// chassis->turnAngle(3.5_deg);
		// chassis->moveDistance(-3.3_ft);
		chassis->moveDistanceAsync(-4.5_ft);

		pros::delay(4000);
		if (chassis->isSettled() != true)
		{
			chassis->stop();
		}
		chassis->setMaxVelocity(350);
		chassis->moveDistanceAsync(-2.0_ft);
		pros::delay(500);
		if (chassis->isSettled() != true)
		{
			chassis->stop();
		}

		chassis->setMaxVelocity(150);
		frontIntake.move(127 * 2);
		pros::delay(100);
		frontIntake.move(-127 * 2);
		backIntake.move(-127 * 2);
		pros::delay(2500);
		frontIntake.move(0);
		backIntake.move(0);
		chassis->moveDistance(1_ft);
		pneumatics.set_value(false);
		chassis->turnAngle(-125_deg);
		chassis->setMaxVelocity(350);
		chassis->moveDistance(-6_ft);
		pros::delay(300);
		chassis->turnAngle(-65_deg);
		chassis->moveDistance(-4_ft);
		pros::delay(20000);
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

bool buttonRegisterDown(ControllerDigital button)
{
	if (!controller.getDigital(button))
	{
		return false;
	}
	else
	{
		auto startTime = std::chrono::steady_clock::now();
		auto endTime = startTime + std::chrono::milliseconds(1000);
		while (std::chrono::steady_clock::now() < endTime)
		{
			if (!controller.getDigital(button))
			{
				return false;
			}
			else
			{
				continue;
			}
		}
		return true;
	}
}

bool toggleA = false;

bool buttonRegisterA()
{
	if (!controller.getDigital(ControllerDigital::A))
	{
		return false;
	}
	else
	{
		if (buttonRegisterDown(ControllerDigital::A))
		{
			toggleA = !toggleA;
			return true;
		}
	}
}

bool toggleLeft = false;

bool buttonRegisterLeft()
{
	if (!controller.getDigital(ControllerDigital::left))
	{
		return false;
	}
	else
	{
		if (buttonRegisterDown(ControllerDigital::left))
		{
			toggleLeft = !toggleLeft;
			return true;
		}
	}
}

bool toggleB = false;

bool buttonRegisterB()
{
	if (!controller.getDigital(ControllerDigital::B))
	{
		return false;
	}
	else
	{
		if (buttonRegisterDown(ControllerDigital::B))
		{
			toggleB = !toggleB;
			return true;
		}
	}
}

void opcontrol()
{
	pneumatics.set_value(false);
	chassis->setMaxVelocity(600);
	while (true)
	{
		chassis->setMaxVelocity(1200 * 40);
		buttonRegisterA();
		buttonRegisterB();
		buttonRegisterLeft();
		/* Code User Control*/
		chassis->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX));

		if (controller.getDigital(ControllerDigital::right) && !toggleA)
		{
			frontIntake.move(127);
			backIntake.move(127);
		}
		else if (controller.getDigital(ControllerDigital::Y))
		{
			frontIntake.move(-127);
			backIntake.move(-127);
			toggleA = false;
		}

		if (toggleA && controller.getDigital(ControllerDigital::right))
		{
			frontIntake.move(127);
			backIntake.move(127);
			toggleA = false;
		}
		if (toggleA && controller.getDigital(ControllerDigital::L2))
		{
			toggleA = false;
			frontIntake.move(-127);
		}
		else if (controller.getDigital(ControllerDigital::L2))
		{
			toggleA = false;
			frontIntake.move(-127);
		}
		else if (toggleA)
		{
			frontIntake.move(-127);
		}

		if (controller.getDigital(ControllerDigital::R2))
		{
			pneumatics.set_value(true);
		}
		else if (controller.getDigital(ControllerDigital::R1))
		{
			pneumatics.set_value(false);
		}

		if (!controller.getDigital(ControllerDigital::right) && !controller.getDigital(ControllerDigital::Y) && !controller.getDigital(ControllerDigital::L2) && !toggleA)
		{
			frontIntake.move(0);
			backIntake.move(0);
		}

		if (!toggleB)
		{
			wings.set_value(toggleB);
		}
		else if (toggleB)
		{
			wings.set_value(toggleB);
		}

		if (!toggleLeft)
		{
			midgoal.set_value(toggleLeft);
		}
		else if (toggleLeft)
		{
			midgoal.set_value(toggleLeft);
		}
		pros::delay(20);
	}
}