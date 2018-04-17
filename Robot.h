/*
 * Robot.h
 *
 *  Created on: Apr 12, 2018
 *      Author: Programming
 */

#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <TimedRobot.h>
#include <Joystick.h>
#include "Subsystems/DriveTrain.h"
#include <WPILib.h>
#include <AHRS.h>
#include "OI.h"
#include "ctre/Phoenix.h"
#include "Commands/ExampleCommand.h"
#include "Commands/MyAutoCommand.h"

class DriveTrain;

class Robot : public frc::TimedRobot{
public:

	static OI *oi;
	static DriveTrain *driveTrain;
	static AHRS *MotionTracker;
	virtual void RobotInit();
	virtual void DisabledInit();
	virtual void DisabledPeriodic();
	virtual void AutonomousInit();
	virtual void AutonomousPeriodic();
	virtual void TeleopInit();
	virtual void TeleopPeriodic();
	virtual void TestPeriodic();
	double ABS(double);

private:
	frc::Command* m_autonomousCommand = nullptr;
	ExampleCommand m_defaultAuto;
	MyAutoCommand m_myAuto;

	frc::SendableChooser<frc::Command*> m_chooser;


};




#endif /* SRC_ROBOT_H_ */
