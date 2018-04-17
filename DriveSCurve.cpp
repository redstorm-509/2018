#include "DriveSCurve.h"

DriveSCurve::DriveSCurve() : frc::Command("DriveSCurve") {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::driveTrain);
}

// Called just before this Command runs the first time
void DriveSCurve::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void DriveSCurve::Execute() {
	Robot::driveTrain->Set(-Robot::oi->GetSCurveLeftJoystickY(), Robot::oi->GetSCurveRightJoystickY());
}

// Make this return true when this Command no longer needs to run execute()
bool DriveSCurve::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveSCurve::End() {
	Robot::driveTrain->Set(0, 0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveSCurve::Interrupted() {
	End();
}
