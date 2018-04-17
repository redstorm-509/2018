#include "Drive.h"




Drive::Drive() : frc::Command("Drive") {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::driveTrain);
}

// Called just before this Command runs the first time
void Drive::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void Drive::Execute() {
	Robot::driveTrain->Set(-Robot::oi->GetLeftJoystickY(), Robot::oi->GetRightJoystickY());
}

// Make this return true when this Command no longer needs to run execute()
bool Drive::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void Drive::End() {
	Robot::driveTrain->Set(0, 0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Drive::Interrupted() {
	End();
}
