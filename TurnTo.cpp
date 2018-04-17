#include "TurnTo.h"

TurnTo::TurnTo(float _targetAngle) : frc::Command("TurnTo") {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::driveTrain);
	this->targetAngle = _targetAngle;
	this->initAngle = Robot::MotionTracker->GetAngle();
	this->robotAngle = Robot::MotionTracker->GetAngle();
}

// Called just before this Command runs the first time
void TurnTo::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void TurnTo::Execute() {
	this->robotAngle = Robot::MotionTracker->GetAngle();
	while (this->targetAngle >= 360) {
		this->targetAngle -= 360;
	}
	while (this->targetAngle < 0) {
		this->targetAngle += 360;
	}

	while (this->robotAngle >= 360) {
		this->robotAngle -= 360;
	}
	while (this->robotAngle < 0) {
		this->robotAngle += 360;
	}
	float dif = this->targetAngle - this->robotAngle; // left is negative

	if (dif<=-180) {
		dif += 360;
	}
	if (dif>180) {
		dif -= 360;
	}

	float speed = ((dif/180)*(TurnMax-TurnMin));

	if (speed<0) {
		speed -= TurnMin;
	}
	else if (speed>0) {
		speed += TurnMin;
	}

	Robot::driveTrain->Set(speed, speed);
}

// Make this return true when this Command no longer needs to run execute()
bool TurnTo::IsFinished() {
	if (this->targetAngle - this->robotAngle <  0){
		if ( this->targetAngle - this->robotAngle > -3){
			return true;
		}
		else return false;
	}
	else {
		if ( this->targetAngle - this->robotAngle < 3){
			return true;
		}
		else return false;
	}
}

// Called once after isFinished returns true
void TurnTo::End() {
	Robot::driveTrain->Set(0, 0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TurnTo::Interrupted() {
	End();
}
