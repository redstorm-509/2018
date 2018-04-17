#include "DriveTrain.h"

DriveTrain::DriveTrain() : frc::Subsystem("DriveTrain") {

}

void DriveTrain::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	SetDefaultCommand(new DriveSCurve());
}

void DriveTrain::Set(double leftValue, double rightValue){
	m_rr.Set(rightValue);
	m_rf.Set(rightValue);
	m_lr.Set(leftValue);
	m_lf.Set(leftValue);
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
