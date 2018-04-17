/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"
#include "Commands/Drive.h"
#include "Commands/DriveSCurve.h"
#include "Commands/TurnTo.h"

OI::OI() {
	rstick = new Joystick(0);
	lstick = new Joystick(1);
	// Process operator interface input here.
	frc::JoystickButton *Rbutton1 = new JoystickButton(rstick, 1),
						*Lbutton1 = new JoystickButton(lstick, 1),
						*Rbutton4 = new JoystickButton(lstick, 4),
						*Rbutton5 = new JoystickButton(rstick, 5);
	Rbutton1->WhileHeld(new Drive());
	Lbutton1->WhileHeld(new Drive());
	Rbutton4->WhenPressed(new TurnTo(-90));
	Rbutton5->WhenPressed(new TurnTo(90));




}

double OI::GetSCurveLeftJoystickY(){
	return sCurve(lstick->GetY(), .5);
}

double OI::GetSCurveRightJoystickY(){
	return sCurve(rstick->GetY(), .5);
}

double OI::GetRightJoystickY(){
	return rstick->GetY();
}
double OI::GetLeftJoystickY(){
	return lstick->GetY();
}

double OI::sCurve(float joyVal, float sCurveFactor = .5){
	return (((1-sCurveFactor)*pow(joyVal,3)) + (sCurveFactor*joyVal));
}
