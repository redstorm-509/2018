/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#ifndef OI_H
#define OI_H

#include <WPILib.h>


class OI {
public:

	OI();
	double GetSCurveLeftJoystickY();
	double GetSCurveRightJoystickY();
	double GetRightJoystickY();
	double GetLeftJoystickY();
	double sCurve(float, float);
private:
	frc::Joystick *lstick;
	frc::Joystick *rstick;

};

#endif
