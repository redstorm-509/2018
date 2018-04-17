#ifndef DriveSCurve_H
#define DriveSCurve_H

#include <Commands/Command.h>
#include "Subsystems/DriveTrain.h"
#include "Robot.h"

class DriveSCurve : public frc::Command {
public:
	DriveSCurve();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveSCurve_H
