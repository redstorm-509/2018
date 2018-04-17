#ifndef TurnTo_H
#define TurnTo_H

#include <Commands/Command.h>
#include "Subsystems/DriveTrain.h"
#include "Robot.h"

class TurnTo : public frc::Command {
public:
	TurnTo(float);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	float initAngle;
	float targetAngle;
	float robotAngle;
	const float TurnMin = .27;
	const float TurnMax = .5;

};

#endif  // TurnTo_H
