#ifndef DriveTrain_H
#define DriveTrain_H

#include <Commands/Subsystem.h>
#include <WPILib.h>
#include "ctre/Phoenix.h"
#include <Talon.h>
#include "Commands/Drive.h"
#include "Commands/DriveSCurve.h"
#include "Commands/TurnTo.h"


class DriveTrain : public frc::Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	WPI_TalonSRX m_rr {13};
	WPI_TalonSRX m_rf {12};
	WPI_TalonSRX m_lr {1};
	WPI_TalonSRX m_lf {0};
public:
	DriveTrain();
	void InitDefaultCommand();
	void Set(double, double);
};

#endif  // DriveTrain_H
