#include <Joystick.h>
#include <SampleRobot.h>
#include <Talon.h>
#include "ctre/Phoenix.h"
#include <Timer.h>
#include <AHRS.h>
#include <math.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>

class Robot: public frc::SampleRobot {

bool tractionDown = false;
bool gyroenabled = false;
bool isJoystick = true;
double PI = 3.14159;
bool IsMechanum = true;
float SminimumY = .5;
float SminimumX = .5;
float TargetMin = .25;
float TurnMax = .5;
float TurnMin = .27;
float sCurveFactor = .5;
int StartPosition = 1;
bool isAutonomoose;
double MatchTime;
bool endGame = false;

int imageXsize = 320, imageYsize = 240;
std::vector<double> visionX;
std::vector<double> visionY;

bool target = true;
int Xdesired = 160, Ydesired = 196;
float xval, yval;
float xpos = 0, ypos = 0;

public:
	float ABS(float input) {
		if (input<0) {
			input = input * -1;
		}
		return input;
	}
	void smtNUM(std::string text, double num){
		SmartDashboard::SmartDashboard::PutNumber(text, num);
	}
	void smtSTR(std::string text){
		SmartDashboard::SmartDashboard::PutString(text, "hit");
	}
	void smtBOOL(std::string text, bool blean){
		SmartDashboard::SmartDashboard::PutBoolean(text, blean);
	}
	void tankDrive(float RY, float LY){
		m_lf.Set(-LY);
		m_lr.Set(-LY);
		m_rf.Set(RY);
		m_rr.Set(RY);
	}
	double sCurve(float joyVal){
		return (((1-sCurveFactor)*pow(joyVal,3)) + (sCurveFactor*joyVal));
	}
	void CanMechanum(float RX, float RY, float LX, float robotangle){

		float mag = sqrt((RY*RY)+(RX*RX));
		float joyangle = 0;
		if (gyroenabled) {
			robotangle = (((PI)/180)*robotangle);
			while ((robotangle >= (2*PI)) or (robotangle < 0)){
				if (robotangle >= (2*PI)){
					robotangle = robotangle - (2*PI);
				}

				else if (robotangle < 0){
					robotangle = robotangle + (2*PI);
				}
			}
			float dif = 0;
			if ((RX>=0) && (RY>0)) { // Top Right
				joyangle = atan(RX/RY) + 0;
				dif = joyangle - robotangle;
			}
			else if ((RX>0) && (RY<=0)) { // Bottom Right
				joyangle = (atan((-RY)/RX)) + ((PI)/2);
				dif = joyangle - robotangle;
			}
			else if ((RX<=0) && (RY<0)) { // Bottom Left
				joyangle = atan((-RX)/(-RY)) + (PI);
				dif = joyangle - robotangle;
			}
			else if ((RX<0) && (RY>=0)) { // Top Left
				joyangle = atan(RY/(-RX)) + ((3*PI)/2);
				dif = joyangle - robotangle;
			}
			RY = mag * cos(dif);
			RX = mag * sin(dif);
		}

		mag = sqrt((RY*RY)+(RX*RX));

		float lfVal = RY + LX + RX;
		float lrVal = RY + LX - RX;
		float rfVal = -RY + LX + RX;
		float rrVal = -RY + LX - RX;

		float maxval = lfVal;
		if (lrVal>maxval) {
			maxval = lrVal;
		}
		if (rfVal>maxval) {
			maxval = rfVal;
		}
		if (rrVal>maxval) {
			maxval = rrVal;
		}

		if (ABS(maxval) > 1){
			lfVal = ((lfVal / ABS(maxval))*(mag/sqrt(2)));
			lrVal = ((lrVal / ABS(maxval))*mag/sqrt(2));
			rfVal = ((rfVal / ABS(maxval))*mag/sqrt(2));
			rrVal = ((rrVal / ABS(maxval))*mag/sqrt(2));
		}

		m_lf.Set(lfVal);
		m_lr.Set(lrVal);
		m_rf.Set(rfVal);
		m_rr.Set(rrVal);
	}

	float TurnTo(int theta) {
		while (theta >= 360) {
			theta -= 360;
		}
		while (theta < 0) {
			theta += 360;
		}
		float robotangle = MotionTracker.GetAngle();
		while (robotangle >= 360) {
			robotangle -= 360;
		}
		while (robotangle < 0) {
			robotangle += 360;
		}
		float dif = theta - robotangle; // left is negative

		if (ABS(dif)<(3)) {
			return 0;
		}

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

		return speed;

	}

	void CAM() {

		target = false;

		visionX = entryX.GetDoubleArray(llvm::ArrayRef<double>());
		if (visionX.size()>0) {
			SmartDashboard::PutNumber("VisionX",visionX[0]);
			target = true;
		}

		visionY = entryY.GetDoubleArray(llvm::ArrayRef<double>());
		if (visionY.size()>0) {
			SmartDashboard::PutNumber("VisionY",visionY[0]);
			target = true;
		}
		smtBOOL("target", target);

	}

	int GetMaxYIndex(){

		int maxIndex = 0, maxY;

		if (target) {
			maxY = visionY[0];
			for (unsigned int i = 0; i < visionY.size(); i++){

				if (visionY[i] > maxY){ //change the less than if y is in wrong direction
					maxY = visionY[i];
					maxIndex = i;
				}

			}

		}
		else {
			maxIndex = -1;
		}

		return maxIndex;
	}

	void Locate(){

		int Tindex = GetMaxYIndex();

		if (target == true && Tindex != -1) {


			yval = visionY[Tindex];
			xval = visionX[Tindex];

			ypos = ((Ydesired - yval)/(imageYsize / 2)); //gives the position in terms of 1 to -1 which can also be used for proportional movement speed
			xpos = ((Xdesired - xval)/(imageXsize / 2));

			if (yval >= 170){
				ypos = 0;
			}
			if (ABS(xval-Xdesired)<=15){
				xpos = 0;
			}

			if (ypos < -.05){
				ypos -= TargetMin;
			}
			else if (ypos > .05){
				ypos += TargetMin;
			}
			else {
				ypos = 0;
			}

			if (xpos < -.05){
				xpos -= TargetMin;
			}
			else if (xpos > .05){
				xpos += TargetMin;
			}
			else{
				xpos = 0;
			}
			smtNUM("Xpos", xpos);
			smtNUM("Ypos", ypos);
			CanMechanum(xpos, ypos, 0, 0);

		}
		else {
			smtBOOL("Located", false);
			//CanMechanum(0, 0, .5, 0);
		}

	}

	void Accessories() {
		m_lifter.Set(opstick.GetY());
		smtNUM("GetY", xdrive.GetY());
		smtNUM("GetYChannel", xdrive.GetYChannel());
		m_rIntake.Set(opstick.GetThrottle());
		m_lIntake.Set(-opstick.GetThrottle());
		if (opstick.GetRawButton(5)){
			m_rIntake.Set(-.95);
			m_lIntake.Set(.95);
		}
		if (opstick.GetRawButton(1)){
			grabSol.Set(DoubleSolenoid::DoubleSolenoid::kForward);
		}
		else {
			grabSol.Set(DoubleSolenoid::DoubleSolenoid::kReverse);
		}
		if (opstick.GetRawButton(3)){
			endGame = true;
		}
		else if (opstick.GetRawButton(2)){
			endGame = false;
		}
		if (endGame){
			if (ABS(opstick.GetY()) > .025){
				brakeSol.Set(DoubleSolenoid::DoubleSolenoid::kForward);
			}
			else{
				brakeSol.Set(DoubleSolenoid::DoubleSolenoid::kReverse);
			}
		}
		else {
			brakeSol.Set(DoubleSolenoid::DoubleSolenoid::kForward);
		}
	}

	void OperatorControl() {
		xdrive.SetXChannel(4);
		xdrive.SetYChannel(5);
		xdrive.SetZChannel(0);
		xdrive.SetThrottleChannel(1);
		//xdrive.SetTwistChannel(2);
		opstick.SetYChannel(1);
		opstick.SetThrottleChannel(2);

		brakeSol.Set(DoubleSolenoid::DoubleSolenoid::kReverse);
		m_rf.Set(0);
		m_lf.Set(0);
		m_rr.Set(0);
		m_lr.Set(0);
		//while(MotionTracker.IsCalibrating()){}
		Comp->SetClosedLoopControl(true);
		while (IsOperatorControl() && IsEnabled()) {
			smtNUM("xdrift", rstick.GetX());
			Accessories();
			float rightX;
			float rightY;
			float leftX;
			float leftY;

			CAM();
			bool holdDown;

			smtNUM("Zval", rstick.GetZ());

			if (rstick.GetRawButton(9) || xdrive.GetRawButton(8)){
				isJoystick = false;
			}
			if (rstick.GetRawButton(8) || xdrive.GetRawButton(7)){
				isJoystick = true;
			}
			if (isJoystick){
				rightX = rstick.GetX();
				rightY = rstick.GetY();
				leftX = lstick.GetX();
				leftY = lstick.GetY();
				if (rstick.GetRawButton(3)){
					MotionTracker.Reset();
				}
				if (rstick.GetRawButton(2)){
					IsMechanum = true;
				}
				else if ( lstick.GetRawButton(2)){
					IsMechanum = false;
				}
				if ((rstick.GetRawButton(1) && lstick.GetRawButton(1))){
					holdDown = true;
				}
				else {
					holdDown = false;
				}


			}
			else {
				rightX = xdrive.GetX();
				rightY = xdrive.GetY();
				leftX = xdrive.GetZ();
				leftY = xdrive.GetThrottle();
				if (xdrive.GetPOV() != -1){
					leftX = TurnTo(xdrive.GetPOV());
				}
				if (xdrive.GetRawButton(4)){
					MotionTracker.Reset();
				}
				if (xdrive.GetRawButton(6)){
					IsMechanum = true;
				}
				else if ( xdrive.GetRawButton(5)){
					IsMechanum = false;
				}
////				if (xdrive.GetTwist() >= .25){
////					holdDown = true;
////				}
//				else {
//					holdDown = false;
//				}
			}

			if (IsMechanum){
				if (false){
					tankDrive(rightY, leftY);
					butterflySol.Set(frc::DoubleSolenoid::kForward);
				}
				else if (rstick.GetRawButton(7)){
					Locate();
				}
				else {
					butterflySol.Set(frc::DoubleSolenoid::kReverse);
					CanMechanum(rightX, -rightY, (leftX/2), MotionTracker.GetAngle());
				}
			}
			else if (!IsMechanum){
				tankDrive(rightY,leftY);
				butterflySol.Set(frc::DoubleSolenoid::kForward);
			}

			DriverStation::GetInstance().GetMatchTime();
			if (MatchTime >= 120 && MatchTime <= 125){
				opstick.SetRumble(Joystick::kRightRumble, .9);
				opstick.SetRumble(Joystick::kLeftRumble, .9);
			}
			else {
				opstick.SetRumble(Joystick::kRightRumble, 0);
				opstick.SetRumble(Joystick::kLeftRumble, 0);
			}



			Wait(kUpdatePeriod);

		}
	}
	void move(float XDisp, float YDisp, float mag = .95){
		smtNUM("moveInitializeNUM", MotionTracker.GetDisplacementX());
		double initialXDisp = MotionTracker.GetDisplacementX();
		double initialYDisp = MotionTracker.GetDisplacementY();
		double speedX = 0;
		double speedY = 0;

		XDisp = (XDisp - .528)/110;
		YDisp = ((YDisp*12) - .528)/110;
		smtNUM("YDisp", YDisp);

		SminimumY = YDisp;
		SminimumX = XDisp;

		while(isAutonomoose){
			//double speedX = ((((XDisp - (MotionTracker.GetDisplacementX()-initialXDisp))/(1))/(1/(mag - Sminimum))));
			//if (((XDisp-(MotionTracker.GetDisplacementX()-initialXDisp))/XDisp) >= .5){
				if (XDisp != 0){
					speedX = mag*(((XDisp-(MotionTracker.GetDisplacementX()-initialXDisp))/XDisp));
					if (ABS(speedX) < SminimumX){
						if (speedX < 0){
							speedX = -SminimumX;
						}
						else {
							speedX = SminimumX;
						}
					}
				}
				else
					speedX = 0;
				if (YDisp != 0){
					speedY = mag*(((YDisp-(MotionTracker.GetDisplacementY()-initialYDisp))/YDisp));
					if (ABS(speedY) < SminimumY){
						if (speedY < 0){
							speedY = -SminimumY;
						}
						else {
							speedY = SminimumY;
						}
					}
				}
				else
					speedY = 0;

			smtNUM("speedY", speedY);
			smtNUM("Y relative dis",MotionTracker.GetDisplacementY()-initialYDisp );
			smtNUM("xdis", MotionTracker.GetDisplacementX());
			smtNUM("ydis", MotionTracker.GetDisplacementY());
			if (XDisp < 0){
				if (MotionTracker.GetDisplacementX()-initialXDisp < XDisp){
					speedX = 0;
				}
			}
			else {
				if (MotionTracker.GetDisplacementX()-initialXDisp >= XDisp){
					speedX = 0;
				}

			}
			if (YDisp < 0){
				if (MotionTracker.GetDisplacementY()-initialYDisp < YDisp){
					speedY = 0;
				}
			}
			else {
				if (MotionTracker.GetDisplacementY()-initialYDisp >= YDisp){
					speedY = 0;
				}
			}

			if (speedX == 0 && speedY == 0){
				break;
			}
//(1-(ABS(MotionTracker.GetDisplacementY()-initialYDisp))/1.5)
			if ((ABS(MotionTracker.GetVelocityX())< .04 && ABS(MotionTracker.GetVelocityY())<.04) && .90 <= ((MotionTracker.GetDisplacementY()-initialYDisp))/YDisp){
				break;
			}

			CanMechanum(speedX, speedY, 0, MotionTracker.GetAngle());
			if (IsOperatorControl()){
				break;
			}
		}
		smtSTR("HIT");
		CanMechanum(0, 0, 0, 0);
	}

	void Autonomous(){
		isAutonomoose = true;
		MotionTracker.Reset();
		bool isRightSwitch = true;
		//brakeSol.Set(DoubleSolenoid::DoubleSolenoid::kForward);



		//m_lifter.Set(.5);
		//m_rIntake.Set(.95);
		//m_lIntake.Set(-.95);
		//Wait(1);
		//m_lifter.Set(0);
		//grabSol.Set(DoubleSolenoid::DoubleSolenoid::kReverse);

		//Wait(.75);
		//m_lifter.Set(-.95);
		//Wait(2);
		m_lifter.Set(0);
		m_rIntake.Set(0);
		m_lIntake.Set(0);

		StartPosition = 4;
//		std::string gameData;
//		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
//		if(gameData.length() > 0){
//			if(gameData[0] == 'R'){
//				isRightSwitch = true;
//			}
//			else {
//				isRightSwitch = false;
//			}
//		}




		if (isRightSwitch&&StartPosition == 1){
			move(0, 14);
		}
		else if (isRightSwitch&&StartPosition == 2){
			//Lift lifter
			move(3.75, 10.5);
			m_rIntake.Set(-.95);
			m_lIntake.Set(.95);
			Wait(1);
			m_rIntake.Set(0);
			m_lIntake.Set(0);

			//Shoot cube
		}
		else if (isRightSwitch&&StartPosition == 3){
			//Lift
			move(0,14);
			TurnTo(270);
			m_rIntake.Set(-.95);
			m_lIntake.Set(.95);
			Wait(1);
			m_rIntake.Set(0);
			m_lIntake.Set(0);
			//Shoot Cube
		}
		else if (!isRightSwitch&&StartPosition == 1){
			//Lift
			move(0,14);
			TurnTo(90);
			m_rIntake.Set(-.95);
			m_lIntake.Set(.95);
			Wait(1);
			m_rIntake.Set(0);
			m_lIntake.Set(0);
			//Shoot Cube
		}
		else if(!isRightSwitch&&StartPosition == 2){
			//Lift lifter
			move(4, 10.5);
			m_rIntake.Set(-.95);
			m_lIntake.Set(.95);
			Wait(1);
			m_rIntake.Set(0);
			m_lIntake.Set(0);
			//Shoot cube
		}
		else if(!isRightSwitch&&StartPosition == 3){
			move(0, 14);
		}
		else {
			smtNUM("Zval", rstick.GetZ());
			move(0, 8);
		}




		isAutonomoose = false;
	}

private:
	frc::Joystick rstick  { 0 };
	frc::Joystick lstick  { 1 };
	frc::Joystick xdrive  { 2 };
	frc::Joystick opstick { 3 };
	WPI_TalonSRX m_rf { 1 };
	WPI_TalonSRX m_rr { 12 };
	WPI_TalonSRX m_lf { 0 };
	WPI_TalonSRX m_lr { 13 };
	WPI_TalonSRX m_lifter { 2 };
	WPI_TalonSRX m_rIntake { 14 };
	WPI_TalonSRX m_lIntake { 15 };
	AHRS MotionTracker {SPI::Port::kMXP}; //new motion tracker
	DoubleSolenoid grabSol {0, 1};
	DoubleSolenoid butterflySol {2, 3};
	DoubleSolenoid brakeSol {5, 4};

	nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table = inst.GetTable("GRIP/CubeReport1");
	nt::NetworkTableEntry entryY = table->GetEntry("centerX");
	nt::NetworkTableEntry entryX = table->GetEntry("centerY");

	Compressor *Comp = new Compressor (0);
	static constexpr double kUpdatePeriod = 0.005;


};

START_ROBOT_CLASS(Robot)
