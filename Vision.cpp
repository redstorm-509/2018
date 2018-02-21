

int imageXsize = 320, imageYsize = 240;
std::vector<double> visionX;
std::vector<double> visionY;

bool target = true;
int Xdesired = 160, Ydesired = 120;
int xval, yval;
int xpos, ypos;

void CAM() {
	
	target = false;

	visionX = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
	if (visionX.size()>0) {
		SmartDashboard::PutNumber("VisionX",visionX[0]);
		target = true;
	}
	
	visionY = table->GetNumberArray("centerY", llvm::ArrayRef<double>());
	if (visionY.size()>0) {
		SmartDashboard::PutNumber("VisionY",visionY[0]);
		target = true;
	}	
	
}

int GetMinYIndex(){
	
	int minIndex = 0, minY;
	
	if (target) {
		minY = visionY[0];
		for (unsigned int i = 0; i < visionY.size(); i++){
			
			if (visionY[i] < minY){ //change the less than if y is in wrong direction
				minY = visionY[i];
				minIndex = i;
			}
			
		}
		
	}
	else {
		minIndex = NULL;
	}
	
	return minIndex;
}

void Locate(){
	
	int index = GetMinYIndex();
	
	if (target == true && index != NULL) {
		
		
		yval = visionY[index];
		xval = visionX[index];
		
		ypos = ((yval - Ydesired)/(imageYsize / 2));
		xpos = ((xval - Xdesired)/(imageXsize / 2));
		
		CanMechanum(xpos, ypos, 0, 0);
		
	}
	else {
		CanMechanum(0, 0, .5, 0);
	}
	
}

