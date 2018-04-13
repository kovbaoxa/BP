#include "ofApp.h"

#define loopX(x) for(int x = 0; x < HEIGHT ; x++)
#define loopY(y) for(int y = 0; y < WIDTH; y++)
#define index(i,j) (i*(WIDTH) + j)

//--------------------------------------------------------------
void ofApp::setup(){
    
	//Uncomment for verbose info from libfreenect2
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofBackground(30, 30, 30);
 
	//see how many devices we have.
	ofxKinectV2 tmp;
	vector <ofxKinectV2::KinectDeviceInfo> deviceList = tmp.getDeviceList();
    
	//allocate for this many devices
	kinects.resize(deviceList.size());
	texDepth.resize(kinects.size());
	texRGB.resize(kinects.size());
        
	panel.setup("", "settings.xml", 10, 100);

	//kinect = new ofxKinectV2();
	//kinect->open(deviceList[0].serial);
	
       	kinects[0] = shared_ptr <ofxKinectV2> (new ofxKinectV2());
       	kinects[0]->open(deviceList[0].serial);
       	panel.add(kinects[0]->params);

	kinect = kinects[0];

	fbo.allocate(512,424,GL_RGBA,4);//GL_LUMINANCE);

//	foundHand = false;   

	fbo.begin();
	ofClear(0.0f,0.0f,0.0f,0.0f);//255,255,255,0);
	fbo.end();

}

//--------------------------------------------------------------
void ofApp::update(){
    
	kinect->update();
        if(kinect->isFrameNew()){
        	texDepth[0].loadData(kinect->getDepthPixels());
        	texRGB[0].loadData(kinect->getRgbPixels());
		getDepth = kinects[0]->getDepthPixels();
		ofTexture depth = texDepth[0];

		sizeOfHand = 0;
//		closestDepth = -1; // some not reachable value in case hand get further 
		//if(ofGetFrameNum()%2==0){//(int)ofGetElapsedTimef()%5==0){
			printf("\n NEW UPDATE \t time: %f \n", ofGetElapsedTimef());
			//load picture into own array
			loopX(x){
				loopY(y){
					index = index(x,y);
					myDepth[index] = getDepth[index];
				//	printf("%d\t",myDepth[y*WIDTH +x]);
					backupDepth[index] = getDepth[index];
				}
			}


//			timeStart = ofGetElapsedTimef();
		//			timeEnd = ofGetElapsedTimef();
//			howLong = timeEnd-timeStart;
//			printf("howLong: %f\n", howLong);
			

	//		printf("backupDepth\n");
	//		printArray(backupDepth);
			filterNoise();
	//		printf("AFTER FILTER\n");

	//		printArray(myDepth);
			treshold();
			//findClosestSpot();
			findInBinary();
			detectHand();
			findSquare();
		//	searchFingerBlocks();

	//		printf("BINARY\n");
	//		printArray(myBinary);


	//	} //end for time partition
	} //end for if isFrameNew

	
}


//--------------------------------------------------------------
void ofApp::draw(){

	ofEnableBlendMode(OF_BLENDMODE_ALPHA);

	fbo.begin();
	ofClear(0.0f,0.0f,0.0f,0.0f);
	ofSetColor(255,255,255);
	/*shift into center of depht video*/
	ofTranslate(256,221);
	ofNoFill();
	//ofRotate(ofGetElapsedTimef()*50,1,0,0.5);
	ofSetColor(255,0,0);
//	ofDrawBox(-100,-100,0,50);
	int r = (sizeOfHand > 100) ? (100) : 50;
	ofSetLineWidth(5);	
	if(sizeOfHand > 10){
		ofDrawBox((Xavg-255),Yavg-212,0,r,r,5);
	}
	//ofSetColor(0,255,0);
	//ofDrawBox(255,212,0, 50,50,5);
	r = max_of_M/2;
	if((max_i != 0) & (max_j != 0)){
		ofSetColor(0,255,255);
		ofSetLineWidth(8);		
		ofDrawBox((max_i-r)-255, (max_j-r)-212, 0, max_of_M, max_of_M, 5);	

	}
//	for(int i = 0; i < 4; i++){
//		ofDrawSphere(fingers[i][0]-255, fingers[i][1]-212, max_of_M/4);
//	}


	//ofDrawBox(-5,-33,0,r,r,5);

/*	loopX(i){
		loopY(j){
			if(myBinary[index(i,j)] == 1){
				ofDrawSphere(closestSpot[i][0]-255, closestSpot[i][1]-212,3);
			}
		}
	}*/
/*	for(int i = 0; i < sizeOfHand; i++){
		ofDrawSphere((closestSpot[i][0]-256),(closestSpot[i][1]-212),5);
	}*/
	

	fbo.end();

	ofEnableBlendMode(OF_BLENDMODE_ADD);
	
	float dwHD = 1920/4;
	float dhHD = 1080/4;

	float shiftY = 100 + ((10 + texDepth[0].getHeight()) * 0);
    
	texDepth[0].draw(200, shiftY);
	texRGB[0].draw(210 + texDepth[0].getWidth(), shiftY, dwHD, dhHD);

	fbo.draw(200,shiftY);
 	ofSetColor(255,0,0);
//	panel.draw();
	ofSetColor(255,255,255);

}

/*
* Find closest spot according to depth data.
*
*/
void ofApp::findClosestSpot(){
	closestDepth = -1;
	sizeOfHand = 0;
//	printf("--start searching \n");
	int temp;
	closestDepth = *std::max_element(myDepth, myDepth+WIDTH*HEIGHT);	
	
	loopX(i){
		loopY(j){
	//		int temp = getDepth[j*WIDTH + i];
			temp = myDepth[index(i,j)];
		//	if(i == 200 && j == 50)
			//printf("myDepth %d \t",myDepth[j*WIDTH + i]);
			if(temp == closestDepth){
				//printf("closestDepth: %d \t temp: %d \n", closestDepth, temp);
				if(sizeOfHand < (WIDTH*HEIGHT)/4){
					closestSpot[sizeOfHand][0] = i;
					closestSpot[sizeOfHand][1] = j;
					sizeOfHand++;
					foundHand = true;
				}else if(sizeOfHand==0){
					foundHand = false;
				//	foundHand = false;
				//	printf("---NOT HAND---");
				}
			 }
		}
   	}
//	printf("--search done\n");
}

/*
* Find closest spot according to depth data from binary image.
*
*/
void ofApp::findInBinary(){
	closestDepth = -1;
	sizeOfHand = 0;
//	printf("--start searching \n");
	char temp;
	closestDepth = *std::max_element(myDepth, myDepth+WIDTH*HEIGHT);	
	
	loopX(i){
		loopY(j){
	//		int temp = getDepth[j*WIDTH + i];
			temp = myBinary[index(i,j)];
		//	if(i == 200 && j == 50)
			//printf("myDepth %d \t",myDepth[j*WIDTH + i]);
			if(temp == 'y'){
				//printf("closestDepth: %d \t temp: %d \n", closestDepth, temp);
				if(sizeOfHand < (WIDTH*HEIGHT)/4){
					closestSpot[sizeOfHand][0] = j;
					closestSpot[sizeOfHand][1] = i;
					sizeOfHand++;
					foundHand = true;
				}else{
				//	foundHand = false;
				//	printf("---NOT HAND---");
				}
			 }
		}
   	}
//	printf("--search done\n");
}

/*
* Filter noise from picture (at least some of it) with median filter.
* Median is calculated from size defined NEIGHBORHOOD.
*/
void ofApp::filterNoise(){
//	printf("--filter started\n");
	int arr[NEIGHBORHOOD];
	int m;
	loopX(k){
		loopY(l){
			m = 0;
			//leave outer pixels out of blurring
			if(l > 0 && l < WIDTH-1  && k > 0 && k < HEIGHT-1){  
				for(int n = k-NEIGH_OFFSET; n < k+NEIGH_OFFSET+1; n++){
					for(int o = l-NEIGH_OFFSET; o < l+NEIGH_OFFSET+1; o++){
						arr[m] = backupDepth[index(n,o)];
						m++;
					}
				}
			/*	for(int i = 0; i < 9; i++){
					printf("%d,", arr[i]);
				}
				printf(":\n");
			*/	
				quickSort(arr, 0, 8);
			/*	for(int i = 0; i < 9; i++){
					printf("%d,", arr[i]);
				}
				printf("::\n");
				printf(">%d, %d\n", myDepth[index(k, l)], arr[4]);*/
				myDepth[index(k,l)] = arr[4];
			}

			
		 }
	}
//	printf("--done filter\n");
}

void ofApp::detectHand(){			
//	printf(foundHand?"found\n":"not found\n");
	if(sizeOfHand>10){
		printf("sizeOfHand: %d \n", sizeOfHand);
		printf("closest depth: %d \n", closestDepth);

//	contourFinder.findContours(texDepth,0,50,0,false,true);
		Xavg = 0;
		Yavg = 0;

		if(sizeOfHand > 0){
			for(int i = 0; i < sizeOfHand; i++){
				if(closestSpot[i][0] != -1){ 	//just to be sure
					Xavg+=(closestSpot[i][0]);
					Yavg+=(closestSpot[i][1]);
				//	printf("count of points: %d\t sum:%d\n")		
				//	printf("sum%d\n",Xavg);
				}
			}

			Xavg /= sizeOfHand;
			Yavg /= sizeOfHand;
			printf("Xavg: %d \t Yavg: %d \n", Xavg,Yavg);
			/*coord. system is (-256,256)x(-212,212)*/
		//	Xavg -= 255;
		//	Yavg -=212;
		}
	}
}


/*
* Going through binary image and looking for biggest square.
* inspired by https://www.geeksforgeeks.org/maximum-size-sub-matrix-with-all-1s-in-a-binary-matrix/
*/
void ofApp::findSquare(){

	int i,j;
	int M[HEIGHT][WIDTH];
	max_of_M = 0;
	max_i = 0;
	max_j = 0;

	for(i = 0; i < HEIGHT; i++){
		M[i][0] = (myBinary[i*WIDTH] == 'y');// ? 1 : 0;
	}
	for(j = 0; j < WIDTH; j++){
		M[0][j] = (myBinary[j] == 'y');// ? 1 : 0;
	}
	
	for(i = 1; i < HEIGHT; i++){
		for(j = 1; j < WIDTH; j++){
			if(myBinary[index(i,j)] != 'y'){
				M[i][j] = 0;
			}else{
				M[i][j] = int (findMin(M[i][j-1], M[i-1][j], M[i-1][j-1]) + 1);
			//	printf("%d\t %d %d %d\n", M[i][j], myBinary[index(i, j-1)], myBinary[index(i-1,j)], myBinary[index(i-1,j-1)]);i
				if(M[i][j] > max_of_M){
					max_of_M = M[i][j];
					max_j = i;
					max_i = j;
				}
			}
		}
	}

	/*for(i = 0; i < HEIGHT; i++){
		for(j = 0; j < WIDTH; j++){
			//printf("%d ", M[i][j]);
				}
	//	printf("\n");
	}*/
	printf("max_x: %d\t max_y: %d\t max_of_M: %d\n", max_i, max_j, max_of_M);

	

}

int ofApp::findMin(int a, int b, int c){

	int min = a;
	if(b < min){
		min = b;
	}
	if(c < min){
		min = c;
	}
	return min;
}



/*
* Create my binary image, where 1 is pixel with closest value
*/
void ofApp::treshold(){
	loopX(i){
		loopY(j){
			myBinary[index(i,j)]= (myDepth[index(i,j)]>(closestDepth-10)) ? 'y' : 'n';
		//	printf((myBinary[index(i,j)])? "." : " ");
		}
	//	printf("\n");
	}
//	printArray(myBinary);
}

void ofApp::printArray(int arr[]){

	loopX(i){
		loopY(j){
			printf("%d ", arr[index(i,j)]);
		}
		printf("\n");
	}
}


/*
* Identify blocks, where fingers should be. 
*
*/
void ofApp::searchFingerBlocks(){

	int x_start, x_end, y_start, y_end;

	y_start = max_j - 3*max_of_M;
	y_end = max_j - max_of_M;

	for(int a = 1; a < 5; a++){
		x_start = max_i - a*(max_of_M/4);
		x_end = x_start + max_of_M/4;
		findFingerTip(a, x_start, x_end, y_start, y_end);
	} 
}

int ofApp::countFingers(int a, int b1, int b2, bool isX){

	int counter = 0;
	int lastValue;
	int size = 0;

//	int index = isX ? index[b1,a] : index[a,b1];
	if(isX){ //a is X coodinate
		lastValue = myBinary[index(b1,a)];
		for(int i = b1+1; i <= b2; i++){
			if(myBinary[index(i,a)] != lastValue){
				if(myBinary[index(i,a)] == 1){
					size = 1;
				}else{
					if(size < max_of_M/2){
						counter++;
					}
					size = 0;
				}
			}
			size++;
		}
	}else{
		lastValue = myBinary[index(a,b1)];
		for(int i = b1+1; i <= b2; i++){
			if(myBinary[index(a,i)] != lastValue){
				if(myBinary[index(i,a)] == 1){
					size = 1;
				}else{
					if(size < max_of_M/2){
						counter++;
					}
					size = 0;
				}
			}
			size++;
		}
	}
	return counter;
}


/*
* Search blocks with fingers for local maxima, which should be finger tips. (TODO: if too short?)
* a is index of finger, (x_start, x_end) is range of x coordinates, (y_start, y_end) is range of y coordinates
*/
void ofApp::findFingerTip(int a, int x_start, int x_end, int y_start, int y_end){

	int localMax_x = 0;
	int localMax_y = y_end;
//	printf("y_start: %d\t y_end: %d\t x_start: %d\t x_end: %d\t", y_start, y_end, x_start, x_end );	


	for(int i = y_start; i < y_end; i++){
		for(int j = x_start; j < x_end; j++){
			if(myBinary[index(i,j)] == 1){
				if (i < localMax_y){
					localMax_y = i;
					localMax_x = j;
				}
			}
		}
	}
	fingers[a][0] = localMax_x;
	fingers[a][1] = localMax_y;
//	printf("FINGER No. %d\t x: %d\t y: %d\t\n",a ,localMax_x ,localMax_y);
}


//void ofApp::flip(){

//	myDepth

//}

/*
* quick sort from 
* https://www.geeksforgeeks.org/quick-sort/
*/
void ofApp::swap(int* a, int* b){

	int temp = *a;
	*a = *b;
	*b = temp;
}

int ofApp::partition(int array[], int low, int high){

	int pivot = array[high];
	int i = low - 1;
	
	for(int j = low; j <= high - 1; j++){
		if(array[j] <= pivot){
			i++;
			swap(&array[i], &array[j]);
		}	
	}	
	swap(&array[i+1], &array[high]);
	return (i+1);
}


void ofApp::quickSort(int array[], int low, int high){
	if(low<high){
		int partI = partition(array, low, high);
		quickSort(array, low, partI - 1);
		quickSort(array, partI + 1, high);

	}

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}


//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}


