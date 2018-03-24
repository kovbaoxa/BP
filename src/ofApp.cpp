#include "ofApp.h"

#define WIDTH 512
#define HEIGHT 424
#define loopX(x) for(int x = 0; x < WIDTH-1; x++)
#define loopY(y) for(int y = 0; y < HEIGHT-1; y++)

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

	//foundHand = false;   

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
		closestDepth = -1; // some not reachable value in case hand get further 
		if((int)ofGetElapsedTimef()%5==0){
			printf("NEW UPDATE \t time: %f \n", ofGetElapsedTimef());

//			timeStart = ofGetElapsedTimef();
			loopX(i){
				loopY(j){
					int temp = getDepth[j*WIDTH + i];
					myDepth[j*WIDTH + i] = temp;	
					if(temp == closestDepth){
					//printf("closestDepth: %d \t temp: %d \n", closestDepth, temp);
						closestSpot[sizeOfHand][0] = i;
						closestSpot[sizeOfHand][1] = j;
		                       		sizeOfHand++;
					 }
				
					if(temp > closestDepth){
						closestDepth = temp;
					//printf("i: %d \t j: %d \t closestDepth: %d\n",i,j,closestDepth);
						for(int k = sizeOfHand; k > 0; k--){
							closestSpot[k][0] = -1;
							closestSpot[k][1] = -1;
						}
						closestSpot[0][0] = i;
						closestSpot[0][1] = j;
						sizeOfHand = 1;	
					}		
				}
   			}
//			timeEnd = ofGetElapsedTimef();
//			howLong = timeEnd-timeStart;
//			printf("howLong: %f\n", howLong);

			int arr[9];		
			int m;

			loopX(k){
				loopY(l){
					m = 0;
					for(int n = k-1; n < k+1; n++){
						for(int o = l-1; o < l+1; o++){
							arr[m] = myDepth[o*WIDTH + k];
							m++;
						}
					}
					quickSort(arr, 0, 8);
					myDepth[l*WIDTH + k] = arr[4];
				 }
			}

			printf("sizeOfHand: %d \n", sizeOfHand);
			printf("closest depth: %d \n", closestDepth);
			for(int k = 0; k < sizeOfHand-1; k++){
				printf("i: %d\t j: %d\n",closestSpot[k][0],closestSpot[k][1]);
			}

//	contourFinder.findContours(texDepth,0,50,0,false,true);
			Xavg = 0;
			Yavg = 0;
			if(sizeOfHand > 0){
				for(int i = 0; i < sizeOfHand; i++){
					if(closestSpot[i][0] != -1){ 	//just to be sure
						Xavg+=(closestSpot[i][0]);
						Yavg+=(closestSpot[i][1]);
					}
				}
	
				Xavg /= sizeOfHand;
				Yavg /= sizeOfHand;
				printf("Xavg: %d \t Yavg: %d \n", Xavg,Yavg);
				/*coord. system is (-256,256)x(-212,212)*/
				Xavg -= 255;
				Yavg -=212;
			}

		} //end for time partition
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
	int r = (sizeOfHand > 100) ? (sizeOfHand/2) : 50;
	ofSetLineWidth(3);	
	ofDrawBox(Xavg,Yavg,0,r,r,5);
	//ofDrawBox(-5,-33,0,r,r,5);



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
