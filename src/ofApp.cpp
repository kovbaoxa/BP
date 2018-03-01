#include "ofApp.h"

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

	panel.loadFromFile("settings.xml");

//	w = texDepth[0].getWidth();
//	h = texDepth[0].getHeight();

	fbo.allocate(512,424,GL_RGBA,4);//GL_LUMINANCE);

	foundHand = false;   

	fbo.begin();
	ofClear(0.0f,0.0f,0.0f,0.0f);//255,255,255,0);
	fbo.end();

	//ofEnableAlphaBlending();
	//texDepth[0].setAlphaMask(fbo.getTexture());

//	closestDepth = -1; // some not reachable value
}

//--------------------------------------------------------------
void ofApp::update(){
    
	kinect->update();
        if(kinect->isFrameNew()){
        	texDepth[0].loadData(kinect->getDepthPixels());
        	texRGB[0].loadData(kinect->getRgbPixels());

		ofTexture depth = texDepth[0];

		w = texDepth[0].getWidth();
		h = texDepth[0].getHeight();
		sizeOfHand = 0;
		closestDepth = -1; // some not reachable value in case hand get further 
		if((int)ofGetElapsedTimef()%2==0){
			printf("NEW UPDATE \t time: %f \n", ofGetElapsedTimef());
			timeStart = ofGetElapsedTimef();
			for(int i = (int) w/3; i < (int) ((2/3.0)*w); i++){
				for(int j = (int)h/3; j < (int) ((2/3.0)*h); j++){			
					int temp = kinects[0]->getDepthPixels()[i*w + j];
					
					if(temp == closestDepth){
					//printf("time: %f\n", ofGetElapsedTimef());
					//printf("closestDepth: %d \t temp: %d \n", closestDepth, temp);
						closestSpot[sizeOfHand][0] = i;
						closestSpot[sizeOfHand][1] = j;
		                       		sizeOfHand++;
						//printf("i: %d\t j: %d\n", i,j);
		              		 }
	
				
					if(temp > closestDepth){
						closestDepth = temp;
				//	printf("i: %d \t j: %d \t closestDepth: %d\n",i,j,closestDepth);
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
			timeEnd = ofGetElapsedTimef();
			howLong = timeEnd-timeStart;
			printf("howLong: %f\n", howLong);
		

//			timeStart = ofGetElapsedTimef();
				
		 
	
		printf("sizeOfHand: %d \n", sizeOfHand);
//	}

       // timeEnd = ofGetElapsedTimef();
       // howLong = timeEnd - timeStart;
      //  printf("howLong: %f\n",howLong);

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

		}
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
	
	ofDrawBox(Xavg,Yavg,0,r,r,5);

/*	for(int i = 0; i < sizeOfHand; i++){
		ofDrawSphere((closestSpot[i][0]-256),(closestSpot[i][1]-212),5);
	}*/
	
/*	for (int i = 0; i < contourFinder.nBlobs; i++){
        	contourFinder.blobs[i].draw();
	} */

	fbo.end();

	ofEnableBlendMode(OF_BLENDMODE_ADD);
	
	float dwHD = 1920/4;
	float dhHD = 1080/4;

	float shiftY = 100 + ((10 + texDepth[0].getHeight()) * 0);
    
	texDepth[0].draw(200, shiftY);
	texRGB[0].draw(210 + texDepth[0].getWidth(), shiftY, dwHD, dhHD);

	fbo.draw(200,shiftY);
 	ofSetColor(255,0,0);
	panel.draw();
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
