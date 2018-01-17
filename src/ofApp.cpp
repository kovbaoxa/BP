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
    
    //Note you don't have to use ofxKinectV2 as a shared pointer, but if you want to have it in a vector ( ie: for multuple ) it needs to be.
    for(int d = 0; d < kinects.size(); d++){
        kinects[d] = shared_ptr <ofxKinectV2> (new ofxKinectV2());
        kinects[d]->open(deviceList[d].serial);
        panel.add(kinects[d]->params);
    }

    panel.loadFromFile("settings.xml");

//    int w = texDepth[0].getWidth();
//    int h = texDepth[0].getHeight();

    fbo.allocate(400,400,GL_RGBA,4);//GL_LUMINANCE);

    foundHand = false;   
  
    fbo.begin();
	ofClear(0.0f,0.0f,0.0f,0.0f);//255,255,255,0);
    fbo.end();

    //ofEnableAlphaBlending();
 //   texDepth[0].setAlphaMask(fbo.getTexture());

   closestDepth = -1; // not reachable value
}

//--------------------------------------------------------------
void ofApp::update(){
    
    for(int d = 0; d < kinects.size(); d++){
        kinects[d]->update();
        if( kinects[d]->isFrameNew() ){
            texDepth[d].loadData( kinects[d]->getDepthPixels() );
            texRGB[d].loadData( kinects[d]->getRgbPixels() );
        }
    }

  ofTexture depth = texDepth[0];

/*  for(int i = 0; i < texDepth[0].getWidth()*texDepth[0].getHeight(); i++){
	printf("i = %d value = %d\n", i, kinects[0]->getDepthPixels()[i]);// readToPixels(depth,i));   //.readToPixels());
// 	std:cout << "value: " << kinects[0]->getDepthPixels()[i];

   }
*/

   closestDepth = -1; // in case hand get farther away 

   for(int i = 0; i < depth.getWidth(); i++){
	for(int j = 0; j < depth.getHeight(); j++){
		int temp = kinects[0]->getDepthPixels()[i*depth.getWidth() + j];
//		printf("%d", temp);
		if(temp > closestDepth){
			closestDepth = temp;
		//	printf("i: %d \t j: %d \t closestDepth: %d\n",i,j,closestDepth);
		}
	}
   }

   sizeOfHand = 0;

   for(int i = 0; i < depth.getWidth(); i++){
        for(int j = 0; j < depth.getHeight(); j++){
                int temp = kinects[0]->getDepthPixels()[i*depth.getWidth() + j];
//              printf("%d", temp);
		
                if(temp == closestDepth){
			closestSpot[sizeOfHand][0] = i;
			closestSpot[sizeOfHand][1] = j;
                        sizeOfHand++;
               }
        }
   }


   if(foundHand){
        printf("Hand found");
 	//coor[][1] == //insert in history array 
   }

   ofSleepMillis(10000);

}


//--------------------------------------------------------------
void ofApp::draw(){

//	for(int i = 0; i < sezeOfHand; i++){
		ofEnableBlendMode(OF_BLENDMODE_ALPHA);
		fbo.begin();
		ofSetColor(0,0,0,20);
		ofDrawRectangle(0,0,400,400);
		ofSetColor(255,255,255);
		ofTranslate(200,200);
		ofNoFill();
		//ofRotate(ofGetElapsedTimef()*50,1,0,0.5);
		//ofDrawBox(0,0,0,200);
	for(int i = 0; i < sizeOfHand; i++){
		ofDrawSphere(closestSpot[i][1],closestSpot[i][2],5);
	}
		fbo.end();
		ofEnableBlendMode(OF_BLENDMODE_ADD);

   float dwHD = 1920/4;
   float dhHD = 1080/4;

   float shiftY = 100 + ((10 + texDepth[0].getHeight()) * 0);
    
   texDepth[0].draw(200, shiftY);
   texRGB[0].draw(210 + texDepth[0].getWidth(), shiftY, dwHD, dhHD);


   fbo.draw(200,shiftY);
 	
//    panel.draw();

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
