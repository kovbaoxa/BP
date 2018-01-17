#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxKinectV2.h"

#define MOVES_IN_HIST

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		void drawFbo();
        
	ofxPanel panel;
    
        vector < shared_ptr<ofxKinectV2> > kinects;
//	ofxKinectV2 kinect;

	int closestSpot[4200][2]; //21000+ pixel in frame and 1 hand can not occupy more then 4200 pixels
	int sizeOfHand;	

	ofFbo fbo;    
        vector <ofTexture> texDepth;
        vector <ofTexture> texRGB;
//	vector <ofCircle> fingers;
	bool foundHand;
	int coor [MOVES_IN_HIST][3];
	int closestDepth;

};
