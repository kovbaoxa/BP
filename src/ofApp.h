#pragma once

#include "ofMain.h"
#include "ofxGui.h"
//#include "ofxOpenCv.h"
#include "ofxKinectV2.h"


//#define MOVES_IN_HIST

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
	shared_ptr<ofxKinectV2> kinect;
	
	ofPixels getDepth;

	/*
	* frame = 210 000+ px
	* frame/9 = 24 000+ px
	* 12 000 px could be more than enough for now
	*/

	int closestSpot[12000][2]; 
	int sizeOfHand;	
	int timer;
	int h;
	int w;
	int Xavg;
	int Yavg;	

	float howLong;
	float timeStart;
	float timeEnd;
	int offsetDepth;

	ofFbo fbo;    
        vector <ofTexture> texDepth;
        vector <ofTexture> texRGB;
	
	bool foundHand;
//	int coor [MOVES_IN_HIST][3];
	int closestDepth;

};
