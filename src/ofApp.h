#pragma once

#include "ofMain.h"
#include "ofxGui.h"
//#include "ofxOpenCv.h"
#include "ofxKinectV2.h"

#define WIDTH 512
#define HEIGHT 424
#define NEIGHBORHOOD 9 // can be 9 (neighborhoor 3x3) or 25 (neighborhood 5x5)
#define NEIGH_OFFSET (NEIGHBORHOOD/9) //for calculating of coordinates


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
		void swap(int* a, int* b);
		int partition(int arr[], int low, int high);
		void quickSort(int arr[], int low, int high);
		void findClosestSpot();
		void filterNoise();
		void detectHand();
		void treshold();
		void findInBinary();
		void printArray(int arr[]);
		void findSquare();
		int findMin(int a, int b, int c);
		void searchFingerBlocks();
		void findFingerTip(int a, int x1, int x2, int y1, int y2);


	ofxPanel panel;
    
        vector < shared_ptr<ofxKinectV2> > kinects;
	shared_ptr<ofxKinectV2> kinect;
	
	ofPixels getDepth;

	/*
	* frame = 210 000+ px
	* frame/9 = 24 000+ px
	* 20 000 px could be more than enough for now
	*/

	int myDepth[WIDTH*HEIGHT] = { };
	int backupDepth[WIDTH*HEIGHT] = { };
	int myBinary[WIDTH*HEIGHT] = { };
	int medianNeigh[NEIGHBORHOOD];
	int closestSpot[(WIDTH*HEIGHT)/9][2]; 
	int sizeOfHand;	
	int timer;

	int Xavg;
	int Yavg;	
	int index;

	int M[WIDTH][HEIGHT];
	int max_of_M = 0;
	int max_i = 0;
	int max_j = 0;


	float howLong;
	float timeStart;
	float timeEnd;
	int offsetDepth;

	ofFbo fbo;
        vector <ofTexture> texDepth;
        vector <ofTexture> texRGB;
	
	bool foundHand;
	int closestDepth;

	int fingers[5][2];
};
