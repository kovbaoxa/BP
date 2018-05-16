#pragma once

#include "ofMain.h"
#include "ofxGui.h"
//#include "ofxOpenCv.h"
#include "ofxKinectV2.h"

#define WIDTH 512
#define HEIGHT 424
#define EIGHTH (WIDTH*HEIGHT/8) //27 136
#define NEIGHBORHOOD 9 // can be 9 (neighborhoor 3x3) or 25 (neighborhood 5x5)
#define NEIGH_OFFSET (NEIGHBORHOOD/9) //for calculating of coordinates

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
		void searchFingerBlocks(int x1, int x2, int y1, int y2);
		void findFingerTip(int a, int x1, int x2, int y1, int y2);
		int countFingers(int a, int b1, int b2, int dir);
		void whereFingersAt();
		void findFingerTip2(int finger_index, int x1, int x2, int y1, int y2);
		void findFingerTip3(int i);
		void findFingerTip4();
		void searchFromTo(int from1, int to1, int from2, int to2, bool other_way, int dir);
		bool isThereFinger(int a);
		int identifyFinger(int x, int y);
		void findFingers();
		void cropROI();

	ofxPanel panel;

        vector < shared_ptr<ofxKinectV2> > kinects;
	shared_ptr<ofxKinectV2> kinect;

	ofPixels getDepth;
	ofFbo fbo;
        vector <ofTexture> texDepth;
        vector <ofTexture> texRGB;

/*
* frame = 217 088 px
*/

	int my_depth[WIDTH*HEIGHT] = { };
	int backup_depth[WIDTH*HEIGHT] = { };
	char my_binary[WIDTH*HEIGHT] = { };
	char my_spot[EIGHTH] = { }; //area of interest

/*
* time related variables
*/
	int timer;	
	float how_long;
	float time_start;
	float time_end;

/*
* coordinates and directions
*/
	int Xavg;	// x coordinate of average centre of closest spot
	int Yavg;	// y coordinate of average centre of closest spot
	int Xc;		// x coordinate of centre of tha palm
	int Yc;		// y coordinate of centre of the palm
	char dir_of_fingers [2] = { };		// fingers appear only on two sides of hand
	int fingers[5][2];	// coordinates of finger tips
	int closest_spot[EIGHTH][2];		//coordinates of closest spot
	int finger_width[4][5][2][2];		//4 dir, 5 fingers, start/end, x/y


/*
* values
*/
	int size_of_hand;	// size of closest spot found on picture
	int index;	
	int offset_depth;	// hand is not always exactly in one plane
	bool found_hand;
	int closest_depth;	// closest depth on the picture	
	int median_neigh[NEIGHBORHOOD];		// index for finding median according to neighborhood size
	bool fingers_vertical;
	int num_of_banned;
	int banned[100];	//size of palm was always <50 
	int fingers_found;
	bool hand_found;

/*
* coordinates and size of found palm
*/
	int M[WIDTH][HEIGHT];	// initial array for search
	int max_of_M = 0;	// size of biggest square found
	int max_i = 0;		// x coordinate of lower right corner of the biggest square found
	int max_j = 0;		// y coordinate of lower right corner of the biggest square found

};
