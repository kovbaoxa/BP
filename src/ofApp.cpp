#include "ofApp.h"

#define loopX(x) for(int x = 0; x < HEIGHT ; x++)
#define loopY(y) for(int y = 0; y < WIDTH; y++)
#define index(i,j) (i*(WIDTH) + j)
#define OFFSET_WIDTH 255
#define OFFSET_HEIGHT 212

//--------------------------------------------------------------
void ofApp::setup(){

	ofBackground(30, 30, 30);

	//see how many devices we have.
	ofxKinectV2 tmp;
	vector <ofxKinectV2::KinectDeviceInfo> deviceList = tmp.getDeviceList();

	//allocate for this many devices
	kinects.resize(deviceList.size());
	texDepth.resize(kinects.size());
	texRGB.resize(kinects.size());

	panel.setup("", "settings.xml", 10, 100);

       	kinects[0] = shared_ptr <ofxKinectV2> (new ofxKinectV2());
       	kinects[0]->open(deviceList[0].serial);
       	panel.add(kinects[0]->params);

	kinect = kinects[0];

	fbo.allocate(512,424,GL_RGBA,4);

	fbo.begin();
	ofClear(0.0f,0.0f,0.0f,0.0f);
	fbo.end();
}

//--------------------------------------------------------------
void ofApp::update(){

	kinect->update();
        if(kinect->isFrameNew()){
		time_start = ofGetElapsedTimef();
        	texDepth[0].loadData(kinect->getDepthPixels());
        	texRGB[0].loadData(kinect->getRgbPixels());
		getDepth = kinects[0]->getDepthPixels();
		ofTexture depth = texDepth[0];

		if(ofGetFrameNum()){

			printf("\n NEW UPDATE \t time: %f \n", ofGetElapsedTimef());
			size_of_hand = 0;
			closest_depth = -1;
			hand_found = false;
			//null banned neighborhood for findFingerTips4
			nullBanned();
			//null number of fingers
			fingers_found = 0;

			//load picture into own array and shadow it into backup one (for blurring)
			loopX(x){
				loopY(y){
					index = index(x,y);
					my_depth[index] = getDepth[index];
					backup_depth[index] = getDepth[index];
				}
			}

			filterNoise();
			treshold();
			findInBinary();
			detectHand();
			findSquare();
			cropROI();
			whereFingersAt();
			if(hand_found){
				findFingerTip4();
			}
		}
	}
}


//--------------------------------------------------------------
void ofApp::draw(){

	ofEnableBlendMode(OF_BLENDMODE_ALPHA);

	fbo.begin();
	ofClear(0.0f,0.0f,0.0f,0.0f);
	ofSetColor(255,255,255);

	/*shift into center of depht video*/
	ofTranslate(OFFSET_WIDTH,OFFSET_HEIGHT);	

	ofNoFill();
	ofSetColor(255,0,0);
	ofSetLineWidth(5);

	int r = max_of_M/2;

	//draw palm if found (coordinates not zero)
	if((max_i != 0) & (max_j != 0)){
		ofSetColor(0,255,255);
		ofSetLineWidth(8);
		ofDrawBox((max_i-r)-OFFSET_WIDTH, (max_j-r)-OFFSET_HEIGHT, 0, max_of_M, max_of_M, 5);
	}

	//draw fingers if found
	for(int i = 0; i <= fingers_found; i++){
		ofDrawSphere(fingers[i][0]-OFFSET_WIDTH, fingers[i][1]-OFFSET_HEIGHT, max_of_M/5);
	}

	fbo.end();
	ofEnableBlendMode(OF_BLENDMODE_ADD);

	float dwHD = 1920/4;
	float dhHD = 1080/4;

	float shiftY = 100 + ((10 + texDepth[0].getHeight()) * 0);

	texDepth[0].draw(200, shiftY);
	texRGB[0].draw(210 + texDepth[0].getWidth(), shiftY, dwHD, dhHD);

	fbo.draw(200,shiftY);
}

/*
* Find closest spot according to depth data. Search through whole picture and save coordinates of pixels.
* 'output' closest_spot[size][2] ... coordinates of all pixels with closest_depth value
*/
void ofApp::findClosestSpot(){

	int temp;
	closest_depth = *std::max_element(my_depth, my_depth+WIDTH*HEIGHT);

	loopX(i){
		loopY(j){
			temp = my_depth[index(i,j)];
			if(temp == closest_depth){
				if(size_of_hand < (WIDTH*HEIGHT)/4){
					closest_spot[size_of_hand][0] = i;
					closest_spot[size_of_hand][1] = j;
					size_of_hand++;
				}
			 }
		}
   	}
}

/*
* Filter noise from picture (at least some of it) with median filter.
* Median is calculated from size defined NEIGHBORHOOD.
*/
void ofApp::filterNoise(){

	int arr[NEIGHBORHOOD];
	int m;

	loopX(k){
		loopY(l){
			m = 0;
			//leave outer pixels out of blurring
			if(l > 0 && l < WIDTH-1  && k > 0 && k < HEIGHT-1){
				for(int n = k-NEIGH_OFFSET; n < k+NEIGH_OFFSET+1; n++){
					for(int o = l-NEIGH_OFFSET; o < l+NEIGH_OFFSET+1; o++){
						arr[m] = backup_depth[index(n,o)];
						m++;
					}
				}
				quickSort(arr, 0, 8);
				my_depth[index(k,l)] = arr[4];
			}
		 }
	}
}

/*
* Create binary image, where 'y' is pixel with closest value and 'n' are rest of them.
*/
void ofApp::treshold(){

	closest_depth = *std::max_element(my_depth, my_depth+WIDTH*HEIGHT);

	loopX(i){
		loopY(j){
			my_binary[index(i,j)]= (my_depth[index(i,j)]>(closest_depth-10)) ? 'y' : 'n';
		}
	}
}

/*
* Find object in binary image.  Search through binary picture and save coordinates of pixels.
* 'output' closest_spot[size][2] ... coordinates of all pixels 
* requires calling treshold() first
*/
void ofApp::findInBinary(){

	char temp;

	loopX(i){
		loopY(j){
			temp = my_binary[index(i,j)];
			if(temp == 'y'){
				if(size_of_hand < (WIDTH*HEIGHT)/4){
					closest_spot[size_of_hand][0] = j;
					closest_spot[size_of_hand][1] = i;
					size_of_hand++;
				}
			 }
		}
   	}
}

/*
* Count average X and average Y of found object (closest spot). 
* 'output' Xavg & Yavg ... coordinates of center of the object
*/
void ofApp::detectHand(){	
		
	if(size_of_hand > 10){

		printf("size_of_hand: %d \n", size_of_hand);
		printf("closest_depth: %d \n", closest_depth);

		Xavg = 0;
		Yavg = 0;

		for(int i = 0; i < size_of_hand; i++){
			if(closest_spot[i][0] != -1){ 	//just to be sure
				Xavg+=(closest_spot[i][0]);
				Yavg+=(closest_spot[i][1]);
			}
		}

		Xavg /= size_of_hand;
		Yavg /= size_of_hand;
		printf("Closest object \t Xavg: %d \t Yavg: %d \n", Xavg,Yavg);
	}
}

/*
* Going through binary image and looking for biggest square representing palm.
* inspired by https://www.geeksforgeeks.org/maximum-size-sub-matrix-with-all-1s-in-a-binary-matrix/
* 'output' Xc, Yc ... coordinates of found biggest square from binary picture
*	max_of_M ... size of biggest square from binary picture
*/
void ofApp::findSquare(){

	int i,j;
	int M[HEIGHT][WIDTH];
	max_of_M = 0;
	max_i = 0;
	max_j = 0;

	//copy top row and left collumn
	for(i = 0; i < HEIGHT; i++){
		M[i][0] = (my_binary[i*WIDTH] == 'y');
	}
	for(j = 0; j < WIDTH; j++){
		M[0][j] = (my_binary[j] == 'y');
	}

	//count matrix to find biggest square
	for(i = 1; i < HEIGHT; i++){
		for(j = 1; j < WIDTH; j++){
			if(my_binary[index(i,j)] != 'y'){
				M[i][j] = 0;
			}else{
				M[i][j] = int (findMin(M[i][j-1], M[i-1][j], M[i-1][j-1]) + 1);
				if(M[i][j] > max_of_M){
					max_of_M = M[i][j];
					max_j = i;
					max_i = j;
				}
			}
		}
	}

	Xc = max_i - max_of_M/2;
	Yc = max_j - max_of_M/2;

	printf("Xc: %d\t Yc: %d\t max_of_M: %d\n", Xc, Yc, max_of_M);
}

/*
* Crop ROI (region of interest) from binary picture. ROI is sized eighth of W*H and center is in palm center.
* 'output' my_spot[size_of_picture/8] ... cut out ROI from binary image
*/
void ofApp::cropROI(){

	//offset on each side of palm-center
	int d = sqrt(EIGHTH)/2; 

	for(int i = Xc - d; i < Xc + d; i++){
		for(int j = Yc - d; j < Yc + d; j++){
			my_spot[index(j,i)] = my_binary[index(j,i)];
		}
	}
}

/*
* Go through binary image by line on coordinates a from b1 to b2
* inputs: a ... coordinate of line (which is same)
*	b1 ... start of line
*	b2 ... end of line
*	dir ... direction according to the palm
*/
int ofApp::countFingers(int a, int b1, int b2, int dir){

	int counter = 0; 	// number of fingers
	int last_value;		// last value on binary image
	int size = 0;		// size (widht) of passing object

	if(dir%2 == 0){ // a is X coordinate
		last_value = my_spot[index(b1,a)];
		for(int i = b1+1; i <= b2; i++){
			if(my_spot[index(i,a)] != last_value){
				if(my_binary[index(i,a)] == 1){
					size = 1;
					// save coordinates of start of the object
					finger_width[dir][counter][0][0] = a;
					finger_width[dir][counter][0][1] = i;
				}else{
					if(size < max_of_M/3){ // then it is a finger
						counter++;
						// save coordinates of end of the finger
						finger_width[dir][counter][1][0] = a;
						finger_width[dir][counter][1][1] = i;
					}
					size = 0;
				}
			}
			size++;
		}
	}else{ // a is Y coordinate
		last_value = my_spot[index(a,b1)];
		for(int i = b1+1; i <= b2; i++){
			if(my_spot[index(a,i)] != last_value){
				if(my_spot[index(a,i)] == 1){
					size = 1;
					// save coordinates of start of the object
					finger_width[dir][counter][0][0] = i;
					finger_width[dir][counter][0][1] = a;

				}else{
					if(size < max_of_M/3){// then it is a finger
						counter++;
						// save coordinates of end of the finger
						finger_width[dir][counter][0][0] = i;
						finger_width[dir][counter][0][1] = a;
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
* Find direction of fingers.
* search all four directions in two different distances. If in both distances in one direction is found
* at least finger, add it into directions.
* 'output' dir_of_fingers[2] ... maximum of directions is two
*/
void ofApp::whereFingersAt(){

	int off_palm = 5*max_of_M/7;	// distance of measurments from edge of !detected! palm
	int x, y;
	int x1 = Xc - off_palm;	// coordinates of bigger square to go through
	int x2 = Xc + off_palm;	// |
	int y1 = Yc - off_palm;	// |
	int y2 = Yc + off_palm;	// v

	int how_many = 0;	// number of found fingers
	int how_many_now = 0;	// number of found fingers at different distance
	int index = 0;

	// reset directions for not reachable value
	for(int i = 0; i < 2; i++){
		dir_of_fingers[i] = -1;
	}

	for(int i = 0; i < 4; i++){ // in each direction
		if(i%2 == 0){
			x = (i == 0) ? x1 : x2;
			how_many = countFingers(x, y1, y2, i);
			x = (i == 0) ? (x - 2*off_palm) : (x + 2*off_palm); 
			how_many_now = countFingers(x, y1, y2, i);
		}else{
			y = (i == 1) ? y1 : y2;
			how_many = countFingers(y, x1, x2, i);
			y = (i == 1) ? (y - 2*off_palm) : (y + 2*off_palm);
			how_many_now = countFingers(y, x1, x2, i);
		}
		if((how_many > 0) && (how_many_now > 0)){
			//fingers can be only on two corresponding sides 0&1; 1&2; 2&3; 3&0
			if((i == 0) || ((i - dir_of_fingers[0])%2 == 1)){
				printf("found fingers on %d.side\n", i);
				dir_of_fingers[index] = i;
				index++;
				hand_found = true; // palm with fingers is sufficient condition
			}
			fingers_vertical = (i%2 == 0);
		}
	}
}

/*
* Search block with fingers for local maximum, which should be finger tip.
* Only for upward position of hand.
* inputs: finger_index ... index of finger
*	x_start, x_end ... range of x coordinates
*	y_start, y_end ... range of y coordinates
* 'output' fingers[5][2] ... fingers with coordinates
*/
void ofApp::findFingerTip(int finger_index, int x_start, int x_end, int y_start, int y_end){

	int localMax_x = 0;
	int localMax_y = y_end;

	for(int i = y_start; i < y_end; i++){
		for(int j = x_start; j < x_end; j++){
			if(my_binary[index(i,j)] == 1){
				if (i < localMax_y){
					localMax_y = i;
					localMax_x = j;
				}
			}
		}
	}
	fingers[finger_index][0] = localMax_x;
	fingers[finger_index][1] = localMax_y;
}

/*
* Search block with fingers for pixel with biggest distance from center of the palm.
* 'output' fingers[5][2] ... fingers with coordinates
* improved funcktion findFingerTip
*/
void ofApp::findFingerTip2(int finger_index, int x_start, int x_end, int y_start, int y_end){

	int max_dist = 0;
	int distance = 0;
	int max_dist_x = 0;
	int max_dist_y = 0;

	for(int i = y_start; i < y_end; i++){
		for(int j = x_start; j < x_end; j++){
			if(my_spot[index(i,j)] == 'y'){
				distance = sqrt((Xc-j)*(Xc-j) + (Yc-i)*(Yc-i));
				if (distance > max_dist){
					max_dist_y = i;
					max_dist_x = j;
					max_dist = distance;
				}
			}
		}
	}

	fingers[finger_index][0] = max_dist_x;
	fingers[finger_index][1] = max_dist_y;
	printf("FINGER No. %d\t x: %d\t y: %d\t\n",finger_index ,max_dist_x ,max_dist_y);
}

/*
* Find a middle point of fingers found in 'whereFingersAt' and then follow them to the maximum value.
* Mark lask found value as finger tip.
* input dir_index ... index of direction to look into
* 'output' fingers[5][2] ... fingers with coordinates
*/
void ofApp::findFingerTip3(int dir_index){

	int middle = -10;	// not reachable value
	int width = -10;	// not reachable value
	int length = 0;		// length of finger
	int min_length = max_of_M/2;	// objects shorter than this are not finger
	int count = 0;		// number of fingers
	int dir = dir_of_fingers[dir_index];


	for(int i = 0; i < 5; i++){

		length = 0;

		//in each direction there are coordinates of start and end (not length) of finger
		int x1 = finger_width[dir][i][0][0];
		int x2 = finger_width[dir][i][1][0];
		int y1 = finger_width[dir][i][0][1];
		int y2 = finger_width[dir][i][1][1];

		if(fingers_vertical){
			width = y2 - y1;
			middle = y2 - (width/2);
		}else{
			width = x2 - x1;
			middle = x2 - (width/2);
		}

		switch(dir){
			case 0:
				for(int j = max_i - max_of_M; j > max_i - 4*max_of_M; j--){
					if(my_spot[index(middle, j)] == 'y'){
						fingers[i][0] = j;
						fingers[i][1] = middle;
						length++;
					}else{
						break;
					}
				}
				break;
			case 1:
				for(int j = max_j - max_of_M; j > max_j - 4*max_of_M; j--){
					if(my_spot[index(j, middle)] == 'y' ){
						fingers[i][0] = middle;
						fingers[i][1] = j;
						length++;
					}else{
						break;
					}
				}
				break;
			case 2:
				for(int j = max_i; j < max_i + 3*max_of_M; j++){
					if(my_spot[index(middle, j)] == 'y' ){
						fingers[i][0] = j;
						fingers[i][1] = middle;
						length++;
					}else{
						break;
					}
				}
				break;
			case 3:
				for(int j = max_j; j < max_j + 3*max_of_M; j++){
					if(my_spot[index(j, middle)] == 'y' ){
						fingers[i][0] = middle;
						fingers[i][1] = j;
						length++;
					}else{
						break;
					}
				}
				break;
			default:
				printf("no");
				break;
		}

		if(length > min_length){ // necessary condition
			count++;
		}else{	// not reachable values
			fingers[i][0] = -1000;
			fingers[i][1] = -1000;
		}

		printf("FINGER No. %d\t x: %d\t y: %d\t direction: %d\t length: %d \n", i, fingers[i][0] ,fingers[i][1], dir, length);
	}
}

/*
* Search block with fingers from outer edge and detect first occurance. Ban neighborhood for next
* search and continue (function searchFromTo()) in directions of found fingers.
* 'output' fingers[5][2] ... fingers with coordinates
*/
void ofApp::findFingerTip4(){

	fingers_found = 0;
	int a1,a2,b1,b2;	// corners of block
	bool backwards = false;		// direction of searching in picture
	int directions =(dir_of_fingers[1] == -1) ? 1 : 2;	// detect number of directions

	for(int i = 0; i < directions; i ++){

		int cur_direction = dir_of_fingers[i];		//current direction
		nullBanned();		//reset banned coordinates
		printf("searching in %d. direction\n", cur_direction );

		switch(cur_direction){
			case 0:
				b1 = (Yc - max_of_M > 0) ? (Yc - max_of_M) : 0;
				b2 = (Yc + max_of_M < HEIGHT) ? (Yc + max_of_M) : HEIGHT;
				a1 = (Xc - 2*max_of_M > 0) ? (Xc - 2*max_of_M) : 0;
				a2 = (Xc - max_of_M > 0) ? (Xc - max_of_M) : 0;
				backwards = false;
				break;
			case 1:
				a1 = (Yc - 2*max_of_M > 0) ? (Yc - 2*max_of_M) : 0;
				a2 = (Yc - max_of_M > 0) ? (Yc - max_of_M) : 0;
				b1 = (Xc - max_of_M > 0) ? (Xc - max_of_M) : 0;
				b2 = (Xc + max_of_M < WIDTH) ? (Xc + max_of_M) : WIDTH;
				backwards = false;
				break;

			case 2:
				b1 = (Yc - max_of_M > 0) ? (Yc - max_of_M) : 0;
				b2 = (Yc + max_of_M < HEIGHT) ? (Yc + max_of_M) : HEIGHT;
				a1 = (Xc + 2*max_of_M < WIDTH) ? (Xc + 2*max_of_M) : WIDTH;
				a2 = (Xc + max_of_M < WIDTH) ? (Xc + max_of_M) : WIDTH;
				backwards = true;
				break;
			case 3:
				a1 = (Yc + 2*max_of_M < HEIGHT) ? (Yc + 2*max_of_M) : HEIGHT;
				a2 = (Yc + max_of_M < HEIGHT) ? (Yc + max_of_M) : HEIGHT;
				b1 = (Xc - max_of_M > 0) ? (Xc - max_of_M) : 0;
				b2 = (Xc + max_of_M < WIDTH) ? (Xc + max_of_M) : WIDTH;
				backwards = true;
				break;
			default:
				printf("Error in findFingerTips4\n");
				break;
		}
		searchFromTo(a1, a2, b1, b2, backwards, cur_direction);
		printf("Found %d fingers \n", fingers_found);
	}
}

/*
* Search block with fingers from outer edge and detect first occurance. Ban neighborhood for next
* search and continue.
* input: outer_from, outer_to ... range for outer 'for cycle'
*	inner_from, inner_to ... range for inner 'for cycle'
*	otherWay ... direction of search in picture
*	direction ... direction of fingers 
*/
void ofApp::searchFromTo(int outer_from, int outer_to, int inner_from, int inner_to, bool otherWay, int direction){
	
	int x,y;

	if(!otherWay){
		for(int i = outer_from; i < outer_to; i++){
			for(int j = inner_from; j < inner_to; j++){
				if(direction%2 == 0){ // fingers horizontal
					x = i;
					y = j;
				}else{	// finger vertical
					x = j;
					y = i;
				}

				if(my_spot[index(y,x)] == 'y'){
					//check if coordinate is not banned from previous search
					if(!isThereFinger(j)){ 
						fingers[fingers_found][0] = x;
						fingers[fingers_found][1] = y;
						printf("FINGER No. %d\t x: %d\t y: %d\t \n", fingers_found, fingers[fingers_found][0] ,fingers[fingers_found][1]);
						identifyFinger(x,y,(direction%2 == 0));
						fingers_found++;
						if(fingers_found == 0){ //cut range when first tip found
							outer_to = i + max_of_M/2; 
						}
						if(fingers_found == 4){ //maximum of number on one side
							return;
						}
					}
				}
			}
		}
	}else{
		for(int i = outer_from; i > outer_to; i--){
			for(int j = inner_from; j < inner_to; j++){
				if(direction%2 == 0){ // fingers horizontal
					x = i;
					y = j;
				}else{ // finger vertical
					x = j;
					y = i;
				}
				if(my_spot[index(i,j)] == 'y'){
					//check if coordinate is not banned from previous search
					if(!isThereFinger(j)){
						fingers[fingers_found][0] = x;
						fingers[fingers_found][1] = y;
						printf("FINGER No. %d\t x: %d\t y: %d\t \n", fingers_found, fingers[fingers_found][0] ,fingers[fingers_found][1]);
						identifyFinger(x,y,(direction%2 == 0));
						fingers_found++;
						if(fingers_found == 0){ //cut range when first tip found
							outer_to = i - max_of_M/2;
						}
						if(fingers_found == 4){//maximum of number on one side
							return;
						}
					}
				}	
			}
		}
	}
}

/*
* Search array of banned coordinated. If it's not there, it means it's free. Then it's added including
* its neighborhood.
* output: true if coordinate is banned
*/
bool ofApp::isThereFinger(int a){

	int offset = max_of_M / 2; // block of size 2*palm is searched -> devided by 4 fingers

	if(num_of_banned == 100){
		return true;
	}

	for(int i = 0; i < 100; i++){
		if(banned[i] == a){
			return true;
		}
	}

	for(int i = a - offset; i < a + offset; i++){
		banned[num_of_banned++] = i;
		if(num_of_banned>99){
			break;
		}
	}

	if(fingers_found > 4){ // to be sure
		printf("%d\t", fingers_found);
	}
	return false;
}

/*
* Return index of finger according to location of found fingertip.
* Not recognising what funger is what index, only their sequence.
* input: x, y ... coordinates of fingertip
* output: index of finger
*/
int ofApp::identifyFinger(int x, int y, bool horizontal){

	int diff = 0;

	if(!horizontal){//fingers_vertical){
		diff = Xc - x;
	} else {
		diff = Yc - y;
	}

	int mod = diff%max_of_M;

	if((mod > -max_of_M) & (mod < -max_of_M/2)){
		printf("finger at %d, %d indexed as 0\n", x, y);
		return 0;
	}else if((mod > -max_of_M/2) & (mod < 0)){
		printf("finger at %d, %d indexed as 1\n", x, y);
		return 1;
	}else if((mod > 0 & mod) < (max_of_M/2)){
		printf("finger at %d, %d indexed as 2\n", x, y);
		return 2;
	}else if((mod > max_of_M/2) & (mod < max_of_M)){
		printf("finger at %d, %d indexed as 3\n", x, y);
		return 3;
	}else{
		printf("problem in identifyFinger\n");
	}
	return -1;
}

/*
* According to the way of fingers, detect what blocks need to be searched for fingertips
*/
void ofApp::findFingers(){

	int off = max_of_M/4;

	for(int i = 0; i < 2; i++){
		printf("Switch in case %d\n", dir_of_fingers[i]);
		switch(dir_of_fingers[i]){
			case 0:
				searchFingerBlocks(max_i-4*max_of_M, max_i-max_of_M, max_j-max_of_M-off, max_j + off);
				break;
			case 1:
				searchFingerBlocks(max_i-max_of_M-off, max_i+off, max_j-4*max_of_M-off, max_j + max_of_M);
				break;
			case 2:
				searchFingerBlocks(max_i, max_i+3*max_of_M, max_j-max_of_M-off,max_j+off);
				break;
			case 3:
				searchFingerBlocks(max_i-max_of_M-off,max_i+off,max_j,max_j+3*max_of_M);
				break;
			default:
				printf("Error in findFinger\n");
				break;
		}
	}
}

/*
* Identify blocks, where fingers should be. 
* input: x_start, x_end ... x coordinate range
*	y_start, y_end ... y coordinate range
*/
void ofApp::searchFingerBlocks(int x_start, int x_end, int y_start, int y_end){

	int x_size = x_end-x_start;
	int y_size = y_end-y_start;
	int x1 = x_start;
	int x2 = x_end;
	int y1 = y_start;
	int y2 = y_end;

	for(float a = 0; a < 4; a++){
		if(fingers_vertical){
			x1 = x_start + (a/4)*x_size;
			x2 = x1 + x_size/4;
		}else{
			y1 = y_start + (a/4)*y_size;
			y2 = y1 + y_size/4;
		}
		findFingerTip2(a, x1, x2, y1, y2);
	}
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

/*
* Reset array of banned coordinates.
*/
void ofApp::nullBanned(){

	for(int i = 0; i < 100; i++){ 
		banned[i] = 0;
	}
	num_of_banned = 0;
}

/*
* Return minimal value from parameters.
*/
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
* For debugging. Print 2D array.
*/
void ofApp::printArray(int arr[]){

	loopX(i){
		loopY(j){
			printf("%d ", arr[index(i,j)]);
		}
		printf("\n");
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


