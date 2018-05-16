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

	fbo.begin();
	ofClear(0.0f,0.0f,0.0f,0.0f);//255,255,255,0);
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

		size_of_hand = 0;
		hand_found = false;
//		closestDepth = -1; // some not reachable value in case hand get further 
		if(ofGetFrameNum()%4==0){//(int)ofGetElapsedTimef()%5==0){
			printf("\n NEW UPDATE \t time: %f \n", ofGetElapsedTimef());
			
			//free banned coordinates from last detection
			for(int i = 0; i < num_of_banned; i++){
				banned[i] = 0;
			}
			num_of_banned = 0;

			//null number of fingers
			fingers_found = 0;

			//load picture into own array
			loopX(x){
				loopY(y){
					index = index(x,y);
					my_depth[index] = getDepth[index];
				//	printf("%d\t",myDepth[y*WIDTH +x]);
					backup_depth[index] = getDepth[index];
				}
			}

	//		printf("backupDepth\n");
	//		printArray(backupDepth);
			filterNoise();
	//		printf("AFTER FILTER\n");

			//printArray(my_depth);
			treshold();
			findInBinary();
			detectHand();
			findSquare();
			cropROI();

			//printf("BINARY\n");
			//printArray(my_binary);
			whereFingersAt();
			if(hand_found){
			//	findFingers();
				findFingerTip4();
			//	findFingerTip3(1);
			}
		} //end for time partition
		time_end = ofGetElapsedTimef();
		how_long = time_end - time_start;
	//	printf("Time durancy: %f", how_long);
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
	int r = (size_of_hand > 100) ? (100) : 50;
	ofSetLineWidth(5);
/*	if(size_of_hand > 10){
		ofDrawBox((Xavg-255),Yavg-212,0,r,r,5);
	}*/
	//ofSetColor(0,255,0);
	//ofDrawBox(255,212,0, 50,50,5);
	r = max_of_M/2;
	if((max_i != 0) & (max_j != 0)){
		ofSetColor(0,255,255);
		ofSetLineWidth(8);
		ofDrawBox((max_i-r)-255, (max_j-r)-212, 0, max_of_M, max_of_M, 5);
	}

	for(int i = 0; i <= fingers_found; i++){
		ofDrawSphere(fingers[i][0]-255, fingers[i][1]-212, max_of_M/5);
	}

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

	closest_depth = -1;
	size_of_hand = 0;
//	printf("--start searching \n");
	int temp;
	closest_depth = *std::max_element(my_depth, my_depth+WIDTH*HEIGHT);

	loopX(i){
		loopY(j){
			temp = my_depth[index(i,j)];
			//printf("myDepth %d \t",myDepth[j*WIDTH + i]);
			if(temp == closest_depth){
				//printf("closestDepth: %d \t temp: %d \n", closestDepth, temp);
				if(size_of_hand < (WIDTH*HEIGHT)/4){
					closest_spot[size_of_hand][0] = i;
					closest_spot[size_of_hand][1] = j;
					size_of_hand++;
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

	size_of_hand = 0;
//	printf("--start searching \n");
	char temp;

	loopX(i){
		loopY(j){
			temp = my_binary[index(i,j)];
			if(temp == 'y'){
				//printf("closestDepth: %d \t temp: %d \n", closestDepth, temp);
				if(size_of_hand < (WIDTH*HEIGHT)/4){
					closest_spot[size_of_hand][0] = j;
					closest_spot[size_of_hand][1] = i;
					size_of_hand++;
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
						arr[m] = backup_depth[index(n,o)];
						m++;
					}
				}
				quickSort(arr, 0, 8);
				my_depth[index(k,l)] = arr[4];
			}
		 }
	}
//	printf("--done filter\n");
}

void ofApp::detectHand(){	
		
//	printf(foundHand?"found\n":"not found\n");
	if(size_of_hand>10){
		printf("sizeOfHand: %d \n", size_of_hand);
		printf("closest depth: %d \n", closest_depth);

		Xavg = 0;
		Yavg = 0;

		if(size_of_hand > 0){
			for(int i = 0; i < size_of_hand; i++){
				if(closest_spot[i][0] != -1){ 	//just to be sure
					Xavg+=(closest_spot[i][0]);
					Yavg+=(closest_spot[i][1]);
				//	printf("count of points: %d\t sum:%d\n")		
				//	printf("sum%d\n",Xavg);
				}
			}

			Xavg /= size_of_hand;
			Yavg /= size_of_hand;
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
		M[i][0] = (my_binary[i*WIDTH] == 'y');// ? 1 : 0;
	}
	for(j = 0; j < WIDTH; j++){
		M[0][j] = (my_binary[j] == 'y');// ? 1 : 0;
	}

	for(i = 1; i < HEIGHT; i++){
		for(j = 1; j < WIDTH; j++){
			if(my_binary[index(i,j)] != 'y'){
				M[i][j] = 0;
			}else{
				M[i][j] = int (findMin(M[i][j-1], M[i-1][j], M[i-1][j-1]) + 1);
			//	printf("%d\t %d %d %d\n", M[i][j], myBinary[index(i, j-1)], myBinary[index(i-1,j)], myBinary[index(i-1,j-1)]);
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
	printf("Xc: %d\t Yc: %d\t max_of_M: %d\n", max_i-(max_of_M/2), max_j-(max_of_M/2), max_of_M);

	Xc = max_i - max_of_M/2;
	Yc = max_j - max_of_M/2;
}

/*
* Crop ROI (region of interest) from binary picture. AOI is sized tenth of W*H and center is in palm center.
*/
void ofApp::cropROI(){

	int d = sqrt(EIGHTH)/2;

	for(int i = Xc - d; i < Xc + d; i++){
		for(int j = Yc - d; j < Yc + d; j++){
			my_spot[index(j,i)] = my_binary[index(j,i)];
		}
	}
 
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

	closest_depth = *std::max_element(my_depth, my_depth+WIDTH*HEIGHT);

	loopX(i){
		loopY(j){
			my_binary[index(i,j)]= (my_depth[index(i,j)]>(closest_depth-10)) ? 'y' : 'n';
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

int ofApp::countFingers(int a, int b1, int b2, int dir){

	int counter = 0;
	int last_value;
	int size = 0;

//	int index = isX ? index[b1,a] : index[a,b1];
	if(dir%2 == 0){ //a is X coodinate
		last_value = my_spot[index(b1,a)];
		for(int i = b1+1; i <= b2; i++){
			if(my_spot[index(i,a)] != last_value){
				if(my_binary[index(i,a)] == 1){
					size = 1;
					finger_width[dir][counter][0][0] = a;
					finger_width[dir][counter][0][1] = i;
				}else{
					finger_width[dir][counter][1][0] = a;
					finger_width[dir][counter][1][1] = i;

					if(size < max_of_M/3){
						counter++;
					}
					size = 0;
				}
			}
			size++;
		}
	}else{
		last_value = my_spot[index(a,b1)];
		for(int i = b1+1; i <= b2; i++){
			if(my_spot[index(a,i)] != last_value){
				if(my_spot[index(a,i)] == 1){
					size = 1;
					finger_width[dir][counter][0][0] = i;
					finger_width[dir][counter][0][1] = a;

				}else{
					finger_width[dir][counter][0][0] = i;
					finger_width[dir][counter][0][1] = a;

					if(size < max_of_M/3){
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

void ofApp::whereFingersAt(){

	int off_palm = max_of_M/7;
	int x, y;
	int x1 = max_i-12*off_palm;
	int x2 = max_i+5*off_palm;
	int y1 = max_j-12*off_palm;
	int y2 = max_j+5*off_palm;
	int how_many = 0;
	int index = 0;
	int how_many_now = 0;

	for(int i = 0; i < 2; i++){
		dir_of_fingers[i] = -1;
	}

	for(int i = 0; i < 4; i++){
		if(i%2 == 0){
			x = (i == 0) ? x1 : x2;
			how_many = countFingers(x, y1, y2, i);
			x = (i == 0) ? (x - 2*off_palm) : (x + 2*off_palm); 
			how_many_now = countFingers(x, y1, y2, i);
			//fingers_vertical = true;
		}else{
			y = (i == 1) ? y1 : y2;
			how_many = countFingers(y, x1, x2, i);
			y = (i == 1) ? (y - 2*off_palm) : (y + 2*off_palm);
			how_many_now = countFingers(y, x1, x2, i);
			//fingers_vertical = false;
		}
		if((how_many > 0) && (how_many_now > 0)){
			//fingers can be only on two corresponding sides 0&1; 1&2; 2&3; 3&0
			if((i == 0) || ((i - dir_of_fingers[0])%2 == 1)){
				printf("found fingers on %d.side\n", i);
				dir_of_fingers[index] = i;
				index++;
				hand_found = true;
			}
			fingers_vertical = (i%2 == 0);
		//	printf("dir_of_fingers: %d \t %d \n", dir_of_fingers[0], dir_of_fingers[1]);
		}
	}
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
			if(my_binary[index(i,j)] == 1){
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

void ofApp::findFingerTip2(int finger_index, int x_start, int x_end, int y_start, int y_end){

	int max_dist = 0;
	int distance = 0;
	int max_dist_x = 0;
	int max_dist_y = 0;

	printf("Find finger tip in x1: %d\t x2: %d\t y1: %d\t y2: %d\n", x_start, x_end, y_start, y_end);

	for(int i = y_start; i < y_end; i++){
		for(int j = x_start; j < x_end; j++){
			if(my_spot[index(i,j)] == 'y'){
				distance = sqrt((Xc-j)*(Xc-j) + (Yc-i)*(Yc-i));
			//	printf("dstance: %d\n", distance);
				if (distance > max_dist){
					max_dist_y = i;
					max_dist_x = j;
					max_dist = distance;
				//	printf("Distance is %d on %d \t %d \n", distance, j, i);
				}
			}
		}
	}

	fingers[finger_index][0] = max_dist_x;
	fingers[finger_index][1] = max_dist_y;
	printf("FINGER No. %d\t x: %d\t y: %d\t\n",finger_index ,max_dist_x ,max_dist_y);
}

void ofApp::findFingerTip3(int dir_index){

	int middle = -10;
	int width = -10;
	int length = 0;
	int min_length = max_of_M/2;
	int count = 0;
	int dir = dir_of_fingers[dir_index];


	for(int i = 0; i < 5; i++){

		length = 0;

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

		if(length > min_length){
			count++;
		}else{
			fingers[i][0] = -1000;
			fingers[i][1] = -1000;
		}

		printf("FINGER No. %d\t x: %d\t y: %d\t direction: %d\t length: %d \n", i, fingers[i][0] ,fingers[i][1], dir, length);
	}
//	printf("Found %d fingers \n", count);

}

void ofApp::findFingerTip4(){

	fingers_found = 0;
	int a1,a2,b1,b2;
	bool backwards = false;
	int directions =(dir_of_fingers[1] == -1) ? 1 : 2;

	for(int i = 0; i < directions; i ++){
		int cur_direction = dir_of_fingers[i];
		printf("searching in %d. direction\n", cur_direction );
		switch(cur_direction){
			case 0:
				b1 = (Yc - max_of_M > 0) ? (Yc - max_of_M) : 0;
				b2 = (Yc + max_of_M < HEIGHT) ? (Yc + max_of_M) : HEIGHT;
				a1 = (Xc - 2*max_of_M > 0) ? (Xc - 2*max_of_M) : 0;
				a2 = (Xc - max_of_M > 0) ? (Xc - max_of_M) : 0;
				backwards = false;

//				searchFromTo((Xc - 3*max_of_M), (Xc - max_of_M), (Yc - max_of_M), (Yc + max_of_M), false);
				break;
			case 1:
				a1 = (Yc - 2*max_of_M > 0) ? (Yc - 2*max_of_M) : 0;
				a2 = (Yc - max_of_M > 0) ? (Yc - max_of_M) : 0;
				b1 = (Xc - max_of_M > 0) ? (Xc - max_of_M) : 0;
				b2 = (Xc + max_of_M < WIDTH) ? (Xc + max_of_M) : WIDTH;
				backwards = false;
//				searchFromTo((Yc - 3*max_of_M), (Yc - max_of_M), (Xc - max_of_M), (Xc + max_of_M),  false);
				break;

			case 2:
				b1 = (Yc - max_of_M > 0) ? (Yc - max_of_M) : 0;
				b2 = (Yc + max_of_M < HEIGHT) ? (Yc + max_of_M) : HEIGHT;
				a1 = (Xc + 2*max_of_M < WIDTH) ? (Xc + 2*max_of_M) : WIDTH;
				a2 = (Xc + max_of_M < WIDTH) ? (Xc + max_of_M) : WIDTH;
				backwards = true;

//				searchFromTo((Xc + 3*max_of_M), (Xc + max_of_M), (Yc - max_of_M), (Yc + max_of_M), true);
				break;
			case 3:
				a1 = (Yc + 2*max_of_M < HEIGHT) ? (Yc + 2*max_of_M) : HEIGHT;
				a2 = (Yc + max_of_M < HEIGHT) ? (Yc + max_of_M) : HEIGHT;
				b1 = (Xc - max_of_M > 0) ? (Xc - max_of_M) : 0;
				b2 = (Xc + max_of_M < WIDTH) ? (Xc + max_of_M) : WIDTH;
				backwards = true;

//				searchFromTo((Yc + 3*max_of_M), (Yc + max_of_M), (Xc - max_of_M), (Xc + max_of_M), true);
				break;
			default:
				printf("no");
				break;
		}
		searchFromTo(a1, a2, b1, b2, backwards, cur_direction);
		printf("Found %d fingers \n", fingers_found);
	}
}

/*
*
* TODO: find out what finger it is and write in right index of fingers
*/
void ofApp::searchFromTo(int outer_from, int outer_to, int inner_from, int inner_to, bool otherWay, int direction){
	
	int x,y;
	if(!otherWay){
		for(int i = outer_from; i < outer_to; i++){
			for(int j = inner_from; j < inner_to; j++){
				if(direction%2 == 0){
					x = i;
					y = j;
				}else{
					x = j;
					y = i;
				}

				if(my_spot[index(y,x)] == 'y'){
					if(!isThereFinger(j)){
						fingers[fingers_found][0] = x;
						fingers[fingers_found][1] = y;
						printf("FINGER No. %d\t x: %d\t y: %d\t \n", fingers_found, fingers[fingers_found][0] ,fingers[fingers_found][1]);
						fingers_found++;
						if(fingers_found == 0){ //cut range when first tip found
							outer_to = i + max_of_M/2; 
						}
						if(fingers_found == 3){
							return;
						}
					}
				}	
			}
		}
	}else{
		for(int i = outer_from; i > outer_to; i--){
			for(int j = inner_from; j < inner_to; j++){
				if(direction%2 == 0){
					x = i;
					y = j;
				}else{
					x = j;
					y = i;
				}
				if(my_spot[index(i,j)] == 'y'){
					if(!isThereFinger(j)){
						fingers[fingers_found][0] = x;
						fingers[fingers_found][1] = y;
						printf("FINGER No. %d\t x: %d\t y: %d\t \n", fingers_found, fingers[fingers_found][0] ,fingers[fingers_found][1]);
						fingers_found++;
						if(fingers_found == 0){ //cut range when first tip found
							outer_to = i - max_of_M/2;
						}
						if(fingers_found == 3){
							return;
						}
					}
				}	
			}
		}
	}


}

bool ofApp::isThereFinger(int a){

	int offset = max_of_M / 2; // block of size 2*palm is searched devided by 4 fingers

	for(int i = 0; i < num_of_banned; i++){
		if(banned[i] == a){
			return true;
		}
	}
//	printf("num_of_banned: %d \n", num_of_banned);	
	printf("a: %d\n", a);
	for(int i = a - offset; i < a + offset; i++){
		banned[num_of_banned++] = i;
//		printf("%d \t", i);
	}
//	printf("\n");
	return false;
}

/*
* retunr index of finger according to location of found fingertip
* TODO: only one direction handled
* TODO: how do I know which side of hand am I looking at? - sort it desc by other diff(length)?
*/
int ofApp::identifyFinger(int x, int y){

	int diff = 0;

	if(fingers_vertical){ 
		diff = Yc - y;
	} else {
		diff = Xc - x;
	}

	switch(diff%max_of_M){ // magic number which decides index of finger
		case -3 :
			printf("finger at %d, %d indexed as 0\n", x, y);
			return 0;
		case -1:
 			printf("finger at %d, %d indexed as 1\n", x, y);
			return 1;
		case 1 :
			printf("finger at %d, %d indexed as 2\n", x, y);
			return 2;
		case 3 :			
			printf("finger at %d, %d indexed as 3\n", x, y);
			return 3;
		default :
			printf("problem in identifyFinger");
			break;
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
		}
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


