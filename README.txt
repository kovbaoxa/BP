1. Download openFrameworks
	https://openframeworks.cc/download/ 

2. Follow setup guide (openFramwork, qt creator,..)
	Note: When using qt creator, version 4.1 is required.
	http://download.qt.io/official_releases/qtcreator/4.1/4.1.0/


3. Download ofxKinectV2
	https://github.com/ofTheo/ofxKinectV2
	Requires extern libusb library from git repo (or libfreenect2 repo).
	Copy in openFrameworks home folder in addons.

4. Copy project folder in folder of_home_folder/apps/myApps

5. write in command line in project folder "make"

6. write in command line in project folder "make RunRelease"

optional:
7. draw.jar is debugging tool, which draws picture from txt file
	requires 512 collumns and 424 rows
	1st argument input file with numbers 0-255
	2st argument output

Required HW
	USB 3 port
	Kinect camera
