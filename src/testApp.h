#pragma once

#include "ofMain.h"
#include "ofxGui.h"

#include "ofxOpenCv.h"
#include "ofxKinect.h"

class ServoControl;

class testApp: public ofBaseApp {
public:

	static const int n_servos = 6;

	void setup();
	void update();
	void draw();

	void exit();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofSerial serial;

	vector<ServoControl *> servos;

//    ofParameter<int> target;

	ofxPanel gui;

    void drawPointCloud();

	ofxKinect kinect;
	ofxCvColorImage colorImg;

	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image

	ofxCvContourFinder contourFinder;

	bool bThreshWithOpenCV;
	bool bDrawPointCloud;

	int nearThreshold;
	int farThreshold;

	int angle;

	// used for viewing the point cloud
	ofEasyCam easyCam;
};

