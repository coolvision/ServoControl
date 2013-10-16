#include "testApp.h"
#include "ServoControl.h"

int frame_i = 0;

void testApp::setup() {

	ofSetVerticalSync(true);

	// setup serial
	serial.listDevices();
	vector < ofSerialDeviceInfo > deviceList = serial.getDeviceList();
	cout << "deviceList:" << endl;
	for (int i = 0; i < deviceList.size(); i++) {
		cout << deviceList[i].getDevicePath() << endl;
	}
	int baud = 9600;
	serial.setup(0, baud); //open the first device

	gui.setup(); // most of the time you don't need a name but don't forget to call setup

	for (int i = 0; i < n_servos; i++) {

		servos.push_back(new ServoControl());
		ServoControl *s = servos.back();

		s->setNumber(i);
		s->target.set("target", 6000, 2000, 10000);
		s->set_min.set("min", ServoControl::default_min, 500, 2500);
		s->set_max.set("max", ServoControl::default_max, 500, 2500);

		s->parameters.add(s->target);
		s->parameters.add(s->set_min);
		s->parameters.add(s->set_max);
		string name = "servo_" + ofToString(i);
		s->parameters.setName(name);
		s->setSerial(&serial);

		gui.add(s->parameters);

		s->reset_range.addListener(s, &ServoControl::resetRange);
		s->get_error.addListener(s, &ServoControl::getError);
		s->get_target.addListener(s, &ServoControl::getTarget);

		s->target.addListener(s, &ServoControl::updateTarget);

		gui.getGroup(name).add(s->reset_range.setup("reset"));
	}


	ofSetLogLevel(OF_LOG_VERBOSE);

	// enable depth->video image calibration
	kinect.setRegistration(true);
	kinect.init();
	kinect.open();		// opens first available kinect

	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}

	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);

	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;

	ofSetFrameRate(60);

	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);

	// start from the front
	bDrawPointCloud = false;

}

void testApp::exit() {

    kinect.close();
}

void testApp::update() {

    frame_i++;

    kinect.update();



    
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {

		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);

		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds

        if(bThreshWithOpenCV) {

			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);

        } else {

			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImage.getPixels();

			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}

		// update the cv images
		grayImage.flagImageChanged();

		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
	}
}

//--------------------------------------------------------------
void testApp::draw() {
    
	ofBackgroundGradient(ofColor::white, ofColor::gray);

	gui.draw();
    
    ofSetColor(255, 255, 255);

    // draw from the live kinect
    int off_x = 250;

    kinect.drawDepth(off_x + 10, 10, 400, 300);
    kinect.draw(off_x + 420, 10, 400, 300);

    grayImage.draw(off_x + 10, 320, 400, 300);
    contourFinder.draw(off_x + 10, 320, 400, 300);

	// draw instructions
	ofSetColor(255, 255, 255);

    stringstream reportStream;
	reportStream << "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl;

	ofDrawBitmapString(reportStream.str(), off_x + 20, 652);
}

void testApp::drawPointCloud() {
//	int w = 640;
//	int h = 480;
//	ofMesh mesh;
//	mesh.setMode(OF_PRIMITIVE_POINTS);
//	int step = 2;
//	for(int y = 0; y < h; y += step) {
//		for(int x = 0; x < w; x += step) {
//			if(kinect.getDistanceAt(x, y) > 0) {
//				mesh.addColor(kinect.getColorAt(x,y));
//				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
//			}
//		}
//	}
//	glPointSize(3);
//	ofPushMatrix();
//	// the projected points are 'upside down' and 'backwards'
//	ofScale(1, -1, -1);
//	ofTranslate(0, 0, -1000); // center the points a bit
//	ofEnableDepthTest();
//	mesh.drawVertices();
//	ofDisableDepthTest();
//	ofPopMatrix();
}

void testApp::keyPressed(int key) {

	if (key == 's') {
		gui.saveToFile("settings.xml");
		//test_gui.saveToFile("test_settings.xml");
	}

	if (key == 'l') {
		gui.loadFromFile("settings.xml");
		//test_gui.loadFromFile("test_settings.xml");
	}
}



void testApp::keyReleased(int key) {
}
void testApp::mouseMoved(int x, int y) {
}
void testApp::mouseDragged(int x, int y, int button) {
}
void testApp::mousePressed(int x, int y, int button) {
}
void testApp::mouseReleased(int x, int y, int button) {
}
void testApp::windowResized(int w, int h) {
}
void testApp::gotMessage(ofMessage msg) {
}
void testApp::dragEvent(ofDragInfo dragInfo) {
}
