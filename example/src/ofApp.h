#pragma once

#define LIBUSB_DEBUG 4

#include "ofMain.h"
#include "ofxKinectV2.h"
#include "ofxGui.h"

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofxPanel panel;

	std::vector<std::shared_ptr<ofxKinectV2>> kinects;

	std::vector<std::shared_ptr<ofTexture>> texColor;
	std::vector<std::shared_ptr<ofTexture>> texIr;
	std::vector<std::shared_ptr<ofTexture>> texDepth;
	


};
