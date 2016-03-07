#pragma once

#include "ofMain.h"
#include "ofxKinectV2.h"
#include "ofxGui.h"

class ofApp : public ofBaseApp {

public:
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

#if 1
	ofxPanel panel;

	std::vector<std::shared_ptr<ofxKinectV2>> kinects;

	std::vector<std::shared_ptr<ofTexture>> texColor;
	std::vector<std::shared_ptr<ofTexture>> texIr;
	std::vector<std::shared_ptr<ofTexture>> texDepth;
	std::vector<std::shared_ptr<ofTexture>> texAligned;
#else
	libfreenect2::Freenect2 freenect2;

	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline *pipeline = 0;

	libfreenect2::FrameMap frames;

	libfreenect2::Registration* registration;
	libfreenect2::SyncMultiFrameListener* listener;
#endif

	ofRectangle getCenteredRect(int srcWidth, int srcHeight, int otherWidth = ofGetWidth(), int otherHeight = ofGetHeight(), bool isFill = true)
	{
		auto other = ofRectangle(0, 0, otherWidth, otherHeight);
		ofRectangle result;
		result.setFromCenter(other.getCenter(), srcWidth, srcHeight);
		float scaleBy;
		auto aspectAspect = result.getAspectRatio() / other.getAspectRatio();

		if ((isFill && aspectAspect <= 1.0f) || (!isFill && aspectAspect >= 1.0f))
			scaleBy = other.getWidth() / result.getWidth();
		else
			scaleBy = other.getHeight() / result.getHeight();

		result.scaleFromCenter(scaleBy);
		return result;
	}


	
};
