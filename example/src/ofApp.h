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

	void loadGuiTheme(std::shared_ptr<ofxGuiGroup> gui, string path);

	enum {
		WIDTH = 1280,
		HEIGHT = 720
	};

	bool bDebugVisible = true;
	std::shared_ptr<ofxGuiGroup> mGui;
	ofParameter<string> guiMessage;
	const string guiFilename = "settings.xml";
	ofParameterGroup mSettings;
	ofParameter<bool> gShowTextures;
	ofParameter<float> gThreshold;

	struct KinectBundle
	{
		std::shared_ptr<ofxKinectV2> kinect;
		ofTexture color;
		ofTexture ir;
		ofTexture depth;
		ofTexture aligned;
		ofVbo vbo;
		int numIndices = 0;

		ofParameterGroup paramGroup;
		ofParameter<ofVec3f> position;
		ofParameter<ofVec3f> angle;
	};
	std::vector<KinectBundle> bundles;


	std::shared_ptr<ofEasyCam> mCamera;

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
