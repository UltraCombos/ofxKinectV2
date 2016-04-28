#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {

	//Uncomment for verbose info from libfreenect2
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofBackground(30, 30, 30);
	
	//see how many devices we have.
	ofxKinectV2 tmp;
	vector<ofxKinectV2::KinectDeviceInfo> deviceList = tmp.getDeviceList();

	//allocate for this many devices
	bundles.resize(deviceList.size());

	// setup gui
	{
		mSettings.setName("Settings");
		mSettings.add(gShowTextures.set("show_textures", true));
		mSettings.add(gThreshold.set("threshold", 128.0f, 0.0f, 255.0f));

		mGui = shared_ptr<ofxGuiGroup>(new ofxGuiGroup);
		loadGuiTheme(mGui, "fonts/theme.xml");

		mGui->setup("GUI");
		mGui->add(guiMessage.set("message", ""));
		mGui->add(mSettings);
	}


	int d = 0;
	size_t num = 512 * 424;
	vector<ofVec3f> vertices(num);
	vector<ofFloatColor> colors(num);
	for (auto& b : bundles)
	{
		b.kinect = shared_ptr<ofxKinectV2>(new ofxKinectV2);
		b.kinect->open(deviceList[d].serial);
		mGui->add(b.kinect->params);

		b.paramGroup.setName(deviceList[d].serial + "_Transition");
		b.paramGroup.add(b.position.set("position", ofVec3f(0), ofVec3f(-1), ofVec3f(1)));
		b.paramGroup.add(b.angle.set("angle", ofVec3f(0), ofVec3f(-90), ofVec3f(90)));
		mGui->add(b.paramGroup);
		d++;
	}
	
	mGui->loadFromFile(guiFilename);

	mCamera = shared_ptr<ofEasyCam>(new ofEasyCam);
	mCamera->setupPerspective(true, 45, 0.1, 10);
	mCamera->setAspectRatio(16.0f / 9.0f);
	//mCamera->setPosition(0, 0, FBO_WIDTH * 1.2f);
	mCamera->setDistance(2.0f);
}

//--------------------------------------------------------------
void ofApp::update() {
	ofSetWindowTitle(ofVAArgsToString("ofxKinectV2: %3.2f", ofGetFrameRate()));


	for (auto& b : bundles)
	{
		if (gShowTextures)
			b.kinect->updateTexture(&b.color, &b.ir, &b.depth, &b.aligned);

		b.numIndices = b.kinect->getVbo(b.vbo);
		//printf("vbo vertices: %i, indices: %i\n", b.vbo.getNumVertices(), b.numIndices);
	}

}

//--------------------------------------------------------------
void ofApp::draw() {

	if (gShowTextures)
	{
		ofPushMatrix();
		float color_w = 1920.0 * 424 / 1080;
		auto rect = getCenteredRect(color_w + 512 + 512 + 512, 424 * bundles.size(), ofGetWidth(), ofGetHeight(), false);
		rect.y = 0;
		float h = rect.height / bundles.size();
		float sc = h / 424.0f;
		ofTranslate(rect.position);
		for (auto& b : bundles)
		{
			if (!b.color.isAllocated() || !b.ir.isAllocated() || !b.depth.isAllocated() || !b.aligned.isAllocated())
				continue;
			ofPushMatrix();
			ofScale(sc, sc);
			b.color.draw(0, 0, color_w, 424);
			ofTranslate(color_w, 0);
			b.ir.draw(0, 0);
			ofTranslate(b.ir.getWidth(), 0);
			b.depth.draw(0, 0);
			ofTranslate(b.depth.getWidth(), 0);
			b.aligned.draw(0, 0);
			ofPopMatrix();
			ofTranslate(0, h);
		}
		ofPopMatrix();
	}

	mCamera->begin();
	ofEnableDepthTest();
	//ofEnableBlendMode(OF_BLENDMODE_ADD);
	for (auto& b : bundles)
	{
		ofPushMatrix();
		ofTranslate(b.position);
		ofRotateX(b.angle.get().x);
		ofRotateY(b.angle.get().y);
		ofRotateZ(b.angle.get().z);
		b.vbo.drawElements(GL_TRIANGLES, b.numIndices);
		//b.vbo.draw(GL_POINTS, 0, b.vbo.getNumVertices());
		ofPopMatrix();
	}
	//ofEnableBlendMode(OF_BLENDMODE_ALPHA);
	ofDisableDepthTest();
	mCamera->end();

	if (bDebugVisible)
		mGui->draw();

}

//--------------------------------------------------------------
void ofApp::exit() {

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	auto toggleFullscreen = [&]()
	{
		ofToggleFullscreen();
		if (!(ofGetWindowMode() == OF_FULLSCREEN))
		{
			ofSetWindowShape(WIDTH, HEIGHT);
			auto pos = ofVec2f(ofGetScreenWidth() - WIDTH, ofGetScreenHeight() - HEIGHT) * 0.5f;
			ofSetWindowPosition(pos.x, pos.y);
		}
	};

	switch (key)
	{
	case OF_KEY_F1:
		bDebugVisible = !bDebugVisible;
		break;
	case OF_KEY_F11:
		toggleFullscreen();
		guiMessage = "fullscreen";
		break;
	case 's':
		mGui->saveToFile(guiFilename);
		guiMessage = "save-" + ofGetTimestampString("%M%S");
		break;
	case 'l':
		mGui->loadFromFile(guiFilename);
		guiMessage = "load-" + ofGetTimestampString("%M%S");
		break;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}

void ofApp::loadGuiTheme(std::shared_ptr<ofxGuiGroup> gui, string path)
{
	ofXml xml(path);
	gui->loadFont(xml.getValue("FONT_NAME"), xml.getIntValue("FONT_SIZE"));
	gui->setDefaultTextPadding(xml.getIntValue("TEXT_PADDING"));
	gui->setDefaultHeight(xml.getIntValue("HEIGHT"));

	string theme_name = xml.getValue("THEME_NAME");
	if (xml.exists(theme_name))
	{
		xml.setTo(theme_name);
		auto hexHeaderBackgroundColor = ofColor::fromHex(ofHexToInt(xml.getValue("HeaderBackgroundColor")));
		auto hexBackgroundColor = ofColor::fromHex(ofHexToInt(xml.getValue("BackgroundColor")));
		auto hexBorderColor = ofColor::fromHex(ofHexToInt(xml.getValue("BorderColor")));
		auto hexFillColor = ofColor::fromHex(ofHexToInt(xml.getValue("FillColor")));
		auto hexTextColor = ofColor::fromHex(ofHexToInt(xml.getValue("TextColor")));
		gui->setHeaderBackgroundColor(hexHeaderBackgroundColor);
		gui->setBorderColor(hexBorderColor);
		gui->setTextColor(hexTextColor);
		gui->setDefaultHeaderBackgroundColor(hexHeaderBackgroundColor);
		gui->setDefaultBackgroundColor(hexBackgroundColor);
		gui->setDefaultBorderColor(hexBorderColor);
		gui->setDefaultFillColor(hexFillColor);
		gui->setDefaultTextColor(hexTextColor);
	}
}
