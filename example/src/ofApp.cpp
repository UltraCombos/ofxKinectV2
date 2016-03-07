#include "ofApp.h"

//NOTE: if you are unable to connect to your device on OS X, try unplugging and replugging in the power, while leaving the USB connected.
//ofxKinectV2 will only work if the NUI Sensor shows up in the Superspeed category of the System Profiler in the USB section.

//On OS X if you are not using the example project. Make sure to add OpenCL.framework to the Link Binary With Library Build Phase 
//and change the line in Project.xcconfig to OTHER_LDFLAGS = $(OF_CORE_LIBS) $(OF_CORE_FRAMEWORKS) -framework OpenCL

//--------------------------------------------------------------
void ofApp::setup() {

	//Uncomment for verbose info from libfreenect2
	ofSetLogLevel(OF_LOG_VERBOSE);

	ofBackground(30, 30, 30);
#if 1
	//see how many devices we have.
	ofxKinectV2 tmp;
	vector<ofxKinectV2::KinectDeviceInfo> deviceList = tmp.getDeviceList();

	//allocate for this many devices
	kinects.resize(deviceList.size());
	texColor.resize(kinects.size());
	texIr.resize(kinects.size());
	texDepth.resize(kinects.size());
	texAligned.resize(kinects.size());

	panel.setup("", "settings.xml", 10, 100);

	//Note you don't have to use ofxKinectV2 as a shared pointer, but if you want to have it in a vector ( ie: for multuple ) it needs to be.
	for (size_t d = 0; d < kinects.size(); d++) {
		kinects[d] = shared_ptr<ofxKinectV2>(new ofxKinectV2);
		kinects[d]->open(deviceList[d].serial);
		texColor[d] = shared_ptr<ofTexture>(new ofTexture);
		texIr[d] = shared_ptr<ofTexture>(new ofTexture);
		texDepth[d] = shared_ptr<ofTexture>(new ofTexture);
		texAligned[d] = shared_ptr<ofTexture>(new ofTexture);

		panel.add(kinects[d]->params);
	}

	panel.loadFromFile("settings.xml");
#else
	pipeline = new libfreenect2::CpuPacketPipeline();
	string serial = freenect2.getDefaultDeviceSerialNumber();
	dev = freenect2.openDevice(serial, pipeline);
	listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
	dev->setColorFrameListener(listener);
	dev->setIrAndDepthFrameListener(listener);
	dev->start();
	ofLogVerbose("ofxKinectV2::openKinect") << "device serial: " << dev->getSerialNumber();
	ofLogVerbose("ofxKinectV2::openKinect") << "device firmware: " << dev->getFirmwareVersion();
	registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
#endif
}

//--------------------------------------------------------------
void ofApp::update() {
	ofSetWindowTitle(ofVAArgsToString("ofxKinectV2: %3.2f", ofGetFrameRate()));


#if 1
	for (size_t d = 0; d < kinects.size(); d++)
	{
		kinects[d]->updateTexture(texColor[d], texIr[d], texDepth[d], texAligned[d]);
	}
#else
	listener->waitForNewFrame(frames);
	listener->release(frames);
#endif
}

//--------------------------------------------------------------
void ofApp::draw() {
//	ofDrawBitmapString("ofxKinectV2: Work in progress addon.\nBased on the excellent work by the OpenKinect libfreenect2 team\n\n-Requires USB 3.0 port ( superspeed )\n-Requires patched libusb. If you have the libusb from ofxKinect ( v1 ) linked to your project it will prevent superspeed on Kinect V2", 10, 14);
#if 1
	ofPushMatrix();
	float color_w = 1920.0 * 424 / 1080;
	auto rect = getCenteredRect(color_w + 512 + 512 + 512, 424 * kinects.size(), ofGetWidth(), ofGetHeight(), false);
	float h = rect.height / kinects.size();
	float sc = h / 424.0f;
	ofTranslate(rect.position);
	for (size_t d = 0; d < kinects.size(); d++) {
		if (!texDepth[d]->isAllocated() || !texColor[d]->isAllocated() || !texIr[d]->isAllocated() || !texAligned[d]->isAllocated())
			continue;
		ofPushMatrix();
		ofScale(sc, sc);
		texColor[d]->draw(0, 0, color_w, 424);
		ofTranslate(color_w, 0);
		texIr[d]->draw(0, 0);
		ofTranslate(texIr[d]->getWidth(), 0);
		texDepth[d]->draw(0, 0);
		ofTranslate(texDepth[d]->getWidth(), 0);
		texAligned[d]->draw(0, 0);
		ofPopMatrix();
		ofTranslate(0, h);
	}
	ofPopMatrix();
	panel.draw();
#endif
}

//--------------------------------------------------------------
void ofApp::exit() {
#if 0
	listener->release(frames);

	// TODO: restarting ir stream doesn't work!
	// TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
	dev->stop();
	dev->close();

	delete listener;
	listener = NULL;

	delete registration;
	registration = NULL;
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

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
