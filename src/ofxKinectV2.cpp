//
//  ofxKinectV2.cpp
//  kinectExample
//
//  Created by Theodore Watson on 6/23/14.
//
//

#include "ofxKinectV2.h"

//--------------------------------------------------------------------------------
ofxKinectV2::ofxKinectV2() {
	bNewFrame = false;
	bNewBuffer = false;
	bOpened = false;
	lastFrameNo = -1;

	if (ofGetLogLevel() == OF_LOG_VERBOSE) {
		libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
	}
	else {
		libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));
	}

	frameColor.resize(2);
	frameIr.resize(2);
	frameDepth.resize(2);
	frameAligned.resize(2);

	//set default distance range to 50cm - 600cm

	params.add(minDistance.set("minDistance", 500, 0, 12000));
	params.add(maxDistance.set("maxDistance", 6000, 0, 12000));
}

//--------------------------------------------------------------------------------
ofxKinectV2::~ofxKinectV2() {
	close();
}

//--------------------------------------------------------------------------------
static bool sortBySerialName(const ofxKinectV2::KinectDeviceInfo & A, const ofxKinectV2::KinectDeviceInfo & B) {
	return A.serial < B.serial;
}

//--------------------------------------------------------------------------------
vector<ofxKinectV2::KinectDeviceInfo> ofxKinectV2::getDeviceList() {
	vector<KinectDeviceInfo> devices;

	int num = freenect2.enumerateDevices();
	for (int i = 0; i < num; i++) {
		KinectDeviceInfo kdi;
		kdi.serial = freenect2.getDeviceSerialNumber(i);
		kdi.freenectId = i;
		devices.push_back(kdi);
	}

	ofSort(devices, sortBySerialName);
	for (int i = 0; i < num; i++) {
		devices[i].deviceId = i;
	}

	return devices;
}

//--------------------------------------------------------------------------------
unsigned int ofxKinectV2::getNumDevices() {
	return getDeviceList().size();
}

//--------------------------------------------------------------------------------
bool ofxKinectV2::open(unsigned int deviceId) {

	vector <KinectDeviceInfo> devices = getDeviceList();

	if (devices.size() == 0) {
		ofLogError("ofxKinectV2::open") << "no devices connected!";
		return false;
	}

	if (deviceId >= devices.size()) {
		ofLogError("ofxKinectV2::open") << " deviceId " << deviceId << " is bigger or equal to the number of connected devices " << devices.size() << endl;
		return false;
	}

	string serial = devices[deviceId].serial;
	return open(serial);
}

//--------------------------------------------------------------------------------
bool ofxKinectV2::open(string serial) {
	close();

	params.setName("kinectV2 " + serial);

	int retVal = openKinect(serial);

	if (retVal == 0) {
		lastFrameNo = -1;
		startThread(true);
	}
	else {
		return false;
	}

	bOpened = true;
	return true;
}

//--------------------------------------------------------------------------------
void ofxKinectV2::threadedFunction() 
{
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

	while (isThreadRunning()) 
	{
		if (!bOpened)
			continue;
		listener->waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		registration->apply(rgb, depth, &undistorted, &registered);

		frameColor[indexBack].setFromPixels(rgb->data, rgb->width, rgb->height, 4);
		for (auto pixel : frameColor[indexBack].getPixelsIter())
			std::swap(pixel[0], pixel[2]);
		frameIr[indexBack].setFromPixels((float *)ir->data, ir->width, ir->height, 1);
		frameDepth[indexBack].setFromPixels((float *)depth->data, depth->width, depth->height, 1);
		frameAligned[indexBack].setFromPixels(registered.data, registered.width, registered.height, 4);
		for (auto pixel : frameAligned[indexBack].getPixelsIter())
			std::swap(pixel[0], pixel[2]);

		listener->release(frames);


		std::lock_guard<std::mutex> guard(mutex);
		std::swap(indexFront, indexBack);
		bNewFrame = true;
		float delta_time = ofGetElapsedTimef() - timestamp;
		timestamp = ofGetElapsedTimef();
		printf("new frame cost %f seconds\n", delta_time);
	}
}

//--------------------------------------------------------------------------------
void ofxKinectV2::update(bool convertDepthPix) {
	if (ofGetFrameNum() != lastFrameNo) {
		bNewFrame = false;
		lastFrameNo = ofGetFrameNum();
	}
	if (bNewBuffer) {

		std::lock_guard<std::mutex> guard(mutex);
		rgbPix = rgbPixelsFront;
#if 1
		depthPix = depthPixelsFront;
		bNewBuffer = false;
#else
		rawDepthPixels = depthPixelsFront;
		bNewBuffer = false;

		if (rawDepthPixels.size() > 0 && convertDepthPix) {
			if (depthPix.getWidth() != rawDepthPixels.getWidth()) {
				depthPix.allocate(rawDepthPixels.getWidth(), rawDepthPixels.getHeight(), 1);
			}

			float * pixelsF = rawDepthPixels.getData();
			unsigned char * pixels = depthPix.getData();

			for (int i = 0; i < depthPix.size(); i++) {
				pixels[i] = ofMap(rawDepthPixels[i], minDistance, maxDistance, 255, 0, true);
				if (pixels[i] == 255) {
					pixels[i] = 0;
				}
			}

		}
#endif

		bNewFrame = true;
	}
}

void ofxKinectV2::updateTexture(std::shared_ptr<ofTexture> color, std::shared_ptr<ofTexture> ir, std::shared_ptr<ofTexture> depth, std::shared_ptr<ofTexture> aligned)
{
	std::lock_guard<std::mutex> guard(mutex);
	if (!bNewFrame || !color || !ir || !depth || !aligned)
		return;
	if (frameColor[indexFront].isAllocated())
	{
//		if (!color->isAllocated())
//			color->allocate(1920, 1080, GL_RGBA);
		color->loadData(frameColor[indexFront]);
	}
	if (frameIr[indexFront].isAllocated())
	{
//		if (!ir->isAllocated())
//			ir->allocate(512, 424, GL_R32F);
		ir->loadData(frameIr[indexFront]);
	}
	if (frameDepth[indexFront].isAllocated())
	{
//		if (!depth->isAllocated())
//			depth->allocate(512, 424, GL_R32F);
		depth->loadData(frameDepth[indexFront]);
	}
	if (frameDepth[indexFront].isAllocated())
	{
		aligned->loadData(frameAligned[indexFront]);
	}

	bNewFrame = false;
	printf("new texture\n");
}

//--------------------------------------------------------------------------------
bool ofxKinectV2::isFrameNew() {
	return bNewFrame;
}

//--------------------------------------------------------------------------------
ofPixels& ofxKinectV2::getDepthPixels() {
	return depthPix;
}

//--------------------------------------------------------------------------------
ofFloatPixels& ofxKinectV2::getRawDepthPixels() {
	return rawDepthPixels;
}

//--------------------------------------------------------------------------------
ofPixels& ofxKinectV2::getRgbPixels() {
	return rgbPix;
}

//--------------------------------------------------------------------------------
void ofxKinectV2::close() {
	if (!bOpened)
		return;

	waitForThread(true);
	closeKinect();
	bOpened = false;
}

int ofxKinectV2::openKinect(std::string serial)
{
	pipeline = new libfreenect2::CpuPacketPipeline();
	//pipeline = new libfreenect2::OpenGLPacketPipeline();
	//pipeline = new libfreenect2::OpenCLPacketPipeline();

	if (pipeline)
	{
		dev = freenect2.openDevice(serial, pipeline);
	}

	if (dev == 0)
	{
		ofLogError("ofxKinectV2::openKinect") << "failure opening device with serial " << serial;
		return -1;
	}

	listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

	dev->setColorFrameListener(listener);
	dev->setIrAndDepthFrameListener(listener);
	dev->start();

	ofLogVerbose("ofxKinectV2::openKinect") << "device serial: " << dev->getSerialNumber();
	ofLogVerbose("ofxKinectV2::openKinect") << "device firmware: " << dev->getFirmwareVersion();

	registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

	bOpened = true;

	return 0;
}

void ofxKinectV2::closeKinect()
{
	listener->release(frames);

	// TODO: restarting ir stream doesn't work!
	// TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
	dev->stop();
	dev->close();

	delete listener;
	listener = NULL;
	
	delete registration;
	registration = NULL;

	bOpened = false;
}