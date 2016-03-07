//
//  ofxKinectV2.cpp
//  kinectExample
//
//  Created by Theodore Watson on 6/23/14.
//
//

#include "ofxKinectV2.h"
#include <GLFW/glfw3.h>
#include <libfreenect2/logger.h>

//--------------------------------------------------------------------------------
ofxKinectV2::ofxKinectV2() {
	bNewFrame = false;
	bOpened = false;

	if (ofGetLogLevel() == OF_LOG_VERBOSE) {
		libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
	}
	else {
		libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));
	}

	frameColor.resize(2);
	frameDepth.resize(2);
	frameIr.resize(2);
	frameRawDepth.resize(2);
	frameAligned.resize(2);
	pcVertices.resize(2, vector<ofVec3f>(525 * 412));
	pcColors.resize(2, vector<ofFloatColor>(525 * 412));

	//set default distance range to 50cm - 600cm

	params.add(minDistance.set("minDistance", 500, 0, 12000));
	params.add(maxDistance.set("maxDistance", 6000, 0, 12000));
	params.add(bUseRawDepth.set("rawDepth", false));
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

	if (retVal == 0) startThread(true);
	else return false;

	bOpened = true;
	return true;
}

//--------------------------------------------------------------------------------
void ofxKinectV2::threadedFunction() 
{
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

	while (isThreadRunning()) 
	{
		if (!bOpened) continue;

		listener->waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		registration->apply(rgb, depth, &undistorted, &registered);

		frameColor[indexBack].setFromPixels(rgb->data, rgb->width, rgb->height, 4);
		frameIr[indexBack].setFromPixels((float *)ir->data, ir->width, ir->height, 1);
		frameRawDepth[indexBack].setFromPixels((float *)depth->data, depth->width, depth->height, 1);
		frameAligned[indexBack].setFromPixels(registered.data, registered.width, registered.height, 4);
		
		listener->release(frames);
		for (auto pixel : frameColor[indexBack].getPixelsIter()) // swap rgb
			std::swap(pixel[0], pixel[2]);
		for (auto pixel : frameIr[indexBack].getPixelsIter()) // downscale to 0-1
			pixel[0] /= 65535.0f;
		for (auto pixel : frameAligned[indexBack].getPixelsIter()) // swap rgb
			std::swap(pixel[0], pixel[2]);

		if(!bUseRawDepth) 
		{
			auto& depth = frameDepth[indexBack];
			if ((depth.getWidth() != frameRawDepth[indexBack].getWidth()) || (depth.getHeight() != frameRawDepth[indexBack].getHeight()))
			{
				depth.allocate(frameRawDepth[indexBack].getWidth(), frameRawDepth[indexBack].getHeight(), 3);
			}
			std::pair<float, float> minmax(0.0, 0.8);
			for (int i = 0; i < frameRawDepth[indexBack].size(); i++) 
			{
				float hue = ofMap(frameRawDepth[indexBack][i], minDistance, maxDistance, minmax.first, minmax.second, true);
				if (hue == minmax.first || hue == minmax.second) depth.setColor(i * 3, ofColor(0));
				else depth.setColor(i * 3, ofFloatColor::fromHsb(hue, 0.9, 0.9));
			}
		}
		
		// get point cloud
		{
			float rgbPix = 0;
			size_t i = 0;
			for (int y = 0; y < 424; y++) 
			{
				for (int x = 0; x < 512; x++) 
				{
					auto& pt = pcVertices[indexBack][i];
					registration->getPointXYZRGB(&undistorted, &registered, y, x, pt.x, pt.y, pt.z, rgbPix);
					const uint8_t *p = reinterpret_cast<uint8_t*>(&rgbPix);
					pcColors[indexBack][i] = ofColor(p[2], p[1], p[0]);
					i++;
				}
			}
		}
		

		std::lock_guard<std::mutex> guard(mutex);
		std::swap(indexFront, indexBack);
		bNewFrame = true;
	}
}


void ofxKinectV2::updateTexture(std::shared_ptr<ofTexture> color, std::shared_ptr<ofTexture> ir, std::shared_ptr<ofTexture> depth, std::shared_ptr<ofTexture> aligned)
{
	std::lock_guard<std::mutex> guard(mutex);
	if (!bNewFrame || !color || !ir || !depth || !aligned)
		return;

	if (frameColor[indexFront].isAllocated())
		color->loadData(frameColor[indexFront]);

	if (frameIr[indexFront].isAllocated())
		ir->loadData(frameIr[indexFront]);

	if (frameDepth[indexFront].isAllocated())
		depth->loadData(frameDepth[indexFront]);

	if (frameRawDepth[indexFront].isAllocated())
		aligned->loadData(frameAligned[indexFront]);

	bNewFrame = false;
	std::printf("new texture\n");
}

std::vector<ofVec3f> ofxKinectV2::getPointCloudVertices()
{
	std::lock_guard<std::mutex> guard(mutex);
	return pcVertices[indexFront];
}

std::vector<ofFloatColor> ofxKinectV2::getPointCloudColors()
{
	std::lock_guard<std::mutex> guard(mutex);
	return pcColors[indexFront];
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
	//pipeline = new libfreenect2::CpuPacketPipeline();
	//ofAppGLFWWindow * glfwWindow = (ofAppGLFWWindow*)ofGetWindowPtr();
	//GLFWwindow* window = glfwWindow->getGLFWWindow();
	//pipeline = new libfreenect2::OpenGLPacketPipeline(window);
	pipeline = new libfreenect2::OpenCLPacketPipeline();

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

	dev->stop();
	dev->close();

	delete listener;
	listener = NULL;
	
	delete registration;
	registration = NULL;

	bOpened = false;
}