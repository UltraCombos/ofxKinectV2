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
	pcVertices.resize(2, vector<ofVec4f>(DEPTH_WIDTH * DEPTH_HEIGHT));
	pcColors.resize(2, vector<ofFloatColor>(DEPTH_WIDTH * DEPTH_HEIGHT));

	//set default distance range to 50cm - 600cm

	params.add(minDistance.set("minDistance", 500, 0, 12000));
	params.add(maxDistance.set("maxDistance", 6000, 0, 12000));
	params.add(bUseRawDepth.set("rawDepth", false));

	computeIndices.unload();
	computeIndices.setupShaderFromSource(GL_COMPUTE_SHADER, comp_glsl);
	computeIndices.linkProgram();

	depthTexture.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, GL_R32F);

	vector<int> indices((DEPTH_WIDTH - 1) * (DEPTH_HEIGHT - 1) * 6, 0);
	indicesBuffer.allocate(indices, GL_DYNAMIC_DRAW);

	atomicCounter.allocate();
	int data = 0;
	atomicCounter.setData(sizeof(int), &data, GL_DYNAMIC_COPY);
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

	params.setName("kinectV2_" + serial);

	int retVal = openKinect(serial);

	if (retVal == 0) startThread(true);
	else return false;

	bOpened = true;
	return true;
}

//--------------------------------------------------------------------------------
void ofxKinectV2::threadedFunction() 
{
	libfreenect2::Frame undistorted(DEPTH_WIDTH, DEPTH_HEIGHT, 4), registered(DEPTH_WIDTH, DEPTH_HEIGHT, 4);

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
			auto& raw_depth = frameRawDepth[indexBack];
			if ((depth.getWidth() != raw_depth.getWidth()) || (depth.getHeight() != raw_depth.getHeight()))
			{
				depth.allocate(raw_depth.getWidth(), raw_depth.getHeight(), 3);
			}
			std::pair<float, float> minmax(0.0, 0.8);
			for (int i = 0; i < raw_depth.size(); i++)
			{
				float hue = ofMap(raw_depth[i], minDistance, maxDistance, minmax.first, minmax.second, true);
				if (hue == minmax.first || hue == minmax.second) depth.setColor(i * 3, ofColor(0));
				else depth.setColor(i * 3, ofFloatColor::fromHsb(hue, 0.9, 0.9));
			}
		}
		
		// get point cloud
		{
			float rgbPix = 0;
			size_t i = 0;
			for (int y = 0; y < DEPTH_HEIGHT; y++)
			{
				for (int x = 0; x < DEPTH_WIDTH; x++)
				{
					auto& pt = pcVertices[indexBack][i];
					registration->getPointXYZRGB(&undistorted, &registered, y, x, pt.x, pt.y, pt.z, rgbPix);
					pt.z *= -1.0f;
					pt.w = 1.0f;
					const uint8_t *p = reinterpret_cast<uint8_t*>(&rgbPix);
					pcColors[indexBack][i] = ofColor(p[2], p[1], p[0]);
					i++;
				}
			}
		}
		
		//while (bNewFrame)
		{
			// wait for main thread
		}

		std::lock_guard<std::mutex> guard(mutex);
		std::swap(indexFront, indexBack);
		bNewFrame = true;
	}
}


void ofxKinectV2::updateTexture(ofTexture* color, ofTexture* ir, ofTexture* depth, ofTexture* aligned)
{
	std::lock_guard<std::mutex> guard(mutex);
	if (!bNewFrame)
		return;

	if (frameColor[indexFront].isAllocated() && color)
		color->loadData(frameColor[indexFront]);

	if (frameIr[indexFront].isAllocated() && ir)
		ir->loadData(frameIr[indexFront]);

	if (frameDepth[indexFront].isAllocated() && depth)
		depth->loadData(frameDepth[indexFront]);

	if (frameRawDepth[indexFront].isAllocated() && aligned)
		aligned->loadData(frameAligned[indexFront]);

	bNewFrame = false;
}

std::vector<ofVec4f>& ofxKinectV2::getPointCloudVertices()
{
	return pcVertices[indexFront];
}

std::vector<ofFloatColor>& ofxKinectV2::getPointCloudColors()
{
	return pcColors[indexFront];
}

int ofxKinectV2::getVbo(ofVbo& vbo)
{
	auto& vertices = pcVertices[indexFront];
	auto& colors = pcColors[indexFront];
	if (!vbo.getIsAllocated())
	{
		vector<ofVec2f> texCoords;
		for (int y = 0; y < DEPTH_HEIGHT; y++)
		{
			float coord_y = (float)y / DEPTH_HEIGHT;
			for (int x = 0; x < DEPTH_WIDTH; x++)
			{
				float coord_x = (float)x / DEPTH_HEIGHT;
				texCoords.emplace_back(coord_x, coord_y);
			}
		}
		vbo.setTexCoordData(&texCoords[0], texCoords.size(), GL_STATIC_DRAW);
		vbo.setVertexData(&vertices[0].x, 4, vertices.size(), GL_DYNAMIC_DRAW);
		vbo.setColorData(&colors[0], colors.size(), GL_DYNAMIC_DRAW);
		vbo.setIndexBuffer(indicesBuffer);
	}
	else
	{
		vbo.updateVertexData(&vertices[0].x, vertices.size());
		vbo.updateColorData(&colors[0].r, colors.size());
	}

	auto& depth = frameRawDepth[indexFront];
	if (depth.getWidth() && depth.getHeight())
	{
		{
			std::lock_guard<std::mutex> guard(mutex);
			depthTexture.loadData(&depth[0], DEPTH_WIDTH, DEPTH_HEIGHT, GL_RED);

			//vector<float> dd(512 * 424);
			//memcpy(&dd[0], depth.getData(), sizeof(float) * dd.size());
		}
		
		depthTexture.bindAsImage(0, GL_READ_ONLY);
		indicesBuffer.bindBase(GL_SHADER_STORAGE_BUFFER, 0);
		atomicCounter.bindBase(GL_ATOMIC_COUNTER_BUFFER, 0);

		computeIndices.begin();
		computeIndices.setUniform1i("bUseUserMap", 0);
		computeIndices.dispatchCompute(DEPTH_WIDTH / 32, DEPTH_HEIGHT / 8, 1);
		computeIndices.end();

		indicesBuffer.unbindBase(GL_SHADER_STORAGE_BUFFER, 0);
		atomicCounter.unbindBase(GL_ATOMIC_COUNTER_BUFFER, 0);

#if 0
		auto atomic = atomicCounter.map<int>(GL_READ_ONLY);
		int num = atomic[0];
		atomicCounter.unmap();
		cout << "atomic: " << num << endl;
#endif
		int data = 0;
		atomicCounter.updateData(0, sizeof(int), &data);
	}

	return indicesBuffer.size() / sizeof(int);
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