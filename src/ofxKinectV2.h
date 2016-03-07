//
//  ofxKinectV2.h
//  kinectExample
//
//  Created by Theodore Watson on 6/23/14.
//
//

#pragma once

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include "ofMain.h"

class ofxKinectV2 : public ofThread {

public:

	struct KinectDeviceInfo {
		string serial;
		int deviceId;   //if you have the same devices plugged in device 0 will always be the same Kinect
		int freenectId; //don't use this one - this is the index given by freenect2 - but this can change based on order device is plugged in
	};

	ofxKinectV2();
	~ofxKinectV2();

	//for some reason these can't be static - so you need to make a tmp object to query them
	vector<KinectDeviceInfo> getDeviceList();
	unsigned int getNumDevices();

	bool open(string serial);
	bool open(unsigned int deviceId = 0);
	void update(bool convertDepthPix = true);
	void updateTexture(std::shared_ptr<ofTexture> color, std::shared_ptr<ofTexture> ir, std::shared_ptr<ofTexture> depth, std::shared_ptr<ofTexture> aligned);
	void close();

	bool isFrameNew();

	ofPixels& getDepthPixels();
	ofPixels& getRgbPixels();
	ofFloatPixels& getRawDepthPixels();

	ofParameterGroup params;
	ofParameter<float> minDistance;
	ofParameter<float> maxDistance;

protected:
	void threadedFunction();
	int openKinect(std::string serial);
	void closeKinect();

	ofPixels rgbPix;
	ofPixels depthPix;
	ofFloatPixels rawDepthPixels;
	ofPixels rgbPixelsBack;
	ofPixels rgbPixelsFront;
	ofFloatPixels depthPixelsBack;
	ofFloatPixels depthPixelsFront;

	bool bNewBuffer = false;
	
	bool bOpened = false;

	int indexFront = 0;
	int indexBack = 1;
	bool bNewFrame = false;
	float timestamp = 0.0f;
	std::vector<ofPixels> frameColor;
	std::vector<ofFloatPixels> frameIr;
	std::vector<ofFloatPixels> frameDepth;
	std::vector<ofPixels> frameAligned;

	int lastFrameNo;

private:
	libfreenect2::Freenect2 freenect2;

	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline *pipeline = 0;

	libfreenect2::FrameMap frames;

	libfreenect2::Registration* registration;
	libfreenect2::SyncMultiFrameListener* listener;
};