//
//  ofxKinectV2.h
//  kinectExample
//
//  Created by Theodore Watson on 6/23/14.
//
//

#pragma once

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/registration.h>
#include <libfreenect2/frame_listener_impl.h>

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
	bool isFrameNew() { return bNewFrame; }
	void updateTexture(ofTexture* color, ofTexture* ir = nullptr, ofTexture* depth = nullptr, ofTexture* aligned = nullptr);
	std::vector<ofVec4f>& getPointCloudVertices();
	std::vector<ofFloatColor>& getPointCloudColors();
	// return number of indices
	int getVbo(ofVbo& vbo);
	void close();

	ofParameterGroup params;
	ofParameter<float> minDistance;
	ofParameter<float> maxDistance;
	ofParameter<bool> bUseRawDepth;
	
protected:
	void threadedFunction();
	int openKinect(std::string serial);
	void closeKinect();
	
	bool bOpened = false;

	int indexFront = 0;
	int indexBack = 1;
	bool bNewFrame = false;
	
	std::vector<ofPixels> frameColor;
	std::vector<ofPixels> frameDepth;
	std::vector<ofFloatPixels> frameIr;
	std::vector<ofFloatPixels> frameRawDepth;
	std::vector<ofPixels> frameAligned;

	std::vector<std::vector<ofVec4f> > pcVertices;
	std::vector<std::vector<ofFloatColor> > pcColors;

private:
	libfreenect2::Freenect2 freenect2;

	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline *pipeline = 0;

	libfreenect2::FrameMap frames;

	libfreenect2::Registration* registration;
	libfreenect2::SyncMultiFrameListener* listener;

	const int DEPTH_WIDTH = 512;
	const int DEPTH_HEIGHT = 424;

	ofTexture depthTexture;
	ofShader computeIndices;
	ofBufferObject indicesBuffer;
	ofBufferObject atomicCounter;

	const char* comp_glsl = R"(
#version 430 core

layout(r32f, binding = 0) uniform readonly image2D src;
layout(r32f, binding = 1) uniform readonly image2D user;

layout(std430, binding = 0) buffer indices_buffer {
    int indices[];
};

layout(binding = 0) uniform atomic_uint mAtomicCounter;

uniform int bUseUserMap = 0;

int get_index(ivec2 p, int image_width)
{
	return p.y * image_width + p.x;
}

void push_index(uint i, int p1, int p2, int p3)
{
	indices[i + 0] = p1;
	indices[i + 1] = p2;
	indices[i + 2] = p3;
}

layout(local_size_x = 32, local_size_y = 8, local_size_z = 1) in;
void main()
{		
	ivec2 T = ivec2(gl_GlobalInvocationID.xy);
	if (T.x == 0 || T.y == 0) return;
	
	// set texcoord of corners
	ivec2 T_top_left = T + ivec2(-1, -1);
	ivec2 T_top = T + ivec2(0, -1);
	ivec2 T_left = T + ivec2(-1, 0);
	
	// get user of corners
	float user_ori = 1.0;
	float user_top_left = 1.0;
	float user_top = 1.0;
	float user_left = 1.0;
	if (bUseUserMap != 0)
	{
		user_ori = (imageLoad(user, T).r > 0.0) ? 1.0 : 0.0;
		user_top_left = (imageLoad(user, T_top_left).r > 0.0) ? 1.0 : 0.0;
		user_top = (imageLoad(user, T_top).r > 0.0) ? 1.0 : 0.0;
		user_left = (imageLoad(user, T_left).r > 0.0) ? 1.0 : 0.0;
	}
	
	// get depth of corners
	float depth = imageLoad(src, T).r * user_ori;
	float depth_top_left = imageLoad(src, T_top_left).r * user_top_left;
	float depth_top = imageLoad(src, T_top).r * user_top;
	float depth_left = imageLoad(src, T_left).r * user_left;
	
	// get index of corners
	ivec2 image_size = imageSize(src);
	int index = get_index(T, image_size.x);
	int index_top_left = get_index(T_top_left, image_size.x);
	int index_top = get_index(T_top, image_size.x);
	int index_left = get_index(T_left, image_size.x);
	
	const float MAX_NEAR = 100.0;
	
	// check edges
	bool has_top_edge = MAX_NEAR > abs(depth_top_left - depth_top);
	bool has_left_edge = MAX_NEAR > abs(depth_top_left - depth_left);
	bool has_right_edge = MAX_NEAR > abs(depth_top - depth);
	bool has_bottom_edge = MAX_NEAR > abs(depth_left - depth);
	
	int num_triangles = 0;

			// check top-right and left-bottom triangles
	if (depth_top_left != 0.0 && depth != 0.0)
	{
		if (has_top_edge && has_right_edge && depth_top != 0.0)
		{
			uint i = atomicCounterIncrement(mAtomicCounter) * 3;
			push_index(i, index, index_top, index_top_left);
			num_triangles++;
		}
		
		if (has_left_edge && has_bottom_edge && depth_left != 0.0)
		{
			uint i = atomicCounterIncrement(mAtomicCounter) * 3;
			push_index(i, index, index_top_left, index_left);
			num_triangles++;
		}
	}
	
	// skip if we have enough triangles
	if (num_triangles > 0) 
	{	
		while (num_triangles < 2)
		{
			uint i = atomicCounterIncrement(mAtomicCounter) * 3;
			push_index(i, 0, 0, 0);
			num_triangles++;
		}
		return;
	}
	
	// check top-left and right-bottom triangles
	if (depth_top != 0.0 && depth_left != 0.0)
	{
		if (has_top_edge && has_left_edge && depth_top_left != 0.0)
		{
			uint i = atomicCounterIncrement(mAtomicCounter) * 3;
			push_index(i, index_top, index_top_left, index_left);
		}
		
		if (has_right_edge && has_bottom_edge && depth != 0.0)
		{
			uint i = atomicCounterIncrement(mAtomicCounter) * 3;
			push_index(i, index_left, index, index_top);
		}
	}

			while (num_triangles < 2)
	{
		uint i = atomicCounterIncrement(mAtomicCounter) * 3;
		push_index(i, 0, 0, 0);
		num_triangles++;
	}
}

)";

};