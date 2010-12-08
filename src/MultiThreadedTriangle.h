/*
 *  MultiThreadedTriangle.h
 *  ofxKinect_triangle
 *
 *  Created by jason van cleave on 12/5/10.
 *  Copyright 2010 . All rights reserved.
 *
 */



#ifndef _MULTITHREADED_APP
#define _MULTITHREADED_APP

#include "MultiThreadedTriangle.h"
#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxCvContourFinder.h"
#include "ofxOpenCv.h"
#include "ofxSimpleGuiToo.h"
#include "ofxVectorMath.h"
#include "contourSimplify.h"
#include "ofxTriangle.h"
#include "ContourFinderThread.h"
class MultiThreadedTriangle : public ofBaseApp{
	
public:
	void setup();
	void update();
	void draw();
	
	void keyPressed  (int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void drawImage(ofBaseDraws &baseDraw, int x, int y, int width, int height);

	//
	unsigned char * getKinectPixels();
	
	//
	ofxKinect kinect;
	//
	ofxCvColorImage			colorImg;
	
	ofxSimpleGuiToo gui;
	
	
	vector<ContourFinderThread *> threads;
};

#endif