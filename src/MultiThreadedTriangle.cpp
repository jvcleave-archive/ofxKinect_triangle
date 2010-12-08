/*
 *  MultiThreadedTriangle.cpp
 *  ofxKinect_triangle
 *
 *  Created by jason van cleave on 12/5/10.
 *  Copyright 2010 . All rights reserved.
 *
 */


#include "MultiThreadedTriangle.h"


float videoWidth = 640;
float videoHeight = 480;

int blur = 0;
int threshold = 0;
int nearThreshold = 50;
int farThreshold  = 180;
int minArea = 0;
int maxArea = 300000;
int numConsidered = 10;
bool findHoles =false;

bool useCalibrated = false;


int xPos =250;
int yPos =100;
bool bLearnBakground = true;

//
bool doDrawColorImage = false;
bool doDrawGrayImage = false;
bool doDrawKinectDepth = false;
bool doDrawCountours = false;
bool doDrawSimpleContours = false;
bool doDrawTriangles = false;
bool doSimpleTriangles = true;
bool forceGuiDraw = true;
//
float smoothPct = 0.75f;
int tolerance = 4;
//
int numThreads = 16;
float thresholdStep = 2.0f;
int farThresholdStep;
bool hasStarted = false;

void MultiThreadedTriangle::setup()
{
	//ofSetFrameRate(30);
	threshold = 80;
	blur =0;
	farThresholdStep = 75;
	nearThreshold = thresholdStep;
	farThreshold  = farThresholdStep;
	maxArea = (videoWidth * videoHeight)/2;
	kinect.init();
	kinect.enableDepthNearValueWhite(true);
	kinect.setVerbose(true);
	kinect.close();
	kinect.open();
	//
	colorImg.allocate(videoWidth, videoHeight);

	
	gui.addToggle("doDrawColorImage", doDrawColorImage);
	gui.addToggle("doDrawGrayImage", doDrawGrayImage);
	gui.addToggle("doDrawKinectDepth", doDrawKinectDepth);
	gui.addToggle("doDrawCountours", doDrawCountours);
	gui.addToggle("doDrawSimpleContours", doDrawSimpleContours);
	gui.addToggle("doDrawTriangles", doDrawTriangles);
	gui.addToggle("doSimpleTriangles", doSimpleTriangles);
	gui.addSlider("smoothPct", smoothPct, 0.01f, 1.0f);
	gui.addSlider("tolerance", tolerance, 1, 40);
	
	//
	gui.addPage("BLOBS");
	gui.addSlider("minArea", minArea, 0.0, 1000);
	gui.addSlider("maxArea", maxArea, 0.0, 400000);
	gui.addSlider("numConsidered", numConsidered, 0, 100);
	gui.addToggle("findHoles", findHoles);
	gui.addSlider("threshold", threshold, 0.0, 200);
	gui.addSlider("nearThreshold", nearThreshold, 0, 80);
	gui.addSlider("farThreshold", farThreshold, nearThreshold, 200);
	gui.addSlider("thresholdStep", thresholdStep, 1.0f, 10.00f);
	gui.addSlider("farThresholdStep", farThresholdStep, 1, 200);
	gui.addSlider("blur", blur, 0, 4);
	for (int i=0; i<numThreads; i++) 
	{
		ContourFinderThread * thread = new ContourFinderThread();
		threads.push_back(thread);
		thread->initAndSleep();
	}

	
}

void MultiThreadedTriangle::update(){
	kinect.update();
	colorImg = getKinectPixels();
	bool isFrameNew = kinect.isFrameNew();

	if (hasStarted) 
	{
		if (isFrameNew) 
		{
			for(int i=0; i<threads.size(); i++)
			{
				if(threads[i]->isReady)
				{
					threads[i]->videoPixels = kinect.getDepthPixels();
					threads[i]->blur = blur*3;
					threads[i]->threshold = (numThreads*thresholdStep)*(i+1);
					threads[i]->nearThreshold = (numThreads*thresholdStep)*(i+1);
					threads[i]->farThreshold = (threads[i]->nearThreshold)+farThresholdStep;
					//cout << i << " thread threshold " << threads[i]->threshold <<endl;
					threads[i]->minArea = minArea;
					threads[i]->maxArea = maxArea;
					threads[i]->numConsidered = numConsidered;
					threads[i]->findHoles = findHoles;
					threads[i]->colorImg = colorImg;
					threads[i]->doDrawTriangles = doDrawTriangles;
					threads[i]->doSimpleTriangles = doSimpleTriangles;
					threads[i]->doDrawSimpleContours = doDrawSimpleContours;
					
					threads[i]->smoothPct = smoothPct;
					threads[i]->tolerance = tolerance;
					threads[i]->updateOnce();
				}
			}
		}
		
			
	}
	else 
	{
		for(int i=0; i<threads.size(); i++)
		{
			threads[i]->grayImage.setFromPixels(kinect.getDepthPixels(), videoWidth, videoHeight);
		}
		hasStarted = true;
	}
	
	
	
	
	if(!forceGuiDraw)
	{
		gui.hide();
	}else
	{		
		gui.show();
	}
	
}

void MultiThreadedTriangle::draw(){
	if (doDrawColorImage)
	{
		colorImg.draw(xPos, yPos, videoWidth, videoHeight);
		ofDrawBitmapString("doDrawColorImage", xPos, yPos);
	}
	if (doDrawGrayImage)
	{
		for(int i=0; i<numThreads; i++)
		{
			if (threads[i]->isReady)
			{
				drawImage(threads[i]->grayImage, videoWidth*i, yPos, videoWidth, videoHeight);
				ofPushStyle();
				ofSetColor(0, 255, 0);
				string info="";
				info+=("nearThreshold:"+ofToString(threads[i]->nearThreshold) +"\n");
				info+=("farThreshold:"+ofToString(threads[i]->farThreshold) +"\n");
				ofDrawBitmapString(info, videoWidth*i, yPos+20);
				ofPopStyle();
			}
		}
		
	}
	if(doDrawKinectDepth)
	{
		kinect.drawDepth(xPos, yPos, videoWidth, videoHeight);
	}
	if(doDrawCountours)
	{
		//contourFinder.draw(xPos, yPos, videoWidth, videoHeight);
		for(int i=0; i<numThreads; i++)
		{
			if (threads[i]->isReady)
			{
				threads[i]->contourFinder->draw( videoWidth*i, yPos, videoWidth, videoHeight);
			}
		}
	}
	if (doDrawSimpleContours)
	{
		ofPushStyle();
		for(int i=0; i<numThreads; i++)
		{
			if (threads[i]->isReady)
			{
				//threads[i]->contourFinder->draw( videoWidth*i, yPos, videoWidth, videoHeight);
				for(int j=0; j<threads[i]->simpleContours.size(); j++)
				{
					ofSetColor(0, 255, 0);
					ofNoFill();
					ofBeginShape();
					cout << "threads[i]->simpleContours[j].size()" << threads[i]->simpleContours[j].size() << endl;
					for(int k = 0; k < threads[i]->simpleContours[j].size(); k++)
					{
						ofCurveVertex(threads[i]->simpleContours[j][k].x+(videoWidth*i), threads[i]->simpleContours[j][k].y+yPos);
					}
					ofEndShape(true);
				}
				
			}
		}
		ofPopStyle();
	}
	ofPushStyle();
	if (doDrawTriangles) 
	{
		//triangle.draw(xPos, yPos);
		ofEnableAlphaBlending();
		for(int i=0; i<numThreads; i++)
		{
			if (threads[i]->isReady)
			{
				threads[i]->triangle.draw(xPos, yPos);
			}
		}
		ofDisableAlphaBlending();
		
	}
	ofPopStyle();
	gui.draw();
	
}
void MultiThreadedTriangle::drawImage(ofBaseDraws &baseDraw, int x, int y, int width, int height)
{
	baseDraw.draw(x, y, width, height);
}
void MultiThreadedTriangle::keyPressed(int key){
	switch (key)
	{
		case ' ':
			bLearnBakground = true;
			break;
		case '+':
			threshold ++;
			if (threshold > 255) threshold = 255;
			break;
		case '-':
			threshold --;
			if (threshold < 0) threshold = 0;
			break;
		case 'g':
			forceGuiDraw = !forceGuiDraw;
			glClear(GL_COLOR_BUFFER_BIT);
			break;
		case 'c':
			break;	
			
	}
}

void MultiThreadedTriangle::keyReleased(int key){
	
}

void MultiThreadedTriangle::mouseMoved(int x, int y ){
	
}

void MultiThreadedTriangle::mouseDragged(int x, int y, int button){
	
}

void MultiThreadedTriangle::mousePressed(int x, int y, int button){
	
}

void MultiThreadedTriangle::mouseReleased(int x, int y, int button){
	
}

void MultiThreadedTriangle::windowResized(int w, int h){
	
}
unsigned char * MultiThreadedTriangle::getKinectPixels()
{
	if(!useCalibrated)
	{
		//
		return kinect.getPixels();
	}else {
		return kinect.getCalibratedRGBPixels();
	}
	
}
