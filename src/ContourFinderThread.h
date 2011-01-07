#pragma once

#include "ofMain.h"
#include "ofxRuiThread.h"
#include "ofxOpenCv.h"
//pre-lib update
class ContourFinderThread:public ofxRuiThread{
public:
	int count;
	bool isReady;
	int videoWidth;
	int videoHeight;
	int nearThreshold;
	int farThreshold;
	int threshold;
	int blur;
	ofxCvGrayscaleImage 	grayImage;
	ofxCvGrayscaleImage 	grayThresh;
	ofxCvGrayscaleImage 	grayThreshFar;
	ofxCvColorImage		colorImg;
	ofxCvGrayscaleImage alphaImage;

	int minArea;
	int maxArea;
	int numConsidered;
	bool findHoles;
	ofxCvContourFinder 	*contourFinder;
	bool bLearnBakground;
	bool bThreshWithOpenCV;
	unsigned char * videoPixels;
	unsigned char * pix;
	int numBlobs;
	//
	bool doSimpleTriangles;
	bool doDrawTriangles;
	vector<vector <ofxPoint2f> > simpleContours;
	
	vector <ofxPoint2f> contourReg;
	vector <ofxPoint2f> contourSmooth;
	vector <ofxPoint2f> contourSimple;
	contourSimplify contourSimp;

	ofxTriangle triangle;
	vector<ofPoint> pointsToTriangulate;
	float smoothPct;
	int tolerance;
	bool doDrawSimpleContours;
	ContourFinderThread(){
		numBlobs = 0;
		count = -1;
		isReady = false;
		bLearnBakground = false;
		bThreshWithOpenCV = false;
		videoWidth = 640;
		videoHeight = 480;
		blur = 0;
		threshold = 0;
		nearThreshold = 50;
		farThreshold  = 180;
		minArea = 0;
		maxArea = 300000;
		numConsidered = 10;
		findHoles = false;
		contourFinder = new ofxCvContourFinder();
		allocateImages();
		smoothPct = 0.75f;
		tolerance = 4;
		doSimpleTriangles = true;
		doDrawTriangles = false;
		doDrawSimpleContours = true;
		isReady = true;
	}
	void deleteImages()
	{
		grayImage.clear();
		grayThresh.clear();
		grayThreshFar.clear();
		colorImg.clear();
	}
	void allocateImages()
	{
		grayImage.allocate(videoWidth, videoHeight);
		grayThresh.allocate(videoWidth, videoHeight);
		grayThreshFar.allocate(videoWidth, videoHeight);
		colorImg.allocate(videoWidth, videoHeight);
		alphaImage.allocate(videoWidth, videoHeight);
		pix = grayImage.getPixels();
	}
	void updateThread(){
		isReady = false;
		grayImage.setFromPixels(videoPixels, videoWidth, videoHeight);
		
		if( bThreshWithOpenCV ){
			grayThreshFar = grayImage;
			grayThresh = grayImage;
			grayThreshFar.threshold(farThreshold, true);
			grayThresh.threshold(nearThreshold);
			//grayThresh.blur(blur);
			
			cvAnd(grayThresh.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
			
		}else{
			
			//or we do it ourselves - show people how they can work with the pixels
			
			pix = grayImage.getPixels();
			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			
			for(int i = 0; i < numPixels; i++){
				if( pix[i] > nearThreshold && pix[i] < farThreshold ){
					pix[i] = 255;
				}else{
					pix[i] = 0;
				}
			}
		}
		
		//update the cv image
		grayImage.flagImageChanged();
		grayImage.blur(blur);
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		//contourFinder->findContours(grayImage, 10, (640*480)/2, 20, false);

		contourFinder->findContours(grayImage, minArea, maxArea, numConsidered, findHoles);
		numBlobs = contourFinder->blobs.size();
		count++;	
		if(doDrawTriangles || doDrawSimpleContours)
		{
			triangulate();
		}
		isReady = true;
	}
	void triangulate()
	{
		simpleContours.clear();
		triangle.clear();
		pointsToTriangulate.clear();
		for(int i=0; i<contourFinder->blobs.size(); i++)
		{
			
			if (contourFinder->blobs[i].nPts != -1) 
			{
				
				
				int numPoints = contourFinder->blobs[i].nPts;
				if (doDrawTriangles && !doSimpleTriangles) {
					triangle.triangulate(contourFinder->blobs[i].pts, numPoints);
				}
				
				contourReg.clear();
				contourSmooth.clear();
				contourSimple.clear();
				contourReg.assign(numPoints, ofxPoint2f());
				contourSmooth.assign(numPoints, ofxPoint2f());
				
				for(int j = 0; j < numPoints; j++){
					contourReg[j] = contourFinder->blobs[i].pts[j];
					
				}
				
				contourSimp.smooth(contourReg, contourSmooth, smoothPct);
				contourSimp.simplify(contourSmooth, contourSimple, tolerance);
				simpleContours.push_back(contourSimple);
				
				
				if (doDrawTriangles && doSimpleTriangles)
				{
					for (float f = contourSimple.size()-1; f >=0; f--)
					{
						
						pointsToTriangulate.push_back(ofPoint(contourSimple[f].x, contourSimple[f].y));
					}
					try {
						triangle.triangulate(pointsToTriangulate, pointsToTriangulate.size());
						
					}
					catch (...) {
						//doDrawTriangles = false;
						ofLog(OF_LOG_ERROR, "triangulation failed");
					}
				}
			}
		}
	}

};
