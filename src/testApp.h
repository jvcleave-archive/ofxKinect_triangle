#ifndef _TEST_APP
#define _TEST_APP


#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxCvContourFinder.h"
#include "ofxOpenCv.h"
#include "ofxSimpleGuiToo.h"
#include "ofxVectorMath.h"
#include "contourSimplify.h"
#include "ofxTriangle.h"
class testApp : public ofBaseApp{

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
	
		//
		unsigned char * getKinectPixels();

		//
		ofxKinect kinect;
		//
		ofxCvColorImage			colorImg;
		ofxCvGrayscaleImage 	grayImage;
		ofxCvGrayscaleImage 	grayThresh;
		ofxCvGrayscaleImage 	grayThreshFar;

		ofxCvContourFinder		contourFinder;

		ofxSimpleGuiToo gui;
		
		contourSimplify contourSimp;

		vector<vector <ofxPoint2f> > simpleContours;
	
		vector <ofxPoint2f> contourReg;
		vector <ofxPoint2f> contourSmooth;
		vector <ofxPoint2f> contourSimple;
		ofxTriangle triangle;
		vector<ofPoint> pointsToTriangulate;

};

#endif
