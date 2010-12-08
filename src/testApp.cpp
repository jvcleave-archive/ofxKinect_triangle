#include "testApp.h"


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
bool doDrawTriangles = true;
bool doSimpleTriangles = true;
bool forceGuiDraw = true;
//
float smoothPct = 0.75f;
int tolerance = 4;

void testApp::setup()
{
	kinect.init();
	kinect.enableDepthNearValueWhite(true);
	kinect.setVerbose(true);
	kinect.close();
	kinect.open();
	//
	colorImg.allocate(videoWidth, videoHeight);
	grayImage.allocate(videoWidth, videoHeight);
	grayThresh.allocate(videoWidth, videoHeight);
	grayThreshFar.allocate(videoWidth, videoHeight);
	
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
	gui.addSlider("blur", blur, 0, 12);

	
}

void testApp::update(){
	kinect.update();
	colorImg = getKinectPixels();
	grayImage.setFromPixels(kinect.getDepthPixels(), videoWidth, videoHeight);

	grayThreshFar = grayImage;
	grayThresh = grayImage;
	grayThreshFar.threshold(farThreshold, true);
	grayThresh.threshold(nearThreshold);
	cvAnd(grayThresh.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
	contourFinder.findContours(grayImage, minArea, maxArea, numConsidered, findHoles);
	
	simpleContours.clear();
	triangle.clear();
	pointsToTriangulate.clear();
	for(int i=0; i<contourFinder.blobs.size(); i++)
	{
		
		if (contourFinder.blobs[i].nPts != -1) 
		{
			
			
			int numPoints = contourFinder.blobs[i].nPts;
			if (doDrawTriangles && !doSimpleTriangles) {
				triangle.triangulate(contourFinder.blobs[i].pts, numPoints);
			}
			
			contourReg.clear();
			contourSmooth.clear();
			contourSimple.clear();
			contourReg.assign(numPoints, ofxPoint2f());
			contourSmooth.assign(numPoints, ofxPoint2f());
			
			for(int j = 0; j < numPoints; j++){
				contourReg[j] = contourFinder.blobs[0].pts[j];
				
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
					doDrawTriangles = false;
					ofLog(OF_LOG_ERROR, "triangulation failed");
				}
			}
			//
			/*ofxBox2dLine lineStrip;
			
			
			for (float f = contourSimple.size()-1; f >=0; f--) {
				
				lineStrip.addPoint(contourSimple[f].x, imageHeight+contourSimple[f].y);
			}
			lineStrip.setWorld(myWorld);
			lineStrip.bIsLoop = true;
			lineStrip.createShapeWithOptions(lineFriction, lineRestitution, lineDensity);
			
			lineStrips.push_back(lineStrip);*/	
			
		}
	}
	
	
	if(!forceGuiDraw)
	{
		gui.hide();
	}else
	{		
		gui.show();
	}

}

void testApp::draw(){
	if (doDrawColorImage)
	{
		colorImg.draw(xPos, yPos, videoWidth, videoHeight);
		ofDrawBitmapString("doDrawColorImage", xPos, yPos);
	}
	if (doDrawGrayImage)
	{
		grayImage.draw(xPos, yPos, videoWidth, videoHeight);
	}
	if(doDrawKinectDepth)
	{
		kinect.drawDepth(xPos, yPos, videoWidth, videoHeight);
	}
	if(doDrawCountours)
	{
		contourFinder.draw(xPos, yPos, videoWidth, videoHeight);
	}
	if (doDrawSimpleContours)
	{
		ofPushStyle();
		
		for(int i=0; i<simpleContours.size(); i++)
		{
			ofSetColor(0, 255, 0);
			ofNoFill();
			ofBeginShape();
			for(int j = 0; j < simpleContours[i].size(); j++)
			{
				ofCurveVertex(simpleContours[i][j].x+xPos, simpleContours[i][j].y+yPos);
			}
			ofEndShape(true);
		}
		ofPopStyle();
	}
	ofPushStyle();
	if (doDrawTriangles) {
		triangle.draw(xPos, yPos);
	}
	ofPopStyle();
	gui.draw();

}

void testApp::keyPressed(int key){
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

void testApp::keyReleased(int key){

}

void testApp::mouseMoved(int x, int y ){

}

void testApp::mouseDragged(int x, int y, int button){

}

void testApp::mousePressed(int x, int y, int button){

}

void testApp::mouseReleased(int x, int y, int button){

}

void testApp::windowResized(int w, int h){

}
unsigned char * testApp::getKinectPixels()
{
	if(!useCalibrated)
	{
		//
		return kinect.getPixels();
	}else {
		return kinect.getCalibratedRGBPixels();
	}
	
}
