/*
 *  ofxMotionTemplates.h
 *  opencvExample
 *
 *  Created by enrico <nausea> viola on 09/10/12.
 *  Copyright 2012 A.r.t. All rights reserved.
 *  MIT license
 *
 */

#include "ofMain.h"
#include "ofxOpenCv.h"

class ofxMotionTemplates{
public:
	ofxMotionTemplates();
	~ofxMotionTemplates();
	
	void setup(int w, int h, int thr=0, double sec=1);
	void update(IplImage * img);
	void draw();
	bool isMainGestureDone();
	
	double MHI_DURATION;
	double MAX_TIME_DELTA;
	double MIN_TIME_DELTA;
	
	int N;
	int last;
	int diff_threshold;
	
	IplImage ** buff;
	IplImage * mhi;
	IplImage * orient;
	IplImage * mask;
	IplImage * segmask;
	
	CvMemStorage * storage;
	
	CvSize size;
	
	ofxCvColorImage motion;
	
	typedef struct{
		int x;
		int y;
		float dim;
		float angle;
	}moTemp;
	
	moTemp mainMotion;
	vector <moTemp> motionFrag;
	int oldMain;
	
};