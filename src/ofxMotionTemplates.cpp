/*
 *  ofxMotionTemplates.cpp
 *  opencvExample
 *
 *  Created by enrico <nausea> viola on 09/10/12.
 *  Copyright 2012 A.r.t. All rights reserved.
 *  MIT license
 *
 */

#include "ofxMotionTemplates.h"

ofxMotionTemplates::ofxMotionTemplates(){

}

ofxMotionTemplates::~ofxMotionTemplates(){
	cvReleaseImage(&mhi);
	cvReleaseImage(&orient);
	cvReleaseImage(&mask);
	cvReleaseImage(&segmask);
	
	cvReleaseMemStorage(&storage);
	
	for(int i=0;i<N;i++){
		cvReleaseImage(&buff[i]);
	}
	
	motionFrag.clear();
}

void ofxMotionTemplates::setup(int w, int h, int thr, double sec){
	//secs
    MHI_DURATION = sec;//1;
    MAX_TIME_DELTA = sec/2;//0.5;
    MIN_TIME_DELTA = MAX_TIME_DELTA/10;//0.05;
	
    N = 4;
	
    //ring img buffer
    buff=0;
    last=0;
	
    mhi=0;
    orient=0;
    mask=0; //valid orientation mask
    segmask=0; //motion segmentation mask
	
    storage=0;
	
    motion.allocate(w,h);
    size = cvSize(w,h);
	
	
    buff = (IplImage**)malloc(N*sizeof(buff[0]));
    memset(buff,0,N*sizeof(buff[0]));
	
	for(int i=0;i<N;i++){
        cvReleaseImage(&buff[i]);
        buff[i] = cvCreateImage(size,IPL_DEPTH_8U,1);
        cvZero(buff[i]);
    }
	
    mhi = cvCreateImage(size,IPL_DEPTH_32F,1);
    cvZero(mhi);
    orient = cvCreateImage(size,IPL_DEPTH_32F,1);
    segmask = cvCreateImage(size,IPL_DEPTH_32F,1);
    mask = cvCreateImage(size,IPL_DEPTH_8U,1);
	
    storage = cvCreateMemStorage(0);
	
    diff_threshold = thr;
	
    mainMotion.x=w/2;
    mainMotion.y=h/2;
    mainMotion.dim=0;
    mainMotion.angle=0;
	
    oldMain=0;
}

void ofxMotionTemplates::update(IplImage * img){
	oldMain=mainMotion.dim;
	
    double timestamp = (double)clock()/CLOCKS_PER_SEC;
	
    int i, idx1 = last, idx2;
    IplImage * silh;
    CvSeq * seq;
    CvRect comp_rect;
    double count;
    double angle;
    CvPoint center;
    double magnitude;
    CvScalar color;
	
    img->nChannels>1?cvCvtColor(img,buff[last],CV_BGR2GRAY):cvCopy(img,buff[last]);
	
    idx2 = (last+1)%N; //last-(N-1)
    last = idx2;
	
    silh = buff[idx2];
    cvAbsDiff(buff[idx1],buff[idx2],silh);
	
    if(diff_threshold>0)cvThreshold(silh,silh,diff_threshold,1,CV_THRESH_BINARY);
	
    cvUpdateMotionHistory(silh,mhi,timestamp,MHI_DURATION);
	
    cvCvtScale(mhi,mask,255./MHI_DURATION,(MHI_DURATION-timestamp)*255./MHI_DURATION);
    cvZero(motion.getCvImage());
    cvCvtPlaneToPix(0,0,mask,0,motion.getCvImage());
    //cvCvtPlaneToPix(mask,0,0,0,motion.getCvImage());
    motion.flagImageChanged();
	
    cvCalcMotionGradient(mhi,mask,orient,MAX_TIME_DELTA,MIN_TIME_DELTA,3);
	
    cvClearMemStorage(storage);
	
    seq = cvSegmentMotion(mhi,segmask,storage,timestamp,MAX_TIME_DELTA);
	
    motionFrag.clear();
    for(i=-1;i<seq->total;i++){
		
        if(i<0){//whole img
            comp_rect = cvRect(0,0,size.width,size.height);
            color = CV_RGB(255,255,255);
            magnitude = 100;
        }else{ //ith motion component
            comp_rect = ((CvConnectedComp*)cvGetSeqElem(seq,i))->rect;
            if(comp_rect.width+comp_rect.height<100)continue;//reject small stuff
            color = CV_RGB(255,0,0);
            magnitude = 30;
        }
		
        cvSetImageROI(silh,comp_rect);
        cvSetImageROI(mhi,comp_rect);
        cvSetImageROI(orient,comp_rect);
        cvSetImageROI(mask,comp_rect);
		
        angle = cvCalcGlobalOrientation(orient,mask,mhi,timestamp,MHI_DURATION);
        //angle = 360.0-angle;
        //angle = -angle+90;
        if(i<0)mainMotion.angle=angle;
		
        count = cvNorm(silh,0,CV_L1,0);
		
        cvResetImageROI(silh);
        cvResetImageROI(mhi);
        cvResetImageROI(orient);
        cvResetImageROI(mask);
		
        if(count<comp_rect.width*comp_rect.height*0.05){
            mainMotion.dim=0;
            continue;
		}
		
        if(i<0){
            mainMotion.dim = cvRound(magnitude*1.2);
        }else{
            moTemp m;
            m.x = comp_rect.x+comp_rect.width/2;
            m.y = comp_rect.y+comp_rect.height/2;
            m.dim = cvRound(magnitude*1.2);
            m.angle=angle;
            motionFrag.push_back(m);
        }
		
        /*center = cvPoint((comp_rect.x+comp_rect.width/2),(comp_rect.y+comp_rect.height/2));
		 cvCircle(motion.getCvImage(),center,cvRound(magnitude*1.2),color,3,CV_AA,0);
		 cvLine(motion.getCvImage(),center,cvPoint(cvRound(center.x+magnitude*cos(angle*CV_PI/180)),cvRound(center.y-magnitude*sin(angle*CV_PI/180))),color,3,CV_AA,0);
		 motion.flagImageChanged();*/
    }
}

void ofxMotionTemplates::draw(){
	ofPushStyle();
	ofSetColor(ofColor::white);
	motion.draw(0, 0);
	ofEnableAlphaBlending();
	ofSetColor(255, 0, 255, 200);
	ofFill();
	
	ofPushMatrix();
	ofTranslate(mainMotion.x, mainMotion.y, 0);
	ofRotate(mainMotion.angle, 0, 0, 1);
	ofTriangle(0, -10, 0, 10, mainMotion.dim, 0);
	ofPopMatrix();
	
	ofSetColor(150, 255, 0, 200);
	for(int i=0;i<motionFrag.size();i++){
		ofPushMatrix();
		ofTranslate(motionFrag[i].x, motionFrag[i].y, 0);
		ofRotate(motionFrag[i].angle, 0, 0, 1);
		ofTriangle(0, -10, 0, 10, motionFrag[i].dim, 0);
		ofPopMatrix();
	}
	
	ofDisableAlphaBlending();
	ofPopStyle();
}

bool ofxMotionTemplates::isMainGestureDone(){
	if(oldMain>0 && mainMotion.dim==0){
		return true;
	}else{
		return false;
	}
}