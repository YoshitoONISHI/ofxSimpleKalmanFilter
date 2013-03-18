//
//  ofxSimpleKalmanFilter.h
//  ofxSimpleKalmanFilter
//
//  Created by Onishi Yoshito on 2/21/13.
//
//

#ifndef ofxSimpleKalmanFilter_ofxSimpleKalmanFilter_h
#define ofxSimpleKalmanFilter_ofxSimpleKalmanFilter_h

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

template<class Type>
class ofxBasicSimpleKalmanFilter
{
public:
    ofxBasicSimpleKalmanFilter()
    :
    mQ(1e-4), mR(1e-2)
    {}
    
    void setup(const Type &P, const Type &X, const Type &K, double Q, double R)
    {
        mQ = Q;
        mR = R;
        mP = P;
        mX = X;
        mK = K;
    }
    
    Type update(Type measurement)
    {
        measurementUpdate();
        mResult = mX + (measurement - mX) * mK;
        mX = mResult;
        return mResult;
    }
    
    Type getResult() const
    {
        return mResult;
    }
    
private:
    void measurementUpdate()
    {
        mK = (mP + mQ) / (mP + mQ + mR);
        mP = mR * (mP + mQ) / (mR + mP + mQ);
    }
    
    Type mResult;
    Type mP, mX, mK;
    double mQ, mR;
};


typedef ofxBasicSimpleKalmanFilter<double> ofxSimpleKalmanFilter;


#include "ofMain.h"

typedef ofxBasicSimpleKalmanFilter<ofVec2f> ofxSimpleKalmanFilterVec2f;
typedef ofxBasicSimpleKalmanFilter<ofVec3f> ofxSimpleKalmanFilterVec3f;
typedef ofxBasicSimpleKalmanFilter<ofVec4f> ofxSimpleKalmanFilterVec4f;

typedef ofxBasicSimpleKalmanFilter<ofQuaternion> ofxSimpleKalmanFilterQuat;

#endif