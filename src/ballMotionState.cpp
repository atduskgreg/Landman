/*
 *  ballMotionState.cpp
 *  emptyExample
 *
 *  Created by Greg Borenstein on 3/16/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */
#include "btBulletDynamicsCommon.h"
#include "ballMotionState.h"
#include "ofMain.h"


class ballMotionState : public btMotionState {
public:
  ballMotionState(const btTransform &initialpos, ofPoint *p) {
    mVisibleobj = p;
    mPos1 = initialpos;
  }
  
  virtual ~ballMotionState() {
  }
  
  void setNode(ofPoint *p) {
    mVisibleobj = p;
  }
  
  void setKinematicPos(ofPoint &currentPos) {
    //cout << "skp: " << currentPos.y << endl;
    btTransform currentTransform = btTransform(btQuaternion(0,0,0,1),btVector3(currentPos.x,currentPos.y,currentPos.z));
    //cout << "ct: " << currentTransform.getOrigin().x() << endl;
    mVisibleobj->x = currentPos.x;
    mVisibleobj->y = currentPos.y;
    mVisibleobj->z = currentPos.z;
    //setWorldTransform(currentTransform);
    mPos1 = currentTransform;
  }
  
  virtual void getWorldTransform(btTransform &worldTrans) const {
    //cout << "gwtX: " << worldTrans.getOrigin().x() << endl;
    worldTrans = mPos1;
  }
  
  virtual void setWorldTransform(const btTransform &worldTrans) {
    if(NULL == mVisibleobj) return; // silently return before we set a node
    btQuaternion rot = worldTrans.getRotation();
    //mVisibleobj->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
    btVector3 pos = worldTrans.getOrigin();
    //mVisibleobj->setPosition(pos.x(), pos.y(), pos.z());
    mVisibleobj->x = pos.x();
    mVisibleobj->y = pos.y();
    mVisibleobj->z = pos.z();
    
  }
  
protected:
  ofPoint *mVisibleobj;
  btTransform mPos1;
};