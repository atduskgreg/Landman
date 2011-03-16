#pragma once

#include "ofMain.h"

#include "ofxAutoControlPanel.h"
#include "DepthVisualizerInput.h"
#include "btBulletDynamicsCommon.h"
#include "btHeightfieldTerrainShape.h"
#include "ballMotionState.cpp"

class testApp : public ofBaseApp {
	
	public:

		void setup();
		void update();
		void draw();
		void exit();
  
    void setupBulletWorld();
    void localCreateRigidBody(btHeightfieldTerrainShape *t);

		void keyPressed  (int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);

		ofxAutoControlPanel panel;
  
    btDiscreteDynamicsWorld* dynamicsWorld;
    btRigidBody* fallRigidBody;
    ballMotionState* ballMS;
	
    ofPoint ballPosition;
    GLUquadricObj *quadratic;

    bool heightFieldCreated;
    btHeightfieldTerrainShape * heightfieldShape;
		DepthVisualizerInput input;
};
