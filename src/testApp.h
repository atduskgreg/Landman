#ifndef _TEST_APP
#define _TEST_APP


#include "ofMain.h"
#include "btBulletDynamicsCommon.h"
#include "btHeightfieldTerrainShape.h"
//#include "ofx3DUtils.h"
#include "ballMotionState.cpp"

#include "ofxOpenCv.h"


class testApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
    void exit();

		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);

  btDiscreteDynamicsWorld* dynamicsWorld;
  btRigidBody* fallRigidBody;
  ballMotionState* ballMS;
  
  ofxCvGrayscaleImage depthMap;
  
  ofPoint ballPosition;
  
};

#endif
