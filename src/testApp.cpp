#include "testApp.h"

int numBalls = 1;
int sphereSize = 20;

void testApp::dropBalls(){

  ballPositions.clear();
  fallRigidBodies.clear();
  
  for(int i = 0; i < 8; i++){
    for(int j = 0; j < 8; j++){
      ballPositions.push_back(ofPoint(i*10, 150, j*15));
    }
  }
  
  for(int i = 0; i < numBalls; i++){

    ofPoint& cur = ballPositions[i];
    ballMotionState* ballMS =
    new ballMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(cur.x, cur.y, cur.z)), &cur);
    
    btScalar mass = 2;
    btVector3 fallInertia(0,0,0);
    fallShape->calculateLocalInertia(mass,fallInertia);
    btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,ballMS,fallShape,fallInertia);
    
    btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
    fallRigidBody->setCollisionFlags( fallRigidBody->getCollisionFlags() & (~btCollisionObject::CF_STATIC_OBJECT));
    fallRigidBody->setRestitution(0.8f);

    dynamicsWorld->addRigidBody(fallRigidBody);    
    
    fallRigidBodies.push_back(fallRigidBody);
    ballMSes.push_back(ballMS);
  }
  
  
}


void testApp::setupBulletWorld(){
  broadphase = new btDbvtBroadphase();

  /*btOverlappingPairCallback* ghostPairCallback = new btGhostPairCallback();
  broadphase->getOverlappingPairCache()->setInternalGhostPairCallback(ghostPairCallback);
  */
  collisionConfiguration = new btDefaultCollisionConfiguration();
  dispatcher = new btCollisionDispatcher(collisionConfiguration);
  solver = new btSequentialImpulseConstraintSolver;
  dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
  
  dynamicsWorld->setGravity(btVector3(0,-100,0));
  
  
  
  groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,-50,0)));
  groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);
    
  
  btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
  groundRigidBody = new btRigidBody(groundRigidBodyCI);
  
  //dynamicsWorld->addRigidBody(groundRigidBody);

  fallShape = new btSphereShape(sphereSize);

  dropBalls();
    
   
  quadratic = gluNewQuadric();

}

// HERE:
void testApp::localCreateRigidBody(btHeightfieldTerrainShape *terrain){
  //btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);
  //  
  btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0)));
  //  
  btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,terrain,btVector3(0,0,0));
  //  
  btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
  
  groundRigidBody->setCollisionFlags(groundRigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);

  groundRigidBody->setRestitution(1.0f);
  dynamicsWorld->addRigidBody(groundRigidBody);
  
}

//--------------------------------------------------------------
void testApp::setup() {
	ofSetFrameRate(60);
  heightFieldCreated = false;
  
  fullAppBuffer.allocate(ofGetWidth(), ofGetHeight(), OF_IMAGE_COLOR);

  model.load("self_portrait_final.obj");
  //model.load("edited_rough_model.obj");
  
	input.setup(panel);  // < -- could pass in useKinnect here?
	
	panel.setup("Control Panel", 1024-310, 5, 300, 600);
	
	panel.addPanel("Threshold and Scale");
	panel.addSlider("near threshold", "nearThreshold", 255, 0, 255, true);
	panel.addSlider("far threshold", "farThreshold", 0, 0, 255, true);
	
	panel.addPanel("Control");
	panel.addSlider("scale", "scale", 2, 1, 20);	
	panel.addSlider("rotate y axis", "rotateY", 0, -360, 360, false);	
  panel.addSlider("rotate x axis", "rotateX", -90, -360, 360, false);	
  panel.addSlider("rotate z axis", "rotateZ", 0, -360, 360, false);	

	panel.addSlider("rotate y axis", "bulletRotateY", 0, -360, 360, false);	
  panel.addSlider("rotate x axis", "bulletRotateX", -90, -360, 360, false);	
  panel.addSlider("rotate z axis", "bulletRotateZ", 0, -360, 360, false);	
	
  panel.addToggle("auto rotate", "autoRotate", false);
	panel.addToggle("draw debug", "drawDebug", false);
	
	panel.addToggle("draw scene bounding frustrum", "drawSceneBox", false);
		
	if (input.usingKinect() == false){
		panel.addSlider("playback speed", "playSpeed", 0.5, -1, 1, false);
		panel.addToggle("playbackPause", "playbackPause", false);
	}
	
	panel.addPanel("Mesh");
	panel.addSlider("vertexStep", "vertexStep", 2, 1, 10, true);
  
  
	setupBulletWorld();
  
  //btScalar heightScale, btScalar minHeight, btScalar maxHeight,int upAxis,
  
  heightfieldShape = new btHeightfieldTerrainShape(640, 480, input.depthImage.getPixels(), 0.4, 0, 255, 1, PHY_UCHAR, false);

  localCreateRigidBody(heightfieldShape);
}




//--------------------------------------------------------------
void testApp::update() {
  dynamicsWorld->stepSimulation(1/60.f,10);
	input.update();
  if (bRecording == true){
		saver.addFrame(fullAppBuffer.getPixels(), 1.0f / ofGetFrameRate()); 
  }
  
  //if(!heightFieldCreated){
    //heightfieldShape = new btHeightfieldTerrainShape(640, 480, input.depthImage.getPixels(), 0.2, 0.0, 17.0, 1, PHY_FLOAT, false);
    //localCreateRigidBody(heightfieldShape);
    //heightFieldCreated = true;
  //}
}

//--------------------------------------------------------------
void testApp::draw() {
	
	ofBackground(0, 0, 0);
	
	// draw debug or non debug
	
	if (panel.getValueB("drawDebug")){
		input.drawDebug();
		
	} else {
    
    
		ofPushMatrix();
		
			// center everything
			ofTranslate(ofGetWidth()/2, ofGetWidth()/2, -1000);
			ofSetColor(255, 255, 255);
    
      ofRotateX(panel.getValueF("rotateX"));      
			ofRotateY(panel.getValueF("rotateY"));
      ofRotateZ(panel.getValueF("rotateZ"));
			
      if (panel.getValueB("autoRotate")){
				ofRotateZ(ofGetElapsedTimef()*5);
			}
			
			float scale = panel.getValueF("scale");
			ofScale(scale, scale, scale);


			int w = input.camWidth;
			int h = input.camHeight;
			
			int vertexStep = panel.getValueI("vertexStep");
			
			int xMax = w - vertexStep;
			int yMax = h - vertexStep;
			
			glEnable(GL_LIGHTING);
			glEnable(GL_DEPTH_TEST);
			glEnable(GL_LIGHT0);
			glEnable(GL_NORMALIZE);
			
			//glBegin(GL_QUADS);
//			ofxVec3f zero(0, 0, 0);
//			for(int y = 0; y < yMax; y += vertexStep) {
//				for(int x = 0; x < xMax; x += vertexStep) {
//				
//					ofxVec3f& nw = input.pointGrid[y][x];
//					ofxVec3f& ne = input.pointGrid[y][x + vertexStep];
//					ofxVec3f& se = input.pointGrid[y + vertexStep][x + vertexStep];
//					ofxVec3f& sw = input.pointGrid[y + vertexStep][x];
//					
//					if(nw != zero && ne != zero && sw != zero && se != zero) {
//					
//						ofxVec3f right = ne - nw;
//						ofxVec3f down = sw - nw;
//						ofxVec3f out = down.cross(right);
//												
//						glNormal3f(out.x, out.y, out.z);						
//						glVertex3f(nw.x, nw.y, nw.z);
//						glVertex3f(ne.x, ne.y, ne.z);
//						glVertex3f(se.x, se.y, se.z);
//						glVertex3f(sw.x, sw.y, sw.z);
//						
//					}
//				}
//			}
//			glEnd();
//			

    
    ofPushMatrix();
      ofTranslate(0, 100, 0);

      ofRotateX(panel.getValueF("bulletRotateX"));
      
      ofRotateY(panel.getValueF("bulletRotateY"));
      
      ofRotateZ(panel.getValueF("bulletRotateZ"));
      
      ofSetColor(255, 0, 0);
      ofNoFill();
      
      //cout << "y: " <<ballPosition.y << endl;
      
    for(int i = 0; i < numBalls; i++){
      ofPushMatrix();
        glPushMatrix();
          glEnable(GL_LIGHTING);
          glEnable(GL_LIGHT0);    
      
          ofPoint ballPosition = ballPositions[i];
          ofTranslate(ballPosition.x * scale - 50, ballPosition.y * scale, ballPosition.z * scale);
          ofScale(0.2, 0.2, 0.2);
          ofSetColor(255, 255, 255);
          ofFill();
          model.draw();
        
          //gluSphere(quadratic, sphereSize, 20, 20);  
          glDisable(GL_LIGHTING);
          glDisable(GL_LIGHT0);    
        glPopMatrix();
      ofPopMatrix();
    }
    
    
    ofSetColor(255,255,255,50);
    
    
    glDisable(GL_LIGHTING);
    //glEnable(GL_DEPTH_TEST);
    //glEnable(GL_LIGHT0);
    //glEnable(GL_NORMALIZE);
    
    
    for (int i = 0; i < 640-10; i+= 10){
      for (int j = 0; j < 480-10; j+= 10){
     
        btVector3 vertexA;
        btVector3 vertexB;
        btVector3 vertexC;
        btVector3 vertexD;
        
        heightfieldShape->getVertex(i,j, vertexA);
        heightfieldShape->getVertex(i+10,j, vertexB);
        heightfieldShape->getVertex(i+10,j+10, vertexC);
        heightfieldShape->getVertex(i,j+10, vertexD);
        
        bool p = true;
        
        //ofPushMatrix();
        glBegin(GL_POLYGON);
        
          btVector3 normal = vertexA.cross(vertexC);
          glNormal3f(normal.x(), normal.y(), normal.z());
        
          float ys[4] = {vertexA.y(), vertexB.y(), vertexC.y(),vertexD.y()};
          sort(ys, ys+4);
          
          /*bool frun = true;
          if(frun){
            cout << "0: " << ys[3] << " a: " << ys[0] << endl;
            frun = false;
          }*/
        
          if(ys[3] - ys[0] > 50 ){ 

            glColor3f(0.55, 0.45, 0.25);
              glVertex3f(vertexA.x() * scale - 50, vertexA.y() * scale, vertexA.z() * scale);
              glVertex3f(vertexB.x() * scale - 50, vertexB.y() * scale, vertexB.z() * scale);
            
            glColor3f(0, 0.45, 0);
              glVertex3f(vertexC.x() * scale - 50, vertexC.y() * scale, vertexC.z() * scale);
              glVertex3f(vertexD.x() * scale - 50, vertexD.y() * scale, vertexD.z() * scale);
          } else{
          
                  
            ofColor c = colorFromVertex(vertexA.y());
            glColor3f(c.r, c.g, c.b);
        
            glVertex3f(vertexA.x() * scale - 50, vertexA.y() * scale, vertexA.z() * scale);
        
            c = colorFromVertex(vertexB.y());
            glColor3f(c.r, c.g, c.b);
        
            glVertex3f(vertexB.x() * scale - 50, vertexB.y() * scale, vertexB.z() * scale);
        
            c = colorFromVertex(vertexC.y());
            glColor3f(c.r, c.g, c.b);
          
            glVertex3f(vertexC.x() * scale - 50, vertexC.y() * scale, vertexC.z() * scale);
          
            c = colorFromVertex(vertexD.y());
            glColor3f(c.r, c.g, c.b);
        
            glVertex3f(vertexD.x() * scale - 50, vertexD.y() * scale, vertexD.z() * scale);
          }

        glEnd();
        

      }
    }
      
    ofPopMatrix();
    
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
		ofPopMatrix();
    

		
	}
	
  if(bRecording == true){
    fullAppBuffer.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
  }
}

ofColor testApp::colorFromVertex(float y){

  ofColor result;
  
  if(y < -115){
    result.r = ofMap(y, -125, -115, 0, 0.55, true);
    result.g = ofMap(y, -125, -115, 0, 0.45, true);
    result.b = ofMap(y, -125, -115, 1, 0.25, true);
  }
  
  if(y >= -115 && y < -80){
    result.r = ofMap(y, -115, -80, 0.55, 0, true);
    result.g = ofMap(y, -115, -80, 0.45, 0.8, true);
    result.b = ofMap(y, -115, -80, 0.25, 0.1, true);
  }
  
  /*result.r = r;//0.55;//150;
  result.g = g;//0.45;//100;
  result.b = b;//.25;//75;
   */
  return result;
}

//--------------------------------------------------------------
void testApp::exit() {
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	if(key == 'f') {
		ofToggleFullscreen();
	}
  
  if(key == 'b'){
    dropBalls();
  }
  
  if(key == 'n'){
    heightFieldCreated = false;
  }
  
  if (key == 'a'){
		saver.setup(ofGetWidth(),ofGetHeight(),"ball_drop.mov");
	} else if (key == 's'){
		saver.finishMovie();
		bRecording = false;
	} else if (key == 'r'){
		bRecording = !bRecording;
	}
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y) {
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button) {
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button) {
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button) {
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h) {
}

