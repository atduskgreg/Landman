#include "testApp.h"

void testApp::setupBulletWorld(){
  btBroadphaseInterface* broadphase = new btDbvtBroadphase();
  
  btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
  
  btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
  
  dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
  
  dynamicsWorld->setGravity(btVector3(0,-10,0));
  
  
  
  btCollisionShape* fallShape = new btSphereShape(1);
  
    
  
  ballMS =
  new ballMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,50,0)), &ballPosition);
  btScalar mass = 1;
  btVector3 fallInertia(0,0,0);
  fallShape->calculateLocalInertia(mass,fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,ballMS,fallShape,fallInertia);
  fallRigidBody = new btRigidBody(fallRigidBodyCI);
  fallRigidBody->setCollisionFlags( fallRigidBody->getCollisionFlags() & (~btCollisionObject::CF_STATIC_OBJECT) &  ( btCollisionObject::CF_KINEMATIC_OBJECT));
  dynamicsWorld->addRigidBody(fallRigidBody);
  
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
  
  dynamicsWorld->addRigidBody(groundRigidBody);
  
}

//--------------------------------------------------------------
void testApp::setup() {
	ofSetFrameRate(60);
  heightFieldCreated = false;

	input.setup(panel);  // < -- could pass in useKinnect here?
	
	panel.setup("Control Panel", 1024-310, 5, 300, 600);
	
	panel.addPanel("Threshold and Scale");
	panel.addSlider("near threshold", "nearThreshold", 255, 0, 255, true);
	panel.addSlider("far threshold", "farThreshold", 0, 0, 255, true);
	
	panel.addPanel("Control");
	panel.addSlider("scale", "scale", 7, 1, 20);	
	panel.addSlider("rotate y axis", "rotateY", 0, -360, 360, false);	
  panel.addSlider("rotate x axis", "rotateX", -90, -360, 360, false);	
  panel.addSlider("rotate z axis", "rotateZ", 0, -360, 360, false);	

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
}

//--------------------------------------------------------------
void testApp::update() {
  dynamicsWorld->stepSimulation(1/60.f,10);
	input.update();
  if(!heightFieldCreated){
    heightfieldShape = new btHeightfieldTerrainShape(640, 480, input.depthImage.getPixels(), 0.2, 0.0, 17.0, 1, PHY_FLOAT, false);
    localCreateRigidBody(heightfieldShape);
    heightFieldCreated = true;
  }
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
			
			glBegin(GL_QUADS);
			ofxVec3f zero(0, 0, 0);
			for(int y = 0; y < yMax; y += vertexStep) {
				for(int x = 0; x < xMax; x += vertexStep) {
				
					ofxVec3f& nw = input.pointGrid[y][x];
					ofxVec3f& ne = input.pointGrid[y][x + vertexStep];
					ofxVec3f& se = input.pointGrid[y + vertexStep][x + vertexStep];
					ofxVec3f& sw = input.pointGrid[y + vertexStep][x];
					
					if(nw != zero && ne != zero && sw != zero && se != zero) {
					
						ofxVec3f right = ne - nw;
						ofxVec3f down = sw - nw;
						ofxVec3f out = down.cross(right);
												
						glNormal3f(out.x, out.y, out.z);						
						glVertex3f(nw.x, nw.y, nw.z);
						glVertex3f(ne.x, ne.y, ne.z);
						glVertex3f(se.x, se.y, se.z);
						glVertex3f(sw.x, sw.y, sw.z);
						
					}
				}
			}
			glEnd();
			

    
    ofPushMatrix();
      ofSetColor(255, 0, 0);
      ofNoFill();
      cout << "y: " <<ballPosition.y << endl;
      ofTranslate(ballPosition.x * scale - 50, ballPosition.y * scale, ballPosition.z * scale);
      gluSphere(quadratic, 5.0, 20, 20);  

    ofPopMatrix();
    
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
		ofPopMatrix();
    

		
	}
	
}

//--------------------------------------------------------------
void testApp::exit() {
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	if(key == 'f') {
		ofToggleFullscreen();
	}
  
  if(key == 'n'){
    heightFieldCreated = false;
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

