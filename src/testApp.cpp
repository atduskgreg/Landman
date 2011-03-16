#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
  ofImage loading;
  loading.loadImage("kinect_data.png");
  depthMap.allocate(640, 480);
  
  depthMap.setFromPixels(loading.getPixels(), 640, 480);
  
  
  btBroadphaseInterface* broadphase = new btDbvtBroadphase();
  
  btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
  
  btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
  
  dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
  
  dynamicsWorld->setGravity(btVector3(0,-10,0));
  
  
  //btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);
  
  btCollisionShape* fallShape = new btSphereShape(1);
  
  
  btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0)));

  btHeightfieldTerrainShape * heightfieldShape = new btHeightfieldTerrainShape(640, 480, depthMap.getPixels(), 0.2, 0.0, 17.0, 1, PHY_FLOAT, false);
  btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,heightfieldShape,btVector3(0,0,0));
  
  btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);

  
  
  
  
  dynamicsWorld->addRigidBody(groundRigidBody);
  
  
  ballMS =
  new ballMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,50,0)), &ballPosition);
  btScalar mass = 1;
  btVector3 fallInertia(0,0,0);
  fallShape->calculateLocalInertia(mass,fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,ballMS,fallShape,fallInertia);
  fallRigidBody = new btRigidBody(fallRigidBodyCI);
  fallRigidBody->setCollisionFlags( fallRigidBody->getCollisionFlags() & (~btCollisionObject::CF_STATIC_OBJECT) &  ( btCollisionObject::CF_KINEMATIC_OBJECT));
  dynamicsWorld->addRigidBody(fallRigidBody);
  
  
  
  

}

void::testApp::exit(){
//  delete dynamicsWorld;
//  delete solver;
//  delete dispatcher;
//  delete collisionConfiguration;
//  delete broadphase;  
}

//--------------------------------------------------------------
void testApp::update(){
  dynamicsWorld->stepSimulation(1/60.f,10);
  btTransform trans;
  fallRigidBody->getMotionState()->getWorldTransform(trans);//->getWorldTransform(trans);
  //fallRigidBody->getMotionState()->setWorldTransform(trans);  
  
  //std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;
//  ballYPosition = trans.getOrigin().getY();
}

//--------------------------------------------------------------
void testApp::draw(){
  ofSetColor(0, 0, 0);
  ofRect(0,500, ofGetWidth(), 50);
  
  ofSetColor(255, 0, 0 );
  ofCircle(ofGetWidth()/2, ofMap(ballPosition.y, 0, 50, 500, 0, true), 10);
 }

//--------------------------------------------------------------
void testApp::keyPressed(int key){

}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
  
  float mappedY = ofMap((float)y, 500, 0, 0, 50, true);
  float mappedX = ofMap((float)y, 500, 0, 0, 50, true);
  cout << "x: " << mappedX << endl;
  ofPoint newPos = ofPoint(mappedX, mappedY, 0);
  ballMS->setKinematicPos(newPos);

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

