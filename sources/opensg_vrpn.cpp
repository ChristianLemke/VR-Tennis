#include <cstdlib>
#include <cstddef>
#include <cmath>
#include <iostream>
#include <ios>
 
#include <OpenSG/OSGGLUT.h>
#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGGLUTWindow.h>
#include <OpenSG/OSGMultiDisplayWindow.h>
#include <OpenSG/OSGSceneFileHandler.h>

#include <OpenSG/OSGTextureBackground.h>
#include <OpenSG/OSGGradientBackground.h>
#include <OpenSG/OSGPointLight.h>
#include <OpenSG/OSGSpotLight.h>
#include <ctime>

#include <OSGCSM/OSGCAVESceneManager.h>
#include <OSGCSM/OSGCAVEConfig.h>
#include <OSGCSM/appctrl.h>

#include <vrpn_Tracker.h>
#include <vrpn_Button.h>
#include <vrpn_Analog.h>

// my includes
#include <OpenSG/OSGComponentTransform.h>
#include <OpenSG/OSGMaterialGroup.h>
#include <OpenSG/OSGImage.h>
#include <OpenSG/OSGSimpleTexturedMaterial.h>
#include <chrono>
#include <vector>
#include <OpenSG/OSGGeoFunctions.h>
//#include <sys/time.h>
#include <OpenSG/OSGSkyBackground.h>

#include <OSGIntersectAction.h>
#include <OSGLine.h>
#include <stdlib.h>     /* srand, rand */

OSG_USING_NAMESPACE

OSGCSM::CAVEConfig cfg;
OSGCSM::CAVESceneManager *mgr = nullptr;
vrpn_Tracker_Remote* tracker =  nullptr;
vrpn_Button_Remote* button = nullptr;
vrpn_Analog_Remote* analog = nullptr;


// wand
// flach für drop test
//Quaternion initWand_orientation = Quaternion(Vec3f(1.f,0.f,0.f),osgDegree2Rad(-90.f)) * Quaternion(Vec3f(0.f,0.f,1.f),osgDegree2Rad(-45.f)) ;
// für serve
//Quaternion initWand_orientation = Quaternion(Vec3f(1.f,0.f,0.f),osgDegree2Rad(45.f)) ;
//Quaternion initWand_orientation = Quaternion(Vec3f(1.f,0.f,0.f),osgDegree2Rad(0.f)) * Quaternion(Vec3f(0.f,1.f,0.f),osgDegree2Rad(90.f)) ;
Quaternion initWand_orientation = Quaternion(Vec3f(0.f,0.f,1.f),osgDegree2Rad(60.f-5.f)) * Quaternion(Vec3f(0.f,1.f,0.f),osgDegree2Rad(90.f)) ;
Vec3f initWand_position = Vec3f(0, 0, 10);

Quaternion initWand2_orientation = Quaternion(Vec3f(0.f,0.f,0.f),osgDegree2Rad(0.f)) ;
//Vec3f initWand2_position = Vec3f(0, 254, 0);
Vec3f initWand2_position = Vec3f(0, 154, 0);

// wand racket (3)
Quaternion wand_orientation = initWand_orientation;
Vec3f wand_position = initWand_position;

// wand ball (0)
Vec3f wand2_position = initWand2_position;
Quaternion wand2_orientation = initWand2_orientation;

// ball class
class Ball{
	public:	
		Ball(Vec3f ballPositionPointMeter, Vec3f ballVelocityVectorMeterPerSeconds, NodeRecPtr ballNode, ComponentTransformRecPtr ballCT) :
		ballPositionPointMeter(ballPositionPointMeter), ballVelocityVectorMeterPerSeconds(ballVelocityVectorMeterPerSeconds), node(ballNode), CT(ballCT), ballRayState(0.f), initRotation(Quaternion(Vec3f(0,0,0),osgDegree2Rad(0.f))){}
  	
 		void info(){ 
			std::cout << "ballPositionPointMeter: " << ballPositionPointMeter << " ballVelocityVectorMeterPerSeconds: " << ballVelocityVectorMeterPerSeconds << " ballNode: " << node << " ballCT: " << CT << " ballRayState: " << ballRayState << std::endl; 
		}
 
 		Vec3f ballPositionPointMeter;
 		Vec3f ballVelocityVectorMeterPerSeconds;
		NodeRecPtr node;
 		ComponentTransformRecPtr CT;
		Real32 ballRayState;
		Quaternion initRotation;
};

//------------------------------------------------------------------------------
// My Global Variables
//
// Positionen in cm
// Zeit in s
//------------------------------------------------------------------------------
bool testMode = true;
bool autoServe = false;
int serveEverySec = 4;

Real32 frameTimeDeltaSeconds= 0;

NodeRecPtr rootNode;

ComponentTransformRecPtr racketCTWand;

NodeRecPtr racketPlaneNode;
ComponentTransformRecPtr racketPlaneCT;

ComponentTransformRecPtr racketPlaneFaceCT;
ComponentTransformRecPtr ballRacketRayCT;

Quaternion wandFixRotationQuaternion = Quaternion(Vec3f(1,0,0),osgDegree2Rad(-30.f));

Vec3f racketVectorCM = Vec3f(0,0,0);
Vec3f racketVectorDifferenceCM = Vec3f(0,0,0);
Vec3f wand2VectorCM = Vec3f(0,0,0);

// balls
std::vector<Ball*> vectorOfBalls;
// TODO
// >=2 !!
int maxBalls = 16; 
int currentBallindex = 0;

// ball
Real32 ballRadiusMeter = 0.062f;
Real32 ballRadiusCM = ballRadiusMeter * 100;
Vec3f ballPositionUnvisible = Vec3f(0.f, ballRadiusCM, 10.f);

bool ballHolding = false;

// net
NodeRecPtr netPlaneNode;

// Walls
NodeRecPtr wallPlanesNode;

void cleanup()
{
	//TODO

	delete mgr;
	delete tracker;
	delete button;
	delete analog;
}

void print_tracker();
void addNewBall(Vec3f position, Vec3f direction);
void commandSortBalls();

void addCenterBox(NodeRecPtr rootNode){
	NodeRecPtr centerBox = makeBox(1000,1000,1000,1000,1000,1000);
	SimpleMaterialRecPtr centerBoxMat = SimpleMaterial::create();
	centerBoxMat->setDiffuse(Color3f(1,0.8f,0));
	centerBoxMat->setAmbient(Color3f(0.8f, 0.2f, 0.2f));
	MaterialGroupRecPtr centerBoxMgCore = MaterialGroup::create();
	centerBoxMgCore->setMaterial(centerBoxMat);

	NodeRecPtr centerBoxChild = Node::create();
	centerBoxChild->setCore(centerBoxMgCore);
	centerBoxChild->addChild(centerBox);
	rootNode->addChild(centerBoxChild);
}



void addSkyBox(NodeRecPtr rootNode){
	NodeRecPtr skybox = SceneFileHandler::the()->read("models/skydome.osb");
	if(skybox == NULL){
		std::cout << "models/skydome.osb not avalable. Generateing it out of the model." << std::endl;
		// skybox geht final
		skybox = SceneFileHandler::the()->read("models/skydome/skydome_day.WRL");

		SceneFileHandler::the()->write(skybox, "models/skydome.osb");
	} else{
		std::cout << "models/skydome.osb loaded." << std::endl;
	}
	
	//setName(skybox, "skybox");
	ComponentTransformRecPtr skyboxCT = ComponentTransform::create();
	skyboxCT->setTranslation(Vec3f(0,0,0));
	skyboxCT->setScale(Vec3f(1,1,1)*0.01);
	NodeRecPtr skyboxTrans = Node::create();
	//setName(skyboxTrans, "skyboxTrans");
	skyboxTrans->setCore(skyboxCT);
	skyboxTrans->addChild(skybox);
	rootNode->addChild(skyboxTrans);
	
	//ground
	ImageRecPtr imageSkybox = Image::create();
	imageSkybox->read("images/ground.jpg");

	// Create floor texture from floor image
	SimpleTexturedMaterialRecPtr texSkybox = SimpleTexturedMaterial::create();
	texSkybox->setImage(imageSkybox);

	//Make Ground
	GeometryRecPtr groundGeo = makePlaneGeo(7500.f, 7500.f, 1, 1);
	groundGeo->setMaterial(texSkybox);
	NodeRecPtr ground = Node::create();
	ground->setCore(groundGeo);
	//setName(ground, "ground");
	//Put it to our Feet
	ComponentTransformRecPtr groundCT = ComponentTransform::create();
	groundCT->setRotation(Quaternion(Vec3f(1,0,0),osgDegree2Rad(90)));
	groundCT->setTranslation(Vec3f(0,-20.f,0.f));
	//groundCT->setScale(Vec3f(1,1,1)*10);
	NodeRecPtr groundTrans = Node::create();
	//setName(groundTrans, "groundTrans");
	groundTrans->setCore(groundCT);
	groundTrans->addChild(ground);
	rootNode->addChild(groundTrans);
}

void addRacket(NodeRecPtr rootNode){
	NodeRecPtr racketModel = SceneFileHandler::the()->read("models/racket.osb");
	if(racketModel == NULL){
		std::cout << "models/racket.osb not avalable. Generateing it out of the model." << std::endl;
		// racket geht final 
		racketModel = SceneFileHandler::the()->read("models/rackets/tennisracket/genblau/tennisracquet.obj");
		// NodeRecPtr racket = SceneFileHandler::the()->read("models/rackets/tennisracket/genblau/tennisracquet.wrl");
		// test
		//NodeRecPtr racketChild = SceneFileHandler::the()->read("models/rackets/tennisracket/genblau/tennisracquet.stl");

		SceneFileHandler::the()->write(racketModel, "models/racket.osb");
	} else{
		std::cout << "models/racket.osb loaded." << std::endl;
	}
	
	// model
	ComponentTransformRecPtr racketModelCT = ComponentTransform::create();
	racketModelCT->setScale(Vec3f(3.f,3.f,3.f));
	//racketModelCT->setTranslation(Vec3f(0,0.f,10.f));
	racketModelCT->setTranslation(Vec3f(0.f, 0.f, 24.f));
	
	NodeRecPtr racketModelCTNode = makeNodeFor(racketModelCT);
	racketModelCTNode->setCore(racketModelCT);
	
	// ct2
	ComponentTransformRecPtr racketCT2 = ComponentTransform::create();
	// dienstag racketCTFix->setTranslation(Vec3f(0.f,-20.f,0.f));
	//racketCTFix->setTranslation(Vec3f(0.f,-22.f,-14.5f));
	racketCT2->setRotation(Quaternion(Vec3f(1,0,0),osgDegree2Rad(60.f)));
	
	NodeRecPtr racketCT2Node = makeNodeFor(racketCT2);
	racketCT2Node->setCore(racketCT2);
	
	// global
	racketCTWand = ComponentTransform::create();
	//racketCTWand->setTranslation(Vec3f(0,0,100.f));

	NodeRecPtr racketWandTransNode = makeNodeFor(racketCTWand);
	racketWandTransNode->setCore(racketCTWand);
	
	
	
	// add
 	racketModelCTNode->addChild(racketModel);
 	racketCT2Node->addChild(racketModelCTNode);
	racketWandTransNode->addChild(racketCT2Node);
 	
	rootNode->addChild(racketWandTransNode);
	
}

void addRacketPlane(NodeRecPtr rootNode){
	// RacketPlane
	NodeRecPtr racketPlaneModel = makePlane(35,85,1,1);
	
	//material
	SimpleMaterialRecPtr testMat = SimpleMaterial::create();
	testMat->setDiffuse(Color3f(0,1.f,0.f));
	//testMat->setAmbient(Color3f(0.8f, 0.2f, 0.2f));
	// TODO
	testMat->setTransparency(1.0f);
	
	
	MaterialGroupRecPtr racketPlaneMgCore = MaterialGroup::create();
	racketPlaneMgCore->setMaterial(testMat);
	
	NodeRecPtr racketPlaneMgNode = makeNodeFor(racketPlaneMgCore);
	racketPlaneMgNode->setCore(racketPlaneMgCore);
	
	// transform
	racketPlaneCT = ComponentTransform::create();
	//racketPlaneCT->setRotation(Quaternion(Vec3f(1,0,0),osgDegree2Rad(-30.f)));
	//racketPlaneCT->setTranslation(Vec3f(0.f,37.f,-11.2f));
	//racketPlaneCT->setRotation(Quaternion(Vec3f(1,0,0),osgDegree2Rad(-30.f)));
	//racketPlaneCT->setTranslation(Vec3f(0.f,37.f,-11.2f));
	
	racketPlaneNode = makeNodeFor(racketPlaneCT);
	racketPlaneNode->setCore(racketPlaneCT);
	
	//transform2
	
	ComponentTransformRecPtr racketPlaneCT2 = ComponentTransform::create();
	racketPlaneCT2->setTranslation(Vec3f(0.f,20.0f,0.0f));
	
	NodeRecPtr racketPlaneNode2 = makeNodeFor(racketPlaneCT2);
	racketPlaneNode2->setCore(racketPlaneCT2);
	
	racketPlaneNode->addChild(racketPlaneNode2);
	racketPlaneNode2->addChild(racketPlaneMgNode);
	racketPlaneMgNode->addChild(racketPlaneModel);
	
	rootNode->addChild(racketPlaneNode);
}




NodeRecPtr ballCloneTransNode;
void loadBallModel(NodeRecPtr rootNode){
	NodeRecPtr ballModel = SceneFileHandler::the()->read("models/ball.osb");
	if(ballModel == NULL){
		std::cout << "models/ball.osb not avalable. Generateing it out of the model." << std::endl;
		// gut 2500polys
		//ballModel = SceneFileHandler::the()->read("models/balls/TennisBall_Rendering/gen/TennisBall_Rendering.obj");
		// kleinere rillen aber sch�n glatt 2100polys schnelles laden
		ballModel = SceneFileHandler::the()->read("models/balls/tennisballhigh/gen/tennisballhigh.obj");
		// gr�n, sehr glatt 46000polys
		//NodeRecPtr ballChild = SceneFileHandler::the()->read("models/balls/tennisballgreen/gen/tennis_ball.obj");
		SceneFileHandler::the()->write(ballModel, "models/ball.osb");
	} else{
		std::cout << "models/ball.osb loaded." << std::endl;
	}
	
	ComponentTransformRecPtr ballCT = ComponentTransform::create();
    ballCT->setTranslation(ballPositionUnvisible*100);
	//palmCT->setRotation(Quaternion(Vec3f(0,1,0),osgDegree2Rad(45)));
    ballCT->setScale(Vec3f(1, 1, 1));

	ballCloneTransNode = makeNodeFor(ballCT);
	ballCloneTransNode->setCore(ballCT);
	ballCloneTransNode->addChild(ballModel);

	// add TODO oder weg lassen
	//rootNode->addChild(ballCloneTransNode);
}



void fillVectorofBalls(){
  for(int i = 0; i < maxBalls; i++){
    // clone ball
    NodeRecPtr ballTransNodeClone = OSG::deepCloneTree(ballCloneTransNode);
    ComponentTransformRecPtr ballCTClone = dynamic_cast<ComponentTransform*>(ballTransNodeClone->getCore());

    // create ball obj
    Ball *newBall = new Ball(ballPositionUnvisible, Vec3f(0.f,0.f,0.f), ballTransNodeClone, ballCTClone);

    // add to ballsVector
    vectorOfBalls.push_back(newBall);
    
    rootNode->addChild(newBall->node);
  }
}

void addStadium(NodeRecPtr rootNode){
		/*
	Tennisplatz � die Ma�e
	Der Tennisplatz ist rechteckig, wobei die lange Seite 23,77 Meter lang und die breite Seite 10,97 Meter (inkl. Doppelfeld) breit ist. Im Einzel ist die Spielfl�che schmaler, da ja auf jeder Spielh�lfte �nur� ein Spieler steht. Die beiden �u�eren L�ngsstreifen rechnen daher nicht zum Einzelspielfeld. Im Einzel betr�gt die Breite eines Tennisplatzes genau genommen daher nur 8,23 Meter.
	Im Fernsehen sieht man daher manchmal Pl�tze, bei denen dieses Doppelfeld (verbreiterter Korridor an den beiden Seiten) fehlt. Wird nur Einzel gespielt, ist das grunds�tzlich kein Problem, wenngleich es optisch deutlich anders aussieht.

	Tennisplatz � das Netz
	In der Mitte teilt das Tennisnetz in zwei spiegelgleiche H�ften. Das Netz muss in der Mitte immer genau 91,4 cm hoch sein. Am Rand muss es 1,07 Meter hoch sein, wobei �Rand� beim Einzel weiter innen und beim Doppel weiter au�en (jeweils au�erhalb des Spielfeldes) ist. Um mit einem einheitlichen Netz spielen zu k�nnen, werden daher flexible Netzst�tzen in das f�r das Doppel installierte Netz eingesetzt, die H�he sowie Begrenzung des Netzes f�r das Einzel sicher- bzw. darstellen

	Tennisplatz � die Spielfl�chen
	Linien, die parallel und senkrecht zum Netz bzw. den R�ndern verlaufen, teilen das Spielfeld der Tennispl�tze in verschiedene Spielfl�chen.
	Die beiden kleinen Fl�chen am Netz sind nur als Zielfl�che f�r den Aufschlag relevant, der jeweils im diagonal gegen�berliegenden Feld (dem Aufschlagfeld) landen m�ssen. Die Linien selbst z�hlen �brigens immer zum g�ltigen Spielfeld. Diese Aufschlagfelder sind 6,40 Meter lang und 4,12 Meter breit. Dadurch entsteht optisch in der Draufsicht ein �T�. Daher nennt man diese beiden Felder auch T-Felder.
	Ansonsten z�hlt immer das ganze Spielfeld (im Doppel einschlie�lich der oben angesprochenen Doppelfelder) zur g�ltigen Fl�che.

	*/
	NodeRecPtr stadiumModel = SceneFileHandler::the()->read("models/stadium.osb");
	if(stadiumModel == NULL){
		std::cout << "models/stadium.osb not avalable. Generateing it out of the model." << std::endl;
		//stadiumModel = SceneFileHandler::the()->read("models/courts/myown/myown.3ds");
		stadiumModel = SceneFileHandler::the()->read("models/courts/myown/myown.wrl");
		
		SceneFileHandler::the()->write(stadiumModel, "models/stadium.osb");
	} else{
		std::cout << "models/stadium.osb loaded." << std::endl;
	}


	// transformation

	// rotation
	ComponentTransformRecPtr stadiumCT1 = ComponentTransform::create();
	//stadiumCT1->setRotation(Quaternion(Vec3f(1,0,0),osgDegree2Rad(-90))* Quaternion(Vec3f(0,0,1),osgDegree2Rad(180)));
	stadiumCT1->setRotation(Quaternion(Vec3f(1,0,0),osgDegree2Rad(-180))* Quaternion(Vec3f(0,0,1),osgDegree2Rad(180)));
     

	NodeRecPtr stadiumTransNode1 = makeNodeFor(stadiumCT1);
	stadiumTransNode1->setCore(stadiumCT1);

	// translation und scale
	ComponentTransformRecPtr stadiumCT2 = ComponentTransform::create();
	//stadiumCT->setTranslation(Vec3f(10,5,1));
	stadiumCT2->setTranslation(Vec3f(200.f,0.f,1200.f));
	// original length field 24cm
	stadiumCT2->setScale(Vec3f(100.f,100.f,100.f));    

	NodeRecPtr stadiumTransNode2 = makeNodeFor(stadiumCT2);
	stadiumTransNode2->setCore(stadiumCT2);

	//add
	stadiumTransNode2->addChild(stadiumModel);
	stadiumTransNode1->addChild(stadiumTransNode2);
	rootNode->addChild(stadiumTransNode1);
}

void addNet(NodeRecPtr rootNode){
	NodeRecPtr netPlaneModel = makePlane(1300,100,1,1);
	
	//material
	SimpleMaterialRecPtr testMat = SimpleMaterial::create();
	testMat->setDiffuse(Color3f(0,1.f,0.f));
	testMat->setTransparency(1.0f);
	
	MaterialGroupRecPtr netPlaneMgCore = MaterialGroup::create();
	netPlaneMgCore->setMaterial(testMat);
	
	NodeRecPtr netPlaneMgNode = makeNodeFor(netPlaneMgCore);
	netPlaneMgNode->setCore(netPlaneMgCore);
	
	// image
	//ImageRecPtr imageNet = Image::create();
	//imageNet->read("images/ground.jpg");

	// Create floor texture from floor image
	//SimpleTexturedMaterialRecPtr texNet = SimpleTexturedMaterial::create();
	//texNet->setImage(imageNet);


	// transform
	ComponentTransformRecPtr netPlaneCT = ComponentTransform::create();
	netPlaneCT->setTranslation(Vec3f(-200.f,45.f,-1200.f));

	netPlaneNode = makeNodeFor(netPlaneCT);
	netPlaneNode->setCore(netPlaneCT);
	
	// add
	netPlaneMgNode->addChild(netPlaneModel);
	netPlaneNode->addChild(netPlaneMgNode);
	
	rootNode->addChild(netPlaneNode);
}

void addNet2(NodeRecPtr rootNode){

		// image
	ImageRecPtr imageNet = Image::create();
	imageNet->read("models/courts/bwlarge.jpg");

	// Create floor texture from floor image
	SimpleTexturedMaterialRecPtr texNet = SimpleTexturedMaterial::create();
	texNet->setImage(imageNet);

	GeometryRecPtr netPlaneModel = makePlaneGeo(1300,100,1,1);
	netPlaneModel->setMaterial(texNet);

	NodeRecPtr net2 = Node::create();
	net2->setCore(netPlaneModel);

	



	// transform
	ComponentTransformRecPtr netPlaneCT = ComponentTransform::create();
	netPlaneCT->setTranslation(Vec3f(-200.f,50.f,-200.f));

	NodeRecPtr netPlaneNode2 = makeNodeFor(netPlaneCT);
	netPlaneNode2->setCore(netPlaneCT);
	
	// add


	net2->addChild(netPlaneNode2);
	
	rootNode->addChild(net2);
}

void addWalls(NodeRecPtr rootNode){
	//material
	SimpleMaterialRecPtr testMat = SimpleMaterial::create();
	testMat->setTransparency(1.0f);
	testMat->setDiffuse(Color3f(1.f,0.5f,0.5f));
	MaterialGroupRecPtr mgCore = MaterialGroup::create();
	mgCore->setMaterial(testMat);
  

	wallPlanesNode = Node::create();
	wallPlanesNode->setCore(Group::create());
	
	// front
	NodeRecPtr wallPlaneModelFront = makePlane(1800,400,1,1);
	
	NodeRecPtr wallPlaneMgFrontNode = makeNodeFor(mgCore);
	wallPlaneMgFrontNode->setCore(mgCore);
  
	// transform
	ComponentTransformRecPtr netPlaneFrontCT = ComponentTransform::create();
	netPlaneFrontCT->setTranslation(Vec3f(-200.f,200.f,-2800.f));
	
	NodeRecPtr wallPlaneFrontNode = makeNodeFor(netPlaneFrontCT);
	wallPlaneFrontNode->setCore(netPlaneFrontCT);
	
	wallPlaneMgFrontNode->addChild(wallPlaneModelFront);
	wallPlaneFrontNode->addChild(wallPlaneMgFrontNode);
	wallPlanesNode->addChild(wallPlaneFrontNode);

	// Back
	NodeRecPtr wallPlaneModelBack = makePlane(1800,400,1,1);
	
	NodeRecPtr wallPlaneMgBackNode = makeNodeFor(mgCore);
	wallPlaneMgBackNode->setCore(mgCore);
  
	// transform
	ComponentTransformRecPtr netPlaneBackCT = ComponentTransform::create();
	netPlaneBackCT->setTranslation(Vec3f(-200.f,200.f,+300.f));
	
	NodeRecPtr wallPlaneBackNode = makeNodeFor(netPlaneBackCT);
	wallPlaneBackNode->setCore(netPlaneBackCT);
	
	wallPlaneMgBackNode->addChild(wallPlaneModelBack);
	wallPlaneBackNode->addChild(wallPlaneMgBackNode);
	wallPlanesNode->addChild(wallPlaneBackNode);


	// left
	
	NodeRecPtr wallPlaneModelLeft = makePlane(3000,400,1,1);
	
	NodeRecPtr wallPlaneMgLeftNode = makeNodeFor(mgCore);
	wallPlaneMgLeftNode->setCore(mgCore);
  
	// transform
	ComponentTransformRecPtr netPlaneLeftCT = ComponentTransform::create();
	netPlaneLeftCT->setRotation(Quaternion(Vec3f(0,1,0),osgDegree2Rad(90.f)));
	netPlaneLeftCT->setTranslation(Vec3f(-200.f - 800.f, 200.f, -1200.f));
	
	NodeRecPtr wallPlaneLeftNode = makeNodeFor(netPlaneLeftCT);
	wallPlaneLeftNode->setCore(netPlaneLeftCT);
	
	wallPlaneMgLeftNode->addChild(wallPlaneModelLeft);
	wallPlaneLeftNode->addChild(wallPlaneMgLeftNode);
	wallPlanesNode->addChild(wallPlaneLeftNode);
	
	
	// right
	NodeRecPtr wallPlaneModelRight = makePlane(3000,400,1,1);
	
	NodeRecPtr wallPlaneMgRightNode = makeNodeFor(mgCore);
	wallPlaneMgRightNode->setCore(mgCore);
  
	// transform
	ComponentTransformRecPtr netPlaneRightCT = ComponentTransform::create();
	netPlaneRightCT->setRotation(Quaternion(Vec3f(0,1,0),osgDegree2Rad(90.f)));
	netPlaneRightCT->setTranslation(Vec3f(-200.f + 800.f, 200.f, -1200.f));
	
	NodeRecPtr wallPlaneRightNode = makeNodeFor(netPlaneRightCT);
	wallPlaneRightNode->setCore(netPlaneRightCT);
	
	wallPlaneMgRightNode->addChild(wallPlaneModelRight);
	wallPlaneRightNode->addChild(wallPlaneMgRightNode);
	wallPlanesNode->addChild(wallPlaneRightNode);





	rootNode->addChild(wallPlanesNode);

	
}


void addLight(){
	
	
	
	DirectionalLightRecPtr dirLight1 = DirectionalLight::create();

	//dirLight->setDirection(-0.25f,-0.5f,-1.f);
	dirLight1->setDirection(-1.f,1.f,1.f);

	//color information
	dirLight1->setDiffuse(Color4f(1,1,1,1));
	dirLight1->setAmbient(Color4f(0.2,0.2,0.2,1));
	dirLight1->setSpecular(Color4f(1,1,1,1));

	//wrap the root, cause only nodes below the lights will be lit
	NodeRecPtr ueberroot = makeNodeFor(dirLight1);
	
	
	DirectionalLightRecPtr dirLight2 = DirectionalLight::create();

	//dirLight->setDirection(-0.25f,-0.5f,-1.f);
	dirLight2->setDirection(1.f,1.f,-1.f);

	//color information
	dirLight2->setDiffuse(Color4f(1,1,1,1));
	dirLight2->setAmbient(Color4f(0.2,0.2,0.2,1));
	dirLight2->setSpecular(Color4f(1,1,1,1));

	//wrap the root, cause only nodes below the lights will be lit
	NodeRecPtr ueberroot2 = makeNodeFor(dirLight2);

	ueberroot2->addChild(ueberroot);

	


	ueberroot->addChild(rootNode);
	rootNode = ueberroot2;
	
	



}

void addWandDirection(NodeRecPtr rootNode){
	NodeRecPtr wandBox = makeBox(5,20,5,1,1,1);

	SimpleMaterialRecPtr centerBoxMat = SimpleMaterial::create();
	centerBoxMat->setDiffuse(Color3f(1,0.8f,0));
	centerBoxMat->setAmbient(Color3f(0.8f, 0.2f, 0.2f));
	MaterialGroupRecPtr centerBoxMgCore = MaterialGroup::create();
	centerBoxMgCore->setMaterial(centerBoxMat);

	racketPlaneFaceCT = ComponentTransform::create();
	//ComponentTransformRecPtr planeCT = dynamic_cast<ComponentTransform*>(racketPlaneCTNode->getCore());
	//racketPlaneFaceCT->setRotation(planeCT->getRotation() * Quaternion(Vec3f(1,0,0),osgDegree2Rad(-30.f)));
	//racketPlaneFaceCT->setRotation(wand_orientation * planeWandFixRotationForFaceQuaternion);

	//racketPlaneFaceCT->setTranslation(Vec3f(wand_position.getValues()[0], wand_position.getValues()[1]+0, wand_position.getValues()[2]));
	//racketPlaneFaceCT->setTranslation(wand_position);
	NodeRecPtr CTNode = makeNodeFor(racketPlaneFaceCT);
	CTNode->setCore(racketPlaneFaceCT);


	NodeRecPtr centerBoxChild = Node::create();
	CTNode->addChild(centerBoxChild);
	centerBoxChild->setCore(centerBoxMgCore);
	centerBoxChild->addChild(wandBox);
	rootNode->addChild(CTNode);
}

void addBallRacketRayDirection(NodeRecPtr rootNode){
	NodeRecPtr ballRayBox = makeBox(3,30,3,1,1,1);

	SimpleMaterialRecPtr centerBoxMat = SimpleMaterial::create();
	centerBoxMat->setDiffuse(Color3f(1,0.8f,0));
	centerBoxMat->setAmbient(Color3f(0.8f, 0.2f, 0.2f));
	MaterialGroupRecPtr centerBoxMgCore = MaterialGroup::create();
	centerBoxMgCore->setMaterial(centerBoxMat);

	ballRacketRayCT = ComponentTransform::create();
	//ComponentTransformRecPtr planeCT = dynamic_cast<ComponentTransform*>(racketPlaneCTNode->getCore());
	//racketPlaneFaceCT->setRotation(planeCT->getRotation() * Quaternion(Vec3f(1,0,0),osgDegree2Rad(-30.f)));
	//racketPlaneFaceCT->setRotation(wand_orientation * planeWandFixRotationForFaceQuaternion);

	//racketPlaneFaceCT->setTranslation(Vec3f(wand_position.getValues()[0], wand_position.getValues()[1]+0, wand_position.getValues()[2]));
	//racketPlaneFaceCT->setTranslation(wand_position);
	NodeRecPtr CTNode = makeNodeFor(ballRacketRayCT);
	CTNode->setCore(ballRacketRayCT);


	NodeRecPtr centerBoxChild = Node::create();
	CTNode->addChild(centerBoxChild);
	centerBoxChild->setCore(centerBoxMgCore);
	centerBoxChild->addChild(ballRayBox);
	rootNode->addChild(CTNode);
}

NodeTransitPtr buildScene()
{
	rootNode = Node::create();
	rootNode->setCore(Group::create());
	
	// http://www.opensg.org/wiki/Tutorial/OpenSG2/Basics#ImagesandTextures
	// 3ds, obj, wrl, stl
	// doku: VRML97, OFF, OBJ, RAW, OSG (OpenSG ASCII format), BIN (called OSB after 1.2), Collada
	// maps oder map ordner mit images wichtig!
	// 3ds kann nur farben und keine bmp bilder
	// wrl ladet lange und geht auch
	// obj geht aber dauert lange zum laden
	// 3ds l�dt keine texturen und zerlegt alles
	// stl l�dt sehr schnell aber keine texturen keine farbe

	//Make Skybox
	addSkyBox(rootNode);

	// centerBox
	//addCenterBox(rootNode);

	// RACKET LENGTH 60CM
	// racket 
	addRacket(rootNode);

	addRacketPlane(rootNode);
	
	// ball
	loadBallModel(rootNode);
	// clone balls and add to vector
	fillVectorofBalls();

	//commandSortBalls();
	
	// stadium
	addStadium(rootNode);
	
	addNet(rootNode);
	//addNet2(rootNode);
	
	//addWalls(rootNode);
	
	

	//addWandDirection(rootNode);
	//addBallRacketRayDirection(rootNode);
	
	// light
	addLight();
	
	return NodeTransitPtr(rootNode);
}


// calc frameDelay
Real32 frame=0;
Real32 timeFrame=0;
Real32 timeLastFrame=-1;

float fps;
void calculateFrameDelay(){
	timeFrame= (float)glutGet(GLUT_ELAPSED_TIME) / 1000;

	frameTimeDeltaSeconds= timeFrame - timeLastFrame;
	timeLastFrame= timeFrame;

}

// calc racket velocity
Vec3f lastWand_position = NULL;
Vec3f lastWand2_position = NULL;
Vec3f lastRacketVectorCM = NULL;

void calcualteWandVectors(){
	// racket
	
	if(lastWand_position == NULL){
		lastWand_position = wand_position;
	}
	racketVectorCM = (wand_position - lastWand_position);
	lastWand_position = wand_position;
	
	
	if(lastRacketVectorCM == NULL){
	  lastRacketVectorCM = racketVectorCM;
	}
	racketVectorDifferenceCM = racketVectorCM - lastRacketVectorCM;
	lastRacketVectorCM = racketVectorCM;
	
	
	// wand2 ball
	if(lastWand2_position == NULL){
		lastWand2_position = wand2_position;
	}
	wand2VectorCM = (wand2_position - lastWand2_position);
	lastWand2_position = wand2_position;
}

/*
bool collision(VRGPhysicsObject obj1, VRGPhysicsObjectobj2){
    Line ray = Line(obj1.getPosition() + obj1.getDirection(), obj1.getDirection());
    IntersectActionRefPtr iAct = (IntersectActionRefPtr)IntersectAction::create();
    iAct->setLine(ray, general::hitDistance);
    NodeRefPtr someNode = obj2.getRootNode();
    iAct->apply((Node * const)someNode);
    if (iAct->didHit()){
      reflectionVector = calcReflectionVector(obj1.getDirection(),iAct->getHitNormal());
      // double dotProd = 1 - abs(iAct->getHitNormal() * Vec3f(0,1,0));
      return true;
    }
      return false;
}
*/

/*
// (ballrichtung, planenormale)
Vec3f calcReflectionDirectionVector(Vec3f direction, Vec3f normal){
    Vec3f tempDirection = Vec3f(direction);
    tempDirection.normalize();
    normal.normalize();
    Vec3f r = 2 * (-direction.dot(normal)) * normal + direction;
    r.normalize();
    return r;
}
*/

// (ballrichtung, planenormale)
Vec3f calcReflectionDirectionVector(Vec3f dir, Vec3f nor){
    Vec3f d = Vec3f(dir);
	Vec3f n = Vec3f(nor);
    d.normalize();
    n.normalize();

	const Real32 two = 2;

    Vec3f r = d - (((two * d).dot(n))/std::abs(n*n))*n;

    r.normalize();
    return r;
}

// richtung vorhand
Vec3f getRackePlaneFaceDirection(){
    // TODO TODO TODO TODO TODO test
    Matrix m;
    // +x rechte SchlägerSeite, -x linke SchlägerSeite
    m.setRotate(wand_orientation * wandFixRotationQuaternion);
    //Vec3f dir = m * Vec3f(0.f, 0.f, -1.f);
    Vec3f dir = m * Vec3f(0.f, 0.f, -1.f);
    dir.normalize();
    dir.negate();
    return dir;
}


// gravitation
// 9,81 m/s²
const Real32 gravitation = 9.81f ; 
const Vec3f gravitationDirectionVector = Vec3f(0.f, -1.f, 0.f);

// air resistance
const Real32 airResistance = 1.8f;
const Real32 airDensity = 1.2041; // kg/m³
const Real32 dragCoefficient = 0.4;
const Real32 ballCrossSection = 3.14159 * ballRadiusMeter * ballRadiusMeter;
const Real32 ballMass = 0.058; //kg


// 
const Real32 groundResistancePercentFactor = 0.8f;



// collusion detection
// 0 = no ray collusion, 1 = ray collusion Side1, 2 = rayCollusion Side2
Real32 lastBallRayState = 0;




void displayTennisBall(Ball& ball) {
	
	Real32 t = frameTimeDeltaSeconds;

	Vec3f newballPositionPointMeter;
	Vec3f newBallVelocityVectorMeterPerSeconds = ball.ballVelocityVectorMeterPerSeconds;
	
	//t = glutGet(GLUT_ELAPSED_TIME) / 1000;

	//std::cout << frameTimeDeltaMs << "\n";
	// 16-18ms zwischen frames

	// the new position and Direction
	//newDirectionVector = ball.directionVector + (gravitationVector * frameTimeDeltaMs / 1000);
	// Fallbeschleunigung 
	// Der reibungsfreie Wurf
	// https://de.wikipedia.org/wiki/Wurfparabel
	// https://de.wikipedia.org/wiki/Fall_mit_Luftwiderstand
	// http://www.matheplanet.com/default3.html?call=article.php?sid=1501&ref=http%3A%2F%2Fwww.google.de%2Furl%3Fsa%3Dt%26rct%3Dj%26q%3D%26esrc%3Ds%26source%3Dweb%26cd%3D3%26ved%3D0ahUKEwiY6tuauaXPAhWNyRoKHV-3B2gQFggsMAI
	// https://wiki.zum.de/wiki/GTR_und_TC_mit_CAS_in_Physik/Aufgaben/Bewegungen_mit_Luftwiderstand
	// https://www.c-plusplus.net/forum/272858-full

	// s = v*t

	Real32 vxOld = newBallVelocityVectorMeterPerSeconds.getValues()[0];
	Real32 vyOld = newBallVelocityVectorMeterPerSeconds.getValues()[1];
	Real32 vzOld = newBallVelocityVectorMeterPerSeconds.getValues()[2];

	
	Real32 vx = vxOld - ((airDensity * dragCoefficient * ballCrossSection * vxOld*vxOld) / 2 * ballMass ) * t;
	Real32 vy = vyOld - (gravitation - ( (airDensity * dragCoefficient * ballCrossSection * (gravitation*t) * (gravitation*t) ) / 2 * ballMass)) * t;
	Real32 vz = vzOld - ((airDensity * dragCoefficient * ballCrossSection * vzOld*vzOld) / 2 * ballMass ) * t;
	
	//Vec3f GravWayMeters = Vec3f(newBallVelocityVectorMeterPerSeconds.getValues()[0], newBallVelocityVectorMeterPerSeconds.getValues()[1] * t - (gravitation / 2) * t * t, newBallVelocityVectorMeterPerSeconds.getValues()[2]);
	//newBallVelocityVectorMeterPerSeconds = GravWayMeters / t;
	newBallVelocityVectorMeterPerSeconds = Vec3f(vx,vy,vz);

	// air resistance
	// TODO
	//Vec3f airResistanceDirectionVector = newBallVelocityVectorMeterPerSeconds * -1.f;	   
	//airResistanceDirectionVector.normalize();
	//Vec3f airResistanceVector = airResistanceDirectionVector * airResistance * t;

	
	// together2
	//newBallVelocityVectorMeterPerSeconds = newBallVelocityVectorMeterPerSeconds  + airResistanceVector;
	//newballPositionPointMeter = ball.ballPositionPointMeter + GravWayMeters; 
	newballPositionPointMeter = ball.ballPositionPointMeter + newBallVelocityVectorMeterPerSeconds * t;


	
	// other 
	Real32 ballDistanceMeter = newBallVelocityVectorMeterPerSeconds.length() * t; 
	Real32 racketVectorDistanceMeter = (racketVectorCM/ 100).length();
	Vec3f racketVectorVelocityMeterPerSeconds = racketVectorCM / 100 / t;
	
	
	// racket acceleration
	Real32 racketDeltaMeter = std::abs((racketVectorDifferenceCM/100).length());
	Real32 racketAccelerationMeterSeconds = racketDeltaMeter / t;

	// ball entferung zur planeface in vorhand face richtung
	//Real32 hCM = (ball.ballPositionPointMeter * 100 - wand_position ).dot(getRackePlaneFaceDirection());
	//Real32 hMeter = hCM / 100;
	



	//
	// Check if the new Derection and Position are valid... ground racket walls...
	//

	// check Ground
	if(newballPositionPointMeter.getValues()[1] <= 0.f + ballRadiusMeter){
	  // ground collusion
	 
	  // hardcore set pos
	  newballPositionPointMeter.setValues(newballPositionPointMeter.getValues()[0], ballRadiusMeter, newballPositionPointMeter.getValues()[2]);
	  
	  //std::cout << "Hit Ground"<< "\n";
	  //std::cout << "BallPos: " << ball.positionPoint << "\t\t newDirectionVector:" << newDirectionVector <<" newDirectionVectorLength: " << newDirectionVector.length() << '\n';
	  //std::cout << "Hit ballpos : "<< ball.positionPoint << " Hit Normal: " << Vec3f(0.f,1.f,0.f) << "\n";
	  
	  Vec3f reflectionDirectionVector = calcReflectionDirectionVector(newBallVelocityVectorMeterPerSeconds, Vec3f(0.f,1.f,0.f));  
	  //std::cout << " reflectionDirectionVector: " << reflectionDirectionVector << "\n";	    
	  //std::cout << "\n";
	  
	  // direction minus distanz zum boden
	  // TODO eventuell nicht richtig
	  // TODO geht nur bei senkrechten bällen -> ray wird benötigt 
	  // getHitT() gibt distanz zum getroffenen punkt
	  //newDirectionVector = reflectionDirectionVector * (newDirectionVector.length() - newballPositionPointMeter.getValues()[1] );	  

	  newBallVelocityVectorMeterPerSeconds = reflectionDirectionVector * newBallVelocityVectorMeterPerSeconds.length();
	  newBallVelocityVectorMeterPerSeconds = newBallVelocityVectorMeterPerSeconds * groundResistancePercentFactor;
	} 

	//
	//check walls
	//
	if(newballPositionPointMeter.getValues()[1] < 4.f - ballRadiusMeter){
	  // is in wall hight
	  Vec3f wallSpeedReduce = 0.90f;
	  
	  // front & back wall
	  if(newballPositionPointMeter.getValues()[2] < -28.f + ballRadiusMeter){
	    // front collusion
	  
	  
	    //std::cout << "Hit front wall"<< "\n";
	    //std::cout << "BallPos: " << ball.positionPoint << "\t\t newDirectionVector:" << newDirectionVector <<" newDirectionVectorLength: " << newDirectionVector.length() << '\n';
	    //std::cout << "Hit ballpos : "<< ball.positionPoint << " Hit Normal: " << Vec3f(0.f,1.f,0.f) << "\n";
	    
	    Vec3f reflectionDirectionVector = calcReflectionDirectionVector(newBallVelocityVectorMeterPerSeconds, Vec3f(0.f,0.f,1.f));  
	    //std::cout << " reflectionDirectionVector: " << reflectionDirectionVector << "\n";	    
	    //std::cout << "\n";
	    
	    newBallVelocityVectorMeterPerSeconds = reflectionDirectionVector * newBallVelocityVectorMeterPerSeconds.length();
	    newBallVelocityVectorMeterPerSeconds = newBallVelocityVectorMeterPerSeconds * wallSpeedReduce;
	    newballPositionPointMeter = ball.ballPositionPointMeter;
	    
	  } else if(newballPositionPointMeter.getValues()[2] > 3.f - ballRadiusMeter) {
	    // back collusion

	    //std::cout << "Hit back wall"<< "\n";
	    //std::cout << "BallPos: " << ball.positionPoint << "\t\t newDirectionVector:" << newDirectionVector <<" newDirectionVectorLength: " << newDirectionVector.length() << '\n';
	    //std::cout << "Hit ballpos : "<< ball.positionPoint << " Hit Normal: " << Vec3f(0.f,1.f,0.f) << "\n";
      
	    Vec3f reflectionDirectionVector = calcReflectionDirectionVector(newBallVelocityVectorMeterPerSeconds, Vec3f(0.f,0.f,-1.f));  
	    //std::cout << " reflectionDirectionVector: " << reflectionDirectionVector << "\n";	    
	    //std::cout << "\n";
      
	    newBallVelocityVectorMeterPerSeconds = reflectionDirectionVector * newBallVelocityVectorMeterPerSeconds.length();
	    newBallVelocityVectorMeterPerSeconds = newBallVelocityVectorMeterPerSeconds * wallSpeedReduce;
	    newballPositionPointMeter = ball.ballPositionPointMeter;
	  }

	  // left & right wall
	  if(newballPositionPointMeter.getValues()[0] < -2.f - 8.f + ballRadiusMeter){
	    // left collusion
	  
	    //std::cout << "Hit left wall"<< "\n";
	    //std::cout << "BallPos: " << ball.positionPoint << "\t\t newDirectionVector:" << newDirectionVector <<" newDirectionVectorLength: " << newDirectionVector.length() << '\n';
	    //std::cout << "Hit ballpos : "<< ball.positionPoint << " Hit Normal: " << Vec3f(0.f,1.f,0.f) << "\n";
	    
	    Vec3f reflectionDirectionVector = calcReflectionDirectionVector(newBallVelocityVectorMeterPerSeconds, Vec3f(1.f,0.f,0.f));  
	    //std::cout << " reflectionDirectionVector: " << reflectionDirectionVector << "\n";	    
	    //std::cout << "\n";
	    
	    newBallVelocityVectorMeterPerSeconds = reflectionDirectionVector * newBallVelocityVectorMeterPerSeconds.length();
	    newBallVelocityVectorMeterPerSeconds = newBallVelocityVectorMeterPerSeconds * wallSpeedReduce;
	    newballPositionPointMeter = ball.ballPositionPointMeter;
	  } else if(newballPositionPointMeter.getValues()[0] > -2.f + 8.f - ballRadiusMeter) {
	    // right collusion

	    //std::cout << "Hit right wall"<< "\n";
	    //std::cout << "BallPos: " << ball.positionPoint << "\t\t newDirectionVector:" << newDirectionVector <<" newDirectionVectorLength: " << newDirectionVector.length() << '\n';
	    //std::cout << "Hit ballpos : "<< ball.positionPoint << " Hit Normal: " << Vec3f(0.f,1.f,0.f) << "\n";
      
	    Vec3f reflectionDirectionVector = calcReflectionDirectionVector(newBallVelocityVectorMeterPerSeconds, Vec3f(-1.f,0.f,0.f));  
	    //std::cout << " reflectionDirectionVector: " << reflectionDirectionVector << "\n";	    
	    //std::cout << "\n";
      
	    newBallVelocityVectorMeterPerSeconds = reflectionDirectionVector * newBallVelocityVectorMeterPerSeconds.length();
	    newBallVelocityVectorMeterPerSeconds = newBallVelocityVectorMeterPerSeconds * wallSpeedReduce;
	    newballPositionPointMeter = ball.ballPositionPointMeter;
	  }
	}


	// check net
	//
	IntersectActionRefPtr iActNet = IntersectAction::create();
	Line ballRayNet = Line(newballPositionPointMeter*100, newBallVelocityVectorMeterPerSeconds);
	iActNet->setLine(ballRayNet, ballDistanceMeter*100);
	iActNet->apply((Node * const)netPlaneNode);

	if (iActNet->didHit()) {

		Vec3f netSpeedReduce = 0.75f;

		Vec3f netPlaneNormal ;
		if (ball.ballPositionPointMeter.getValues()[2] >= -12.f){
			// hit on user side
			netPlaneNormal = Vec3f(0.f, 0.f, 1.f);
		} else {
			// hit on server side
			netPlaneNormal = Vec3f(0.f, 0.f, -1.f);
		}				
		
		Vec3f reflectionDirectionVector = calcReflectionDirectionVector(newBallVelocityVectorMeterPerSeconds, netPlaneNormal);

		newBallVelocityVectorMeterPerSeconds= reflectionDirectionVector * newBallVelocityVectorMeterPerSeconds.length() * netSpeedReduce;
		
		newballPositionPointMeter = ball.ballPositionPointMeter + newBallVelocityVectorMeterPerSeconds * t;

		std::cout << "Hit net"<< "\n";
		std::cout << "Hit Point: "<< iActNet->getHitPoint() << "HitNormFromAction: "<< iActNet->getHitNormal() << " netPlaneNormal: " << netPlaneNormal << " reflectionDirectionVector: " << reflectionDirectionVector << "\n";
		std::cout << "BallPos: " << ball.ballPositionPointMeter << " DirectionVector:" << ball.ballPositionPointMeter << " newDirectionVector:" << newBallVelocityVectorMeterPerSeconds << " newDirectionVectorLength: " << newBallVelocityVectorMeterPerSeconds.length() << '\n';    
		std::cout << "\n";
	}


	// check racket
	// via vorhand and rueckhand wechsel, hcm, ray


	
	IntersectActionRefPtr iActRacket1 = IntersectAction::create();
	IntersectActionRefPtr iActRacket2 = IntersectAction::create();
	
	// make a Ray from Ball with Racket Plane Oriantation
	// problem with two getRackePlaneFaceDirection
	Vec3f ballRacketFaceDirection1 = getRackePlaneFaceDirection();
	Vec3f ballRacketFaceDirection2 = getRackePlaneFaceDirection();
	ballRacketFaceDirection2.negate();
	
	// für den fix mit dem radus des balls 
	// osgSgn gibt vorzeichen zurück
	Real32 hRayCM = ( ball.ballPositionPointMeter* 100 - wand_position ).dot(getRackePlaneFaceDirection())  ;
	Vec3f ballNearestDirectionToRacketInCM = ballRacketFaceDirection1 * ballRadiusCM;
	if (hRayCM > 0) {
	  ballNearestDirectionToRacketInCM.negate();
	}
	Vec3f ballRadiusRayStartPointCM = newballPositionPointMeter * 100 + ballNearestDirectionToRacketInCM;
	
	//ballRadiusRayStartPointCM = newballPositionPointMeter * 100 + ballRacketFaceDirection1 * ballRadiusCM;
	
	Line ballRayRacket1 = Line(ballRadiusRayStartPointCM, ballRacketFaceDirection1);
	Line ballRayRacket2 = Line(ballRadiusRayStartPointCM, ballRacketFaceDirection2);
	
	iActRacket1->setLine(ballRayRacket1, 300);
	iActRacket2->setLine(ballRayRacket2, 300);
	iActRacket1->apply((Node * const)racketPlaneNode);
	iActRacket2->apply((Node * const)racketPlaneNode);
	
	Real32 ballRayState = 0;
	Real32 lastBallRayState = ball.ballRayState;

	if (iActRacket1->didHit() || iActRacket2->didHit()) {
	   // Racket collision
	  Vec3f ballRacketFaceDirection;
	  Vec3f racketBallFaceDirection;
	  IntersectActionRefPtr iActRacket;

	  
	  if(iActRacket1->didHit()){
	    // vorhand
	    iActRacket = iActRacket1;
	    ballRacketFaceDirection = ballRacketFaceDirection1;
		racketBallFaceDirection = ballRacketFaceDirection2;
	    //std::cout << "Hit iActRacket1 " << "ballRacketFaceDirection: " << ballRacketFaceDirection  ;
		ballRayState = 1;

	  } else{
	    // rueckhand
	    iActRacket = iActRacket2;
	    ballRacketFaceDirection = ballRacketFaceDirection2;
		racketBallFaceDirection = ballRacketFaceDirection1;
	    //std::cout << "Hit iActRacket2 " << "ballRacketFaceDirection: " << ballRacketFaceDirection  ;
		ballRayState = 2;
	    
	  }

	  //std::cout << " ballRayState: " << ballRayState << " lastBallRayState: " << lastBallRayState << "\n";


	  // ist durch check
	  //if(lastHMeter < 0 && hMeter < 0 || lastHMeter >= 0 && hMeter >= 0){
	  if((ballRayState == 1 && lastBallRayState == 2) || (ballRayState == 2 && lastBallRayState == 1)){
		
		// ist  durch
		// !Achtung! verwende die jeweils andere richtung weil der ball ja auf der anderen seite ist
		Vec3f tmpBallRacketFaceDirection = ballRacketFaceDirection;
		ballRacketFaceDirection = racketBallFaceDirection;
		racketBallFaceDirection = tmpBallRacketFaceDirection;

		

		// test getRackePlaneFaceDirection() geht nicht oder calcReflectionDirectionVector()!!!
		//Vec3f reflectionDirectionVector = calcReflectionDirectionVector(newBallVelocityVectorMeterPerSeconds, racketBallFaceDirection);


		// test calcReflectionDirectionVector2r
		//http://math.stackexchange.com/questions/13261/how-to-get-a-reflection-vector
		Vec3f reflectionDirectionVector = calcReflectionDirectionVector(newBallVelocityVectorMeterPerSeconds, racketBallFaceDirection);
		
		Vec3f ballPosMeter = Vec3f(iActRacket->getHitPoint().getValues()[0], iActRacket->getHitPoint().getValues()[1], iActRacket->getHitPoint().getValues()[2]) / 100;
		/*
		Real32 hMeterAbs = std::abs(hMeter);f
		Real32 newBallRacketHitDistanceMeter = std::abs((ballPosMeter - newballPositionPointMeter).length());
		Real32 ballRacketHitDistanceMeter = std::abs((ballPosMeter - ball.ballPositionPointMeter).length());
		*/
		const Real32 racketResistancePercentFactor = 0.7;
		const Real32 racketBouncePercentFactor = 3.0;


	  // TODO
	  // ca 81km/h = 81000m/h = 22.5 m/s = 2250 cm/s
	  // test war max 7-10 racketVector.length()
	  //racketVectorCM = Vec3f(0,5,0);
	  //racketVectorDistanceMeter = (racketVectorCM/ 100).length();
	  //racketVectorVelocityMeterPerSeconds = racketVectorCM / 100 / t;
	  // TODO testing


		// racket reflection
		newBallVelocityVectorMeterPerSeconds = reflectionDirectionVector * newBallVelocityVectorMeterPerSeconds.length();
		newBallVelocityVectorMeterPerSeconds = newBallVelocityVectorMeterPerSeconds * racketResistancePercentFactor;
		
		//newBallVelocityVectorMeterPerSeconds =  reflectionDirectionVector * (newBallVelocityVectorMeterPerSeconds.length() - hMeterAbs/t);
		//newBallVelocityVectorMeterPerSeconds =  reflectionDirectionVector * (newBallVelocityVectorMeterPerSeconds.length()- ballRacketHitDistanceMeter/t);
		//newBallVelocityVectorMeterPerSeconds = Vec3f(reflectionDirectionVector.getValues()[0] * ballDistanceMeter, reflectionDirectionVector.getValues()[1] * ballDistanceMeter, reflectionDirectionVector.getValues()[2] * ballDistanceMeter);
		
		// racketvector
		//newBallVelocityVectorMeterPerSeconds = newBallVelocityVectorMeterPerSeconds * racketVectorCM / 100 * racketVectorDistanceMeter / t ;
		//
		//newBallVelocityVectorMeterPerSeconds = newBallVelocityVectorMeterPerSeconds + racketVectorVelocityMeterPerSeconds * racketBouncePercentFactor;
		newBallVelocityVectorMeterPerSeconds = newBallVelocityVectorMeterPerSeconds + racketBallFaceDirection * racketVectorVelocityMeterPerSeconds.length() * racketBouncePercentFactor;

		// ball position
		//newballPositionPointMeter = setBallPosMeter + newBallVelocityVectorMeterPerSeconds * t ;//+ racketBallFaceDirection * ballRadiusMeter;
		//newballPositionPointMeter = setBallPosMeter + racketBallFaceDirection * newBallVelocityVectorMeterPerSeconds.length() * t ;
		//newballPositionPointMeter = ballPosMeter + reflectionDirectionVector * newBallRacketHitDistanceMeter ;
		newballPositionPointMeter = ballPosMeter - ballNearestDirectionToRacketInCM / 100.f;//+ racketBallFaceDirection * ballRadiusMeter ;

		/*
		std::cout << "War durch " <<"\n";
		std::cout << " ballRayState: " << ballRayState << " lastBallRayState: " << lastBallRayState << "\n";
	    std::cout << "tDelta: " << t << " hCM: " << hCM << " ballRacketFaceDirection: " << ballRacketFaceDirection << " racketBallFaceDirection: " << racketBallFaceDirection <<"\n";
	    std::cout << "Racket racketVectorCM " << racketVectorCM << " racketVectorDistanceMeter: " << racketVectorDistanceMeter << "\n";
	    std::cout << "Hit point: "<< iActRacket->getHitPoint() << " HitNormFromAction: "<< iActRacket->getHitNormal() << " getRackePlaneFaceDirection(): " << getRackePlaneFaceDirection() << " reflectionDirectionVector: " << reflectionDirectionVector << "\n";
		std::cout << "ballPositionPointMeter: " << ball.ballPositionPointMeter << " ballVelocityVectorMeterPerSeconds:" << ball.ballVelocityVectorMeterPerSeconds << "\n";
		std::cout << "newballPositionPointMeter: " << newballPositionPointMeter << " newBallVelocityVectorMeterPerSeconds:" << newBallVelocityVectorMeterPerSeconds << " newBallVelocityVectorMeterPerSecondsLength: " << newBallVelocityVectorMeterPerSeconds.length() << '\n';    
	    std::cout << "\n";
		*/

		// zwei zu eins, eins zu zwei
		ballRayState = lastBallRayState;

	  }

	  
	}else{
		ballRayState = 0;
	}

	// ballRayState
	ball.ballRayState = ballRayState;














	/*
	// check racket
	// via ray length
	// TODO

	Vec3f ballRacketFaceDirection;
	// fix ray direction from ball to racket with hCM
	if (hCM >= 0){
	  // vorhand
	  ballRacketFaceDirection = getRackePlaneFaceDirection();
	  ballRacketFaceDirection.negate();
	} else {
	  // rueckhand
	  ballRacketFaceDirection = getRackePlaneFaceDirection();
	}
	
	Line ballRay = Line(ball.ballPositionPointMeter*100, ballRacketFaceDirection);
	  
	
	  IntersectActionRefPtr iActRacket = IntersectAction::create();
	  
	  // rayLength
	  Real32 rayLengthMeter = 0;
	  rayLengthMeter = rayLengthMeter + ballDistanceMeter;
	  rayLengthMeter = rayLengthMeter + racketVectorDistanceMeter;
	  //rayLengthMeter = rayLengthMeter + ballRadiusMeter;
	  // Gleichmäßig beschleunigte Bewegung 
	  rayLengthMeter = rayLengthMeter + 0.5 * racketAccelerationMeterSeconds * t*t;
	  
	  iActRacket->setLine(ballRay, rayLengthMeter * 100);
	  iActRacket->apply((Node * const)racketPlaneNode);
	  
	  if (iActRacket->didHit()) {
	    // Racket collision
	    Vec3f racketBallFaceDirection = ballRacketFaceDirection;
	    racketBallFaceDirection.negate();
	    
	    Vec3f reflectionDirectionVector = calcReflectionDirectionVector(newBallVelocityVectorMeterPerSeconds, racketBallFaceDirection); 

	    // TODO velocity + oder *
	    // ca 81km/h = 81000m/h = 22.5 m/s = 2250 cm/s
	    // test war max 7-10 racketVector.length()
	    //racketVector = Vec3f(0,0,5);
	    // TODO testing

	    // TODO
	    
	    //newBallVelocityVectorMeterPerSeconds = reflectionDirectionVector * (newBallVelocityVectorMeterPerSeconds + racketVectorDistanceMeter / t );
	    newBallVelocityVectorMeterPerSeconds = reflectionDirectionVector * ballDistanceMeter / t + racketVectorCM / 100 * racketVectorDistanceMeter / t ;
	    


	    // todo test
	    
	    //newballPositionPointMeter = ball.ballPositionPointMeter + racketBallFaceDirection * ballRadiusMeter + racketBallFaceDirection * ballDistanceMeter;
	    
	    
	    // check if <0 then it went throu the racket
	    //Real32 newhCM = (newballPositionPointMeter * 100 - wand_position ).dot(ballRacketFaceDirection);
	    //if(newhCM < 0){
	    //  std::cout << "fuck is durch! newhCM: " << newhCM << "\n";
	    //}
	    
	    
	    
	    //newballPositionPointMeter = newballPositionPointMeter + ballRadiusMeter * racketPlaneNormal;

	    std::cout << "Hit racket"<< "\n";
	    std::cout << "rayLengthMeter: " << rayLengthMeter << " hCM: " << hCM << " ballRacketFaceDirection: " << ballRacketFaceDirection << " racketBallFaceDirection: " << racketBallFaceDirection <<"\n";
	    std::cout << "Racket racketVectorCM " << racketVectorCM << " racketVectorCMLength: " << racketVectorCM.length() << "\n";
	    std::cout << "Hit point: "<< iActRacket->getHitPoint() << " HitNormFromAction: "<< iActRacket->getHitNormal() << " getRackePlaneFaceDirection(): " << getRackePlaneFaceDirection() << " reflectionDirectionVector: " << reflectionDirectionVector << "\n";
		std::cout << "ballPositionPointMeter: " << ball.ballPositionPointMeter << " ballVelocityVectorMeterPerSeconds:" << ball.ballVelocityVectorMeterPerSeconds << "\n";
		std::cout << "newballPositionPointMeter: " << newballPositionPointMeter << " newBallVelocityVectorMeterPerSeconds:" << newBallVelocityVectorMeterPerSeconds << " newBallVelocityVectorMeterPerSecondsLength: " << newBallVelocityVectorMeterPerSeconds.length() << '\n';    
	    std::cout << "\n";
	  }
	*/

	/*
	//
	// check racket
	// TODO
	IntersectActionRefPtr iActRacket1 = IntersectAction::create();
	IntersectActionRefPtr iActRacket2 = IntersectAction::create();
	
	// make a Ray from Ball with Racket Plane Oriantation
	// problem with two getRackePlaneFaceDirection
	Vec3f ballRayRacketDirection1 = getRackePlaneFaceDirection();
	Vec3f ballRayRacketDirection2 = getRackePlaneFaceDirection();
	ballRayRacketDirection2.negate();
	
	Line ballRayRacket1 = Line(ball.ballPositionPointMeter * 100, ballRayRacketDirection1);
	Line ballRayRacket2 = Line(ball.ballPositionPointMeter * 100, ballRayRacketDirection2);
	
	Real32 rayLineDistanceMeter = 1 * (ballDistanceMeter * 100 + racketVectorDistanceMeter * 100);
	
	iActRacket1->setLine(ballRayRacket1, rayLineDistanceMeter);
	iActRacket2->setLine(ballRayRacket2, rayLineDistanceMeter);
	iActRacket1->apply((Node * const)racketPlaneNode);
	iActRacket2->apply((Node * const)racketPlaneNode);
	
	if (iActRacket1->didHit() || iActRacket2->didHit()) {
	   // Racket collision
	  Vec3f racketPlaneNormal;
	  IntersectActionRefPtr iActRacket;
	  Line ballRayRacket;
	  Vec3f ballRayRacketDirection;
	  
	  if(iActRacket1->didHit()){
	    // vorhand
	    iActRacket = iActRacket1;
	    racketPlaneNormal = iActRacket1->getHitNormal();
	    racketPlaneNormal = getRackePlaneFaceDirection();
	    ballRayRacket = ballRayRacket1;
	    ballRayRacketDirection = ballRayRacketDirection1;
	    std::cout << "Hit racket vorhand " <<"\n";
	  } else{
	    // rueckhand
	    iActRacket = iActRacket2;
	    racketPlaneNormal = iActRacket2->getHitNormal();
	    racketPlaneNormal = getRackePlaneFaceDirection();
	    racketPlaneNormal.negate();
	    ballRayRacket = ballRayRacket2;
	    ballRayRacketDirection = ballRayRacketDirection2;
	    std::cout << "Hit racket rueckhand " <<"\n";
	    
	  }

	  Vec3f reflectionDirectionVector = calcReflectionDirectionVector(newBallVelocityVectorMeterPerSeconds, racketPlaneNormal); 

	  // TODO velocity + oder *
	  // ca 81km/h = 81000m/h = 22.5 m/s = 2250 cm/s
	  // test war max 7-10 racketVector.length()
	  //racketVectorCM = Vec3f(0,0,-5);
	  //racketVectorDistanceMeter = (racketVectorCM/ 100).length() * t;
	  //racketVectorVelocityMeterPerSeconds = racketVectorCM / 100 / t;
	  // TODO testing

	  
	  // TODO es wird nur der reflectionvector als richtung berücksichtigt nicht die schlagdirection
	  // ballrevlection + racketdirection
	  newBallVelocityVectorMeterPerSeconds = (reflectionDirectionVector * newBallVelocityVectorMeterPerSeconds.length()) + (reflectionDirectionVector * racketVectorVelocityMeterPerSeconds.length()); 
	  newballPositionPointMeter = ball.ballPositionPointMeter + racketVectorVelocityMeterPerSeconds.length() * t;
	  

	  std::cout << "Ray "<< ballRayRacket << " ballRayRacketDirection: " << ballRayRacketDirection << " rayLineLength: " << rayLineDistanceMeter << "\n";
	  std::cout << "Racket racketVectorCM " << racketVectorCM << " racketVectorCMLength: " << racketVectorCM.length() << "\n";
	  std::cout << "Hit point: "<< iActRacket->getHitPoint() << " HitNormFromAction: "<< iActRacket->getHitNormal() << " racketPlaneNormal: " << racketPlaneNormal << " reflectionDirectionVector: " << reflectionDirectionVector << "\n";
	  std::cout << "Ball pos: " << ball.ballPositionPointMeter << " DirectionVector:" << ball.ballPositionPointMeter << " newDirectionVector:" << newBallVelocityVectorMeterPerSeconds << " newDirectionVectorLength: " << newBallVelocityVectorMeterPerSeconds.length() << '\n';    
	  std::cout << "\n";
 
	}
	*/


	/*
	//
	// check racket
	// TODO
	
	IntersectActionRefPtr iActRacket = IntersectAction::create();
	Line ballRay = Line(ball.ballPositionPointMeter*100, newBallVelocityVectorMeterPerSeconds);
	
	iActRacket->setLine(ballRay, ballDistanceMeter * 100 + racketVectorDistanceMeter * 100);
	iActRacket->apply((Node * const)racketPlaneNode);
	
	if (iActRacket->didHit()) {
	  // Racket collision
	  Vec3f racketPlaneNormal = iActRacket->getHitNormal();
	  racketPlaneNormal = getRackePlaneFaceDirection();
	  
	  Vec3f reflectionDirectionVector = calcReflectionDirectionVector(newBallVelocityVectorMeterPerSeconds, racketPlaneNormal); 

	  // TODO velocity + oder *
	  // ca 81km/h = 81000m/h = 22.5 m/s = 2250 cm/s
	  // test war max 7-10 racketVector.length()
	  //racketVector = Vec3f(0,0,5);
	  // TODO testing

	  // TODO
	  
	  //newBallVelocityVectorMeterPerSeconds = reflectionDirectionVector * (newBallVelocityVectorMeterPerSeconds + racketVectorDistanceMeter / t );
	  newBallVelocityVectorMeterPerSeconds = reflectionDirectionVector * newBallVelocityVectorMeterPerSeconds.length() ;
		  //(newBallVelocityVectorMeterPerSeconds.length() + racketVectorDistanceMeter * frameTimeDeltaSeconds * 100);

	  // todo test
	  newballPositionPointMeter = ball.ballPositionPointMeter + newBallVelocityVectorMeterPerSeconds * t;
	  //newballPositionPointMeter = newballPositionPointMeter + ballRadiusMeter * racketPlaneNormal;

	  std::cout << "Hit racket"<< "\n";
	  std::cout << "Racket racketVectorCM " << racketVectorCM << " racketVectorCMLength: " << racketVectorCM.length() << "\n";
	  std::cout << "Hit point: "<< iAct->getHitPoint() << "HitNormFromAction: "<< iAct->getHitNormal() << " racketPlaneNormal: " << racketPlaneNormal << " reflectionDirectionVector: " << reflectionDirectionVector << "\n";
	  std::cout << "Ball pos: " << ball.ballPositionPointMeter << " DirectionVector:" << ball.ballPositionPointMeter << " newDirectionVector:" << newBallVelocityVectorMeterPerSeconds << " newDirectionVectorLength: " << newBallVelocityVectorMeterPerSeconds.length() << '\n';    
	  std::cout << "\n";
 
	}
	*/

	
	
	
	// set Ball rotation
	Quaternion rot = ball.CT->getRotation();

	// TODO check factor
	//ball.CT->setRotation( ball.initRotation +
	//	Quaternion(Vec3f(0,0,1),osgDegree2Rad(0) - 35.0f*ball.ballPositionPointMeter.x()) 
	//	* Quaternion(Vec3f(1,0,0),osgDegree2Rad(0) - 35.0f*ball.ballPositionPointMeter.getValues()[2]));

	Vec3f projectedBallVelocity = Vec3f(newBallVelocityVectorMeterPerSeconds.x(),0, newBallVelocityVectorMeterPerSeconds.z());
	ball.CT->setRotation(Quaternion(Vec3f(0,1,0).cross(projectedBallVelocity), projectedBallVelocity.length() * 0.6) * ball.CT->getRotation());

	// set position and direction for next frame
	ball.ballPositionPointMeter = newballPositionPointMeter;
	
	// meter in cm -> *100
	ball.CT->setTranslation(newballPositionPointMeter * 100);

	ball.ballVelocityVectorMeterPerSeconds = newBallVelocityVectorMeterPerSeconds;



}

void displayRacket() {
      racketCTWand->setTranslation(wand_position);
      racketCTWand->setRotation(wand_orientation);

      racketPlaneCT->setRotation(wand_orientation * wandFixRotationQuaternion);
      racketPlaneCT->setTranslation(wand_position);
}

void displayWand2Ball(){
	  Ball& ball = *vectorOfBalls[currentBallindex];
	  ball.ballPositionPointMeter = wand2_position/100;
	  ball.ballVelocityVectorMeterPerSeconds = Vec3f(0.f, 0.f, 0.f);
	  ball.CT->setRotation(wand2_orientation);
	  //racketCTWand->setRotation(wand_orientation);
}

void displayBallRacketRay(){
      //getRackePlaneFaceDirection()
      Ball& ball = *vectorOfBalls[currentBallindex];
      ballRacketRayCT->setTranslation(ball.ballPositionPointMeter* 100);
      ballRacketRayCT->setRotation(wand_orientation * wandFixRotationQuaternion);
      
}

void holdBall(){
	  std::cout << "hold ball" << "\n";
	  addNewBall(wand2_position/100, Vec3f(0,0,0));
	  ballHolding = true;
}

void releaseBall(){
	  std::cout << "release ball" << "\n";
	  ballHolding = false;
	  Ball& ball = *vectorOfBalls[currentBallindex];
	  ball.ballPositionPointMeter = wand2_position / 100;
	  ball.initRotation = wand2_orientation;
	  // 
	  // 
	  ball.ballVelocityVectorMeterPerSeconds = wand2VectorCM/ 100 / frameTimeDeltaSeconds;
}

void addNewBall(Vec3f position, Vec3f direction){
	currentBallindex++;
	if(currentBallindex >= vectorOfBalls.size()){
		currentBallindex = 0;
	}
  
	std::cout << "useOldBall BallPos: " << position << "\t\t BallDir:" << direction << " ballVelocityVectorMeterPerSecondsLength: " << direction.length() << '\n'<<'\n';
	Ball& ball = *vectorOfBalls[currentBallindex];
	ball.ballPositionPointMeter = position;
	ball.ballVelocityVectorMeterPerSeconds = direction;
	ball.ballRayState = 0;
}

void commandDropNewBall(){
	std::cout << "key commandDropNewBall \n";
	addNewBall(wand2_position / 100, Vec3f(0, 0, 0));
}

void commandServeBall(){
	std::cout << "key commandServeBall \n";
	/*
	  Tennisball speed ca 108km/h = 30m/s = 3000cm/s



	*/
	Real32 serveVelocityMeterPerSecond = 17.5;

	//start
	Vec3f ballPositionPointMeter = Vec3f(-4.15f, 1.30f, -24.00f);

	// rand() % 100   -> 0-99
	// ((rand() % 100 - 49) / 10000)   -> -0.0049 bis +0.0049

	// Flugbahn
	Vec3f ballVelocityVectorMeterPerSeconds;
	Real32 x = 0.175f + ((Real32)(rand() % 80 - 45) / 1000);
	Real32 y = 0.175f + ((Real32)(rand() % 50 - 25) / 1000);
	Real32 z = 1.f;

	// TODO rand off
	x = 0.175f ;
	y = 0.225f ;
	z = 1.0f;

	Vec3f serveDirectionVector = Vec3f(x, y, z) ;
	serveDirectionVector.normalize();
	// m/s in cm/s
	serveDirectionVector = serveDirectionVector * serveVelocityMeterPerSecond ;

	addNewBall(ballPositionPointMeter, serveDirectionVector);
}

void commandServeRandomBall(){
	std::cout << "key commandServeRandomBall \n";
	/*
	  Tennisball speed ca 108km/h = 30m/s = 3000cm/s

	*/
	Real32 serveVelocityMeterPerSecond = 17.5;

	//start
	Vec3f ballPositionPointMeter = Vec3f(-4.15f, 1.30f, -24.00f);

	// rand() % 100   -> 0-99
	// ((rand() % 100 - 49) / 1000)   -> -0.049 bis +0.049

	// Flugbahn
	Vec3f ballVelocityVectorMeterPerSeconds;

	Real32 randXValue = ((Real32)(rand() % 8 - 4) / (Real32)100);
	Real32 randYValue = ((Real32)(rand() % 25) / (Real32)100);

	randYValue = randYValue + randXValue/2;

	std::cout << "randXValue: " << randXValue << " randYValue: " << randYValue << "\n";

	Real32 x = 0.175f + randXValue;
	Real32 y = 0.185f + randYValue;
	Real32 z = 1.f;


	Vec3f serveDirectionVector = Vec3f(x, y, z) ;
	serveDirectionVector.normalize();
	// m/s in cm/s
	serveDirectionVector = serveDirectionVector * serveVelocityMeterPerSecond ;

	addNewBall(ballPositionPointMeter, serveDirectionVector);
}

void commandSortBalls(){
	std::cout << "Sort all Balls. \n";

	Real32 m = 3.f / vectorOfBalls.size();
	for(int i = 0; i< vectorOfBalls.size(); i++){
		addNewBall(Vec3f(-2.00f + 6.00f , 0.f + ballRadiusMeter, -0.50f - m * (i)), Vec3f(0,0,0));
	}
}

bool timeserved = false;
void serveTimer(){
	Real32 time = glutGet(GLUT_ELAPSED_TIME);

	if((int)abs(time / 1000) % serveEverySec == 0){
		if(!timeserved){
			commandServeBall();
		}

		timeserved = true;
	} else{
		timeserved = false;
	}

}


template<typename T>
T scale_tracker2cm(const T& value)
{
	static const float scale = OSGCSM::convert_length(cfg.getUnits(), 1.f, OSGCSM::CAVEConfig::CAVEUnitCentimeters);
	return value * scale;
}

Quaternion head_orientation = Quaternion(Vec3f(0.f, 1.f, 0.f), 3.141f);
Vec3f head_position = Vec3f(0.f, 170.f, 200.f);	// a 1.7m Person 2m in front of the scene

void VRPN_CALLBACK callback_head_tracker(void* userData, const vrpn_TRACKERCB tracker)
{
	head_orientation = Quaternion(tracker.quat[0], tracker.quat[1], tracker.quat[2], tracker.quat[3]);
	head_position = Vec3f(scale_tracker2cm(Vec3d(tracker.pos)));
}


void VRPN_CALLBACK callback_wand_tracker(void* userData, const vrpn_TRACKERCB tracker)
{
	wand_orientation = Quaternion(tracker.quat[0], tracker.quat[1], tracker.quat[2], tracker.quat[3]);
	wand_position = Vec3f(scale_tracker2cm(Vec3d(tracker.pos)));
}

void VRPN_CALLBACK callback_wand_tracker2(void* userData, const vrpn_TRACKERCB tracker)
{
	wand2_orientation = Quaternion(tracker.quat[0], tracker.quat[1], tracker.quat[2], tracker.quat[3]);
	wand2_position = Vec3f(scale_tracker2cm(Vec3d(tracker.pos)));
}

Vec3f analog_values = Vec3f();
void VRPN_CALLBACK callback_analog(void* userData, const vrpn_ANALOGCB analog)
{
	if (analog.num_channel >= 2)
		analog_values = Vec3f(analog.channel[0], 0, -analog.channel[1]);
}

void VRPN_CALLBACK callback_button(void* userData, const vrpn_BUTTONCB button)
{
  //std::cout << "button: " << button.button << " "<< button.state << "\n";
  if (button.button == 0 && button.state == 1){
      // hold ball
      holdBall();
  }
  if (button.button == 0 && button.state == 0){
      // release ball
      releaseBall();
  }
  
  
  if (button.button == 1 && button.state == 1)
		commandSortBalls();
  if (button.button == 2 && button.state == 1)
		commandServeRandomBall();
  if (button.button == 3 && button.state == 1)
		commandServeBall();
	
}

void InitTracker(OSGCSM::CAVEConfig &cfg)
{
	try
	{
		const char* const vrpn_name = "DTrack@localhost";
		tracker = new vrpn_Tracker_Remote(vrpn_name);
		tracker->shutup = true;
		tracker->register_change_handler(NULL, callback_head_tracker, cfg.getSensorIDHead());
		
          //TODO check id
          //tracker->register_change_handler(NULL, callback_wand_tracker, cfg.getSensorIDController());
	  tracker->register_change_handler(NULL, callback_wand_tracker, 3);
	  tracker->register_change_handler(NULL, callback_wand_tracker2, 0);
	  
		button = new vrpn_Button_Remote(vrpn_name);
		button->shutup = true;
		button->register_change_handler(nullptr, callback_button);
		analog = new vrpn_Analog_Remote(vrpn_name);
		analog->shutup = true;
		analog->register_change_handler(NULL, callback_analog);
	}
	catch(const std::exception& e) 
	{
		std::cout << "ERROR: " << e.what() << '\n';
		return;
	}
}

void check_tracker()
{
	tracker->mainloop();
	button->mainloop();
	analog->mainloop();
}

void print_tracker()
{
	std::cout << "Head position: " << head_position << " orientation: " << head_orientation << '\n';
	std::cout << "Wand position: " << wand_position << " orientation: " << wand_orientation << '\n';
	std::cout << "Analog: " << analog_values << '\n';
}

void keyboard(unsigned char k, int x, int y)
{
	Real32 ed;
	switch(k)
	{
		case 'q':
		case 27: 
			cleanup();
			exit(EXIT_SUCCESS);
			break;
		case 'e':
			ed = mgr->getEyeSeparation() * .9f;
			std::cout << "Eye distance: " << ed << '\n';
			mgr->setEyeSeparation(ed);
			break;
		case 'E':
			ed = mgr->getEyeSeparation() * 1.1f;
			std::cout << "Eye distance: " << ed << '\n';
			mgr->setEyeSeparation(ed);
			break;
		case 'h':
			cfg.setFollowHead(!cfg.getFollowHead());
			std::cout << "following head: " << std::boolalpha << cfg.getFollowHead() << '\n';
			break;
		case 'i':
			print_tracker();
			break;
		case 't':
			if(vectorOfBalls.size() >0){
				std::cout << "Last Ball, BallPos: " << vectorOfBalls[currentBallindex]->ballPositionPointMeter << "\t\t BallDir:" << vectorOfBalls[currentBallindex]->ballVelocityVectorMeterPerSeconds << " ballVelocityVectorMeterPerSecondsLength: " << vectorOfBalls[currentBallindex]->ballVelocityVectorMeterPerSeconds.length() << " RackePlaneFaceDirection: " << getRackePlaneFaceDirection() << '\n';
			}
			break;
		case 'g':
			commandSortBalls();
			break;
		case 'r':
			commandDropNewBall();
			break;
		case 'f':
			commandServeBall();
			break;
		case '4':
			commandServeRandomBall();
			break;
		// keys
		case 'w':
			wand_position = Vec3f(wand_position.getValues()[0], wand_position.getValues()[1]+10, wand_position.getValues()[2]);
			break;
		case 'a':
			wand_position = Vec3f(wand_position.getValues()[0]-10, wand_position.getValues()[1], wand_position.getValues()[2]);
			break;
		case 's':
			wand_position = Vec3f(wand_position.getValues()[0], wand_position.getValues()[1]-10, wand_position.getValues()[2]);
			break;
		case 'd':
			wand_position = Vec3f(wand_position.getValues()[0]+10, wand_position.getValues()[1], wand_position.getValues()[2]);
			break;
		case 'v':
			wand_orientation =  Quaternion(Vec3f(0.f,1.f,0.f),osgDegree2Rad(45.f)) * wand_orientation;
			break;
		case 'b':
			wand_orientation =  Quaternion(Vec3f(0.f,0.f,1.f),osgDegree2Rad(45.f)) * wand_orientation;
			break;
		case 'n':
			wand_orientation = Quaternion(Vec3f(1.f,0.f,0.f),osgDegree2Rad(0.f)) * Quaternion(Vec3f(0.f,1.f,0.f),osgDegree2Rad(90.f)) ;
			break;
		case 'm':
			wand_orientation = Quaternion(Vec3f(0.f,0.f,1.f),osgDegree2Rad(60.f-5.f)) * Quaternion(Vec3f(0.f,1.f,0.f),osgDegree2Rad(90.f)) ;
			break;

		default:
			std::cout << "Key '" << k << "' ignored\n";
	}
}

void setupGLUT(int *argc, char *argv[])
{
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGB  |GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow("OpenSG CSMDemo with VRPN API");
	glutDisplayFunc([]()
	{
		// black navigation window
		glClear(GL_COLOR_BUFFER_BIT);
		glutSwapBuffers();
	});
	glutReshapeFunc([](int w, int h)
	{
		mgr->resize(w, h);
		glutPostRedisplay();
	});
	glutKeyboardFunc(keyboard);
	glutIdleFunc([]()
	{
		check_tracker(); // todo war erstes!!
		const auto speed = 1.f;
		mgr->setUserTransform(head_position, head_orientation);
		mgr->setTranslation(mgr->getTranslation() + speed * analog_values);
		
		mgr->setHeadlight(false);
		
		calculateFrameDelay();

		calcualteWandVectors();
		
		displayRacket();
		
		for(int i = 0; i < vectorOfBalls.size(); i++){
			displayTennisBall(*vectorOfBalls[i]);
		}

		if(ballHolding){
		  displayWand2Ball();
		}

		//displayBallRacketRay();
		
		if(autoServe){
		  serveTimer();
		}

		

		commitChanges(); //make sure the changes are distributed over the containers
		mgr->redraw(); // redraw the window
		
		// the changelist should be cleared - else things could be copied multiple times
		OSG::Thread::getCurrentChangeList()->clear();
	});
}

int main(int argc, char **argv)
{
#if WIN32
	OSG::preloadSharedObject("OSGFileIO");
	OSG::preloadSharedObject("OSGImageFileIO");
#endif
	try
	{
		bool cfgIsSet = false;
		NodeRefPtr scene = nullptr;

		// ChangeList needs to be set for OpenSG 1.4
		ChangeList::setReadWriteDefault();
		osgInit(argc,argv);

		// evaluate intial params
		for(int a=1 ; a<argc ; ++a)
		{
			if( argv[a][0] == '-' )
			{
				if ( strcmp(argv[a],"-f") == 0 ) 
				{
					char* cfgFile = argv[a][2] ? &argv[a][2] : &argv[++a][0];
					if (!cfg.loadFile(cfgFile)) 
					{
						std::cout << "ERROR: could not load config file '" << cfgFile << "'\n";
						return EXIT_FAILURE;
					}
					cfgIsSet = true;
				}
			} else {
				std::cout << "Loading scene file '" << argv[a] << "'\n";
				scene = SceneFileHandler::the()->read(argv[a], NULL);
			}
		}

		// load the CAVE setup config file if it was not loaded already:
		if (!cfgIsSet) 
		{
			const char* const default_config_filename = "config/mono.csm";
			if (!cfg.loadFile(default_config_filename)) 
			{
				std::cout << "ERROR: could not load default config file '" << default_config_filename << "'\n";
				return EXIT_FAILURE;
			}
		}

		cfg.printConfig();

		// start servers for video rendering
		if ( startServers(cfg) < 0 ) 
		{
			std::cout << "ERROR: Failed to start servers\n";
			return EXIT_FAILURE;
		}

		setupGLUT(&argc, argv);

		InitTracker(cfg);

		MultiDisplayWindowRefPtr mwin = createAppWindow(cfg, cfg.getBroadcastaddress());

		if (!scene) 
			scene = buildScene();
		commitChanges();

		mgr = new OSGCSM::CAVESceneManager(&cfg);
		mgr->setWindow(mwin );
		mgr->setRoot(scene);
		mgr->showAll();
		
		
		mgr->getWindow()->init();
		mgr->turnWandOff();
	}
	catch(const std::exception& e)
	{
		std::cout << "ERROR: " << e.what() << '\n';
		return EXIT_FAILURE;
	}

	glutMainLoop();
}
