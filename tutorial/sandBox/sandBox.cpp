#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>
#include <igl/dqs.h>
#include <igl/forward_kinematics.h>
#include <igl/opengl/Movable.h>
#include <igl/directed_edge_parents.h>
#include <math.h>
#include <stb/stb_image.h>
#include <igl/boundary_loop.h>
#include <igl/map_vertices_to_circle.h>
#include <igl/harmonic.h>

using namespace std;

const int MODE_DEAD = 0;
const int MODE_NORMAL = 1;
const int MODE_SUPERSPEED = 2;
const int MODE_BOMB = 3;
const int MODE_SNAIL = 4;

// Settings
double scale_num = 4;
double snake_length = 1.6;
double num_of_joints = 16;
double tail = (-1) * (snake_length / 2);
double head = snake_length / 2;
double dist_joint = snake_length / num_of_joints;
double moveBy = 0.025 * scale_num;
const double rotateBy = 0.1;
const double cameraDistance = 10.0*scale_num;

double delay = 4;
double frameNum = 0;

float lastXs = 0;
float lastYs = 0;
float lastZs = 0;

const double gravity = 0.01;
const int bombsPoolSize = 10;
int bombsCountdown = 0;

double superSpeedTimer = 0;

GLfloat fogColor[4] = { 0.5f, 0.5f , 0.5f , 1.0f };

Eigen::Matrix3d currentRot = Eigen::Matrix3d::Identity();

string snakeName = "C:/Dev/EngineForAnimationCourse/tutorial/data/snake2.obj";

Eigen::Vector3d toCenter(Movable transd) {
	Eigen::Vector4d notSure = Eigen::Vector4d(0, 0, 0, 1);
	return (transd.MakeTransd().cast<double>() * notSure).block<3, 1>(0, 0);
}


SandBox::SandBox(Display* disp, Renderer* rend)
{
	this->display = disp;
	this->rndr = rend;
	firstPersonView = true;
	changeView();
	score = 0;
	maxScore = 0;
	numOfNormals = 3;
	maxScore = 0;
	firstTime = true;
	SoundEngine = irrklang::createIrrKlangDevice();
	this->muted = false;
	notPlayed = false;
	this->lost = false;
	this->paused = false;
	this->initBM = false;
	this->debug = false;
	this->numOfBombs = 0;
	this->bombsToLaunch = 0;
	this->hard = false;
	this->timer = 0;
	this->endTime = 0;
	this->countFrames = 0;
	this->level = 0;
}
void SandBox::moveBombs() {
	int floor = 12;
	for (int i = 40; i < 40 + bombsPoolSize; i++)
	{
		Eigen::Vector3d currentPosition = toCenter(data(i));
		if (currentPosition.x() < floor || abs(bombsSpeeds[i - 40]) > 0.05) {
			data(i).MyTranslate(Eigen::Vector3d(bombsSpeeds[i - 40], 0, 0), true);
			bombsSpeeds[i - 40] += gravity;

			Eigen::Vector3d newPosition = toCenter(data(i));
			if (newPosition.x() > floor) {
				bombsSpeeds[i - 40] /= 1.6;
				bombsSpeeds[i - 40] = abs(bombsSpeeds[i - 40]) * -1;
			}
		}
	}
}

bool SandBox::getHard() {
	return hard;
}

void SandBox::moveBalls() {
	for (int i = 0; i < 40; i++)
	{
		double ballSpeed = ballsSpeeds[i];
		data(i).MyTranslate(Eigen::Vector3d(0, 0, ballSpeed), false);
		ballsDistances[i] += ballSpeed;
		if (abs(ballsDistances[i]) > ballsMaxDistances[i]) {
			ballsSpeeds[i] = -1 * ballsSpeeds[i];
		}
		if (i >= 20 && ballMode[i] != MODE_DEAD) {
			data(i).init_data_structures();
		}
	}
}

bool SandBox::getFirstTime() {
	return firstTime;
}

void SandBox::changeView() {
	firstPersonView = !firstPersonView;
	if (!firstPersonView) {
		rndr->core().camera_eye = Eigen::Vector3f(0.00001, cameraDistance, 0);
		rndr->core().camera_center = Eigen::Vector3f(0, 0, 0);
		rndr->core().camera_up = Eigen::Vector3f(0, 1, 0);
		//rndr->core().camera_translation = Eigen::Vector3f(0, 0, 0);
		display->moveFPVdisplay(-90, 0, false);
	}
}


void SandBox::playMusic() {
	if (!initBM) {
		backgroundMusic = SoundEngine->play2D("C:/Dev/EngineForAnimationCourse/external/irrKlang/media/breakout.mp3", true, false, true);
		initBM = true;
	}
	else {
		backgroundMusic->setPlayPosition(0);
		backgroundMusic->setIsPaused(false);
	}
}

void SandBox::pauseGame() {
	paused = !paused;
	if (initBM) {
		backgroundMusic->setIsPaused(paused);
	}
}

Eigen::Vector3d SandBox::getHeadPos() {
	return toCenter(joints[0]);

}
Eigen::Vector3d SandBox::getHeadPosFixed() {
	Eigen::Vector3d headPos = getHeadPos();
	return Eigen::Vector3d(headPos.x(), headPos.y(), headPos.z() * scale_num);

}

void SandBox::paintWeights() {
	W.resize(snake->V.rows(), joints.size());

	for (int i = 0; i < snake->V.rows(); i++)
	{
		Eigen::Vector3d currentVertex = snake->V.row(i);
		vector<double> weights;
		for (int j = 0; j < joints.size(); j++)
		{
			double vertex = currentVertex.z();
			double joint = toCenter(joints.at(j)).z();
			double weight;
			if ((j + 1) == joints.size()) {
				weight = (vertex <= joint) ? 1.0 : 0.0;
			}
			else {
				double nextJoint = toCenter(joints.at(j + 1)).z();
				if (j == 0 && vertex >= joint) {
					weight = 1;
				}
				else if (vertex <= joint && vertex > nextJoint) {
					weight = (vertex - nextJoint) / (joint - nextJoint);
					weights.push_back(weight);
					weight = (joint - vertex) / (joint - nextJoint);
					j++;
				}
				else {
					weight = 0;
				}
			}
			weights.push_back(weight);
		}
		Eigen::Matrix<double, 16, 1> weightsVector;
		weightsVector <<
			weights[0], weights[1], weights[2], weights[3],
			weights[4], weights[5], weights[6], weights[7],
			weights[8], weights[9], weights[10], weights[11],
			weights[12], weights[13], weights[14], weights[15];
		W.row(i) = weightsVector;
	}
}

void SandBox::setupJoints() {
	for (size_t i = 0; i < num_of_joints; i++) {
		Movable current_joint;
		current_joint.MyTranslate(Eigen::Vector3d(0, 0, head - (dist_joint * i)), true);
		joints.push_back(current_joint);
	}

	BE.resize(joints.size() - 1, 2);
	for (size_t i = 0; i < num_of_joints - 1; i++)
	{
		BE.row(i) = Eigen::Vector2i(i, i + 1);
	}
	igl::directed_edge_parents(BE, P);

	for (size_t i = 0; i < num_of_joints; i++)
	{
		rotationQueues.push_back(queue<Eigen::Matrix3d>());
		translationQueues.push_back(queue<Eigen::Vector3d>());

		if (delay > 0 && i > 0) {
			for (int j = 0; j < delay; j++)
			{
				translationQueues[i].push(Eigen::Vector3d(0, 0, dist_joint / delay));
			}
		}
	}
}

Eigen::MatrixXd SandBox::calculateJointsMatrix() {
	Eigen::MatrixXd C;
	C.resize(joints.size(), 3);
	for (int i = 0; i < joints.size(); i++)
	{
		C.row(i) = toCenter(joints[i]);
	}
	return C;
}


bool SandBox::checkV(Eigen::Vector3d vRow) {
	return (toCenter(joints[7])).y() > vRow.y();
}

bool SandBox::checkF(Eigen::Vector3i fRow) {
	bool ret = false;
	for (int i = 0; i < 3; i++) {
		int vertex = fRow[i];
		if (checkV(snake->V.row(vertex))) {
			ret = true;
		}
	}
	return ret;
}

double SandBox::linearToCenter(Eigen::Vector3d vRow) {
	double dist = abs((toCenter(joints[7])).y() - vRow.y());
	if (dist == 0) {
		//cout << "dist: " << dist << endl;
		return 1;
	}
	else {
		return 1;
	}
	//return dist*2;
	//return 1;
}

void SandBox::linearTexture(Eigen::MatrixXd* V_uv, Eigen::MatrixXd V) {
	int rows = V_uv->rows();
	for (int i = 0; i < rows; i++) {
		Eigen::Vector3d atRow = V.row(i);
		double dist = linearToCenter(atRow);
		V_uv->row(i) *= dist;
	}
}

void SandBox::setupTexture() {
	Eigen::MatrixXd V_uv;

	Eigen::MatrixXd V;
	Eigen::MatrixXi F;

	int numOfV = snake->V.rows();
	int numOfF = snake->F.rows();

	V.resize(numOfV, snake->V.cols());
	F.resize(numOfF, snake->F.cols());


	for (int i = 0; i < numOfV; i++) {
		if (checkV(snake->V.row(i))) {
			//cout << "yello" << endl;
			V.row(i) = Eigen::Vector3d(0, 0, 0);
		}
		else{
			V.row(i) = snake->V.row(i);
		}
		//F.row(i) = snake->F.row(i);
	}

	for (int j = 0; j < numOfF; j++) {
		if (checkF(snake->F.row(j))) {
			//cout << "yello" << endl;
			F.row(j) = Eigen::Vector3i(0, 0, 0);
		}
		else {
			F.row(j) = snake->F.row(j);
		}
		//F.row(i) = snake->F.row(i);
	}

	Eigen::VectorXi bnd;
	igl::boundary_loop(F, bnd);

	Eigen::MatrixXd bnd_uv;
	igl::map_vertices_to_circle(V, bnd, bnd_uv);

	igl::harmonic(snake->V, snake->F, bnd, bnd_uv, 1, V_uv);

	V_uv *= 0.5;
	//linearTexture(&V_uv, V);

	snake->set_uv(V_uv);

	snake->image_texture("C:/Dev/EngineForAnimationCourse/tutorial/textures/snake1.png");
	snake->show_texture = true;
}

void SandBox::debugSnake() {
	Eigen::RowVector3d tail = toCenter(joints.at(joints.size() - 1)).transpose();
	Eigen::RowVector3d head = toCenter(joints[0]).transpose();

	Eigen::MatrixXd debugPoints;
	debugPoints.resize(joints.size(), 3);

	for (int i = 0; i < joints.size(); i++)
	{
		Movable current_joint = joints.at(i);
		Eigen::Vector3d center = toCenter(current_joint);
		debugPoints.row(i) = center.transpose();
		snake->add_points(center.transpose(), Eigen::RowVector3d(0, 0, 1));
	}

	snake->set_points(debugPoints, Eigen::RowVector3d(0, 0, 1));

	// Debuging important points
	snake->add_points(tail, Eigen::RowVector3d(0, 1, 0));
	snake->add_points(head, Eigen::RowVector3d(0.8, 0.4, 0.4));
	snake->add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(1, 1, 1));
}

void SandBox::setupSnake() {

	snake = &data();

	setupJoints();

	setupTexture();

	paintWeights();
	resetTranspose();
	resetRotation();
	debugSnake();

	// Scaling the snake
	Eigen::Vector3d scale = Eigen::Vector3d(1, 1, scale_num);
	snake->MyScale(scale);
	snake->show_overlay = true;
}

void SandBox::Init(const std::string& config)
{
	setupObjects();

	std::string item_name;
	std::ifstream nameFileout;
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file " << config << std::endl;
	}
	else
	{

		while (nameFileout >> item_name)
		{
			std::cout << "openning " << item_name << std::endl;
			load_mesh_from_file(item_name);

			parents.push_back(-1);
			//data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(1, 1, 1));
			data().show_overlay_depth = false;
			data().point_size = 10;
			data().line_width = 2;
			data().set_visible(false, 1);

			if (item_name == snakeName) {
				setupSnake();
			}

			//data().SetCenterOfRotation(Eigen::Vector3d(10,0, 0));

		}
		nameFileout.close();
	}
	//MyTranslate(Eigen::Vector3d(0, 0, -1), true);


	
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));

	if (snake == 0) {
		throw new exception("No snake has been loaded");
	}


	for (int i = 0; i < data_list.size(); i++) {
		data_list[i].init_data_structures();
	}
}

bool SandBox::ObjectIntersectsSnake(Eigen::Vector3d headPos, Eigen::Vector3d objectPos,int i) {
	return ((objectPos - headPos).norm() < data(i).getDiameter() / 2 + 0.1);
}

void SandBox::launchBombs() {
	bombsCountdown = (numOfBombs * 10) + 1;
	bombsToLaunch = numOfBombs;
}

void SandBox::lostGame() {
	backgroundMusic->setIsPaused(true);
	SoundEngine->play2D("C:/Dev/EngineForAnimationCourse/external/irrKlang/media/GameOver.wav");
	lost = true;

}

void SandBox::launchBomb() {
	int index = nextBomb + 40;
	ballMode[index] = MODE_BOMB;
	Eigen::Vector3d resetVector = -1 * toCenter(data(index));
	data(index).MyTranslate(resetVector, true);
	double randomHorizontalPos = ((rand() % 100 - 50) * 1.0) / (-1 * 4);
	data(index).MyTranslate(Eigen::Vector3d(-13, 0, randomHorizontalPos), true);
	bombsSpeeds[nextBomb] = 0;
	nextBomb = nextBomb == bombsPoolSize ? 0 : nextBomb + 1;
}

int SandBox::getCDT() {
	int remTime = endTime - timer;
	if (remTime == 0) {
		lostGame();
	}
	return endTime - timer;
}

void SandBox::setupLevel() {
	int j = 1;
	srand(time(0));

	nextBomb = 0;

	Eigen::Vector3d headPos = getHeadPosFixed();
	int num1 = 6 * scale_num;
	int num2 = 3 * scale_num;
	for (int i = 0; i < 40 + bombsPoolSize; i++) {
		if (ballMode[i] != MODE_DEAD) {
			data(i).MyTranslate(Eigen::Vector3d(100, 100, 100), true);
		}
	}
	int numOfSpecial = numOfNormals / 3;
	numOfBombs = numOfSpecial * 3;
	int numOfTotalSpheres = numOfNormals + numOfSpecial*3;
	for (int i = 0; i < numOfTotalSpheres; i++) {
		int objectIndex = i < numOfNormals ? i : i + 20;
		double randX = (rand() % num1) - num2;
		double randY = 0;
		double randZ = (rand() % num1) - num2;
		while (ObjectIntersectsSnake(headPos, Eigen::Vector3d(randX, randY, randZ), objectIndex)) {
			randX = (rand() % num1) - num2;
			randY = 0;
			randZ = (rand() % num1) - num2;
		}
		Eigen::Vector3d resetVector = -1 * toCenter(data_list[objectIndex]);
		data_list[objectIndex].MyTranslate(resetVector, true);
		data_list[objectIndex].MyTranslate(Eigen::Vector3d(randX, randY, randZ), true);

		if (i < numOfNormals) {
			ballMode[objectIndex] = MODE_NORMAL;
			data_list[objectIndex].set_colors(Eigen::RowVector3d(0, 1, 0));
			maxScore++;
		}
		else {
			if (j <= numOfSpecial) {
				ballMode[objectIndex] = MODE_SUPERSPEED;
				data_list[objectIndex].set_colors(Eigen::RowVector3d(0, 0, 1));
			}
			else if (j <= numOfSpecial * 2) {
				ballMode[objectIndex] = MODE_BOMB;
				data_list[objectIndex].set_colors(Eigen::RowVector3d(1, 0, 0));
			}
			else {
				ballMode[objectIndex] = MODE_SNAIL;
				data_list[objectIndex].set_colors(Eigen::RowVector3d(1, 1, 0));
			}
			j++;
			
		}
	}

	if (!getFirstTime()) {
		level++;
	}

	if (hard) {
		timer = std::time(0);
		endTime = timer + (20 + level*10);
	}

	firstTime = false;
	numOfNormals += 1;
	notPlayed = true;


}

void SandBox::setupObjects() {
	const double defaultSpeed = 0.02;
	const double defaultMaxDistance = 1;

	srand(time(0));
	
	string spherePath = "C:/Dev/EngineForAnimationCourse/tutorial/data/sphere.obj";
	string cubePath = "C:/Dev/EngineForAnimationCourse/tutorial/data/cube.obj";
	string snailPath = "C:/Dev/EngineForAnimationCourse/tutorial/data/snail.obj";

	//int num1 = 6 * scale_num;
	//int num2 = 3 * scale_num;
	for (int i = 0; i < bombsPoolSize; i++)
	{
		bombsSpeeds.push_back(0);
	}
	for (int i = 0; i < 40 + bombsPoolSize; i++) {

		ballsDistances.push_back(0);
		double randomSpeed = defaultSpeed + double(rand() % 6 - 3) / 100;
		ballsSpeeds.push_back(randomSpeed);
		ballsMaxDistances.push_back(defaultMaxDistance + double(rand() % 10 - 5) / 10);
		ballMode.push_back(MODE_DEAD);

		//double randX = (rand() % num1) - num2;
		//double randY = 0;
		//double randZ = (rand() % num1) - num2;
		if (i < 20) {
			load_mesh_from_file(spherePath);
		}
		else if(i < 40) {
			load_mesh_from_file(cubePath);
		}
		else if (i < 40 + bombsPoolSize) {
			load_mesh_from_file(spherePath);
			data().set_colors(Eigen::RowVector3d(1, 1, 1));
		}

		parents.push_back(-1);
		data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(1, 1, 1));
		data().show_overlay_depth = false;
		data().point_size = 10;
		data().line_width = 2;
		data().set_visible(false, 1);
		data().MyTranslate(Eigen::Vector3d(100, 100, 100), true);


		double rotation = rand() % 360;
		Eigen::Matrix3d yRot;
		yRot <<
			cos(rotation), 0, sin(rotation),
			0, 1, 0,
			-1 * sin(rotation), 0, cos(rotation);
		data().MyRotate(yRot);
		//data().MyScale()
	}

	
}

void SandBox::setIKOverlay() {
	Eigen::MatrixXd P;
	P.resize(4, 3);
	P.row(0) = Eigen::Vector3d(0, 0, 0);

	P.row(1) = Eigen::Vector3d(1, 0, 0);
	P.row(2) = Eigen::Vector3d(0, 1, 0);
	P.row(3) = Eigen::Vector3d(0, 0, 1);

	Eigen::MatrixXi E;
	E.resize(3, 2);
	E.row(0) = Eigen::Vector2i(0, 1);
	E.row(1) = Eigen::Vector2i(0, 2);
	E.row(2) = Eigen::Vector2i(0, 3);

	Eigen::MatrixXd colors;
	colors.resize(3, 3);
	colors.row(0) = Eigen::Vector3d(0.5411, 0.3529, 0.9015);
	colors.row(1) = Eigen::Vector3d(0, 0, 0);
	colors.row(2) = Eigen::Vector3d(0.5411, 0.3529, 0.9015);

	snake->set_edges(P, E, colors);
	snake->toggle_box();
}

SandBox::~SandBox()
{

}

void SandBox::resetTranspose() {
	vT.clear();
	for (int i = 0; i < joints.size(); i++)
	{
		vT.push_back(Eigen::Vector3d::Zero());
	}
}
void SandBox::resetRotation() {
	vQ.clear();
	for (int i = 0; i < joints.size(); i++)
	{
		bool funny = false;
		if (funny) {
			vQ.push_back(Eigen::Quaterniond(Eigen::Matrix3d::Zero()));
		}
		else {
			vQ.push_back(Eigen::Quaterniond(Eigen::Matrix3d::Identity()));
		}
	}
}

void SandBox::skin() {
	Eigen::MatrixXd U1;
	Eigen::MatrixXd U2;
	Eigen::MatrixXd U3;

	std::vector<Eigen::Vector3d> resetvT;
	RotationList identityvQ;
	std::vector<Eigen::Vector3d> identityvT;
	std::vector<Eigen::Vector3d> absolutevT;
	std::vector<Eigen::Vector3d> absolutevT2;

	for (int i = 0; i < joints.size(); i++)
	{
		Eigen::Vector3d jointPosition = toCenter(joints[i]);
		Eigen::Vector3d resetVector = -1 * jointPosition;

		resetvT.push_back(resetVector);
		identityvQ.push_back(Eigen::Quaterniond(Eigen::Matrix3d::Identity()));
		identityvT.push_back(Eigen::Vector3d::Zero());
		absolutevT.push_back(vT[i] + jointPosition);

		Eigen::Vector3d newPosition = vQ[i].toRotationMatrix() * jointPosition;
		absolutevT2.push_back(newPosition - jointPosition);
	}

	//if (debug) {
	//	cout << "debug" << endl;
	//}

	//igl::dqs(snake->V, W, identityvQ, resetvT, U1);
	//igl::dqs(U1, W, vQ, identityvT, U2);
	//igl::dqs(U2, W, identityvQ, absolutevT, U3);

	//igl::dqs(snake->V, W, vQ, identityvT, U1);
	//igl::dqs(U1, W, identityvQ, absolutevT2, U2);


	igl::dqs(snake->V, W, identityvQ, vT, U1);


	snake->set_vertices(U1);
	resetTranspose();
	resetRotation();
}

void SandBox::move(double moveBy, Eigen::Matrix3d rotMatrix, bool LR) {
	if (frameNum < delay) {
		moveBy = dist_joint / delay;
		frameNum++;
	}

	Eigen::Matrix3d newRotation = joints[0].GetRotation();
	Eigen::Vector3d rotationVector = newRotation * Eigen::Vector3d(0,0,1);


	Eigen::Vector3d yTranslate = newRotation * Eigen::Vector3d(0, 0, moveBy);

	Eigen::Vector3d yTranslateFixedScale = Eigen::Vector3d(yTranslate.x(), yTranslate.y(), yTranslate.z() / scale_num);

	translationQueues[0].push(yTranslateFixedScale);
	rotationQueues[0].push(rotMatrix);
	for (size_t i = 0; i < num_of_joints; i++)
	{
		Eigen::Vector3d oldTranslation = translationQueues[i].front();
		Eigen::Matrix3d oldRotation = rotationQueues[i].front();

		translationQueues[i].pop();
		rotationQueues[i].pop();

		joints[i].MyTranslate(oldTranslation, true);
		joints[i].MyRotate(oldRotation);
		vT[i] = oldTranslation;
		vQ[i] = oldRotation;


		if (i + 1 < num_of_joints) {
			translationQueues[i + 1].push(oldTranslation);
			rotationQueues[i + 1].push(oldRotation);
		}
	}
}

void SandBox::moveLeft() {
	Eigen::Matrix3d upRot;
	upRot <<
		cos(rotateBy), -1 * sin(rotateBy), 0,
		sin(rotateBy), cos(rotateBy), 0,
		0, 0, 1;
	Eigen::Matrix3d yRot;
	yRot <<
		cos(rotateBy), 0, sin(rotateBy),
		0, 1, 0,
		-1 * sin(rotateBy), 0, cos(rotateBy);

	currentRot = yRot;
	//move(moveBy, yRot, true);
}
void SandBox::moveRight() {
	Eigen::Matrix3d yRot;
	yRot <<
		cos(-1 * rotateBy), 0, sin(-1 * rotateBy),
		0, 1, 0,
		-1 * sin(-1 * rotateBy), 0, cos(-1 * rotateBy);

	currentRot = yRot;

	//move(moveBy, yRot, true);
}
void SandBox::moveDown() {
	Eigen::Matrix3d xRot;
	xRot <<
		1, 0, 0,
		0, cos(rotateBy), -1 * sin(rotateBy),
		0, sin(rotateBy), cos(rotateBy);

	currentRot = xRot;

	//move(moveBy, xRot, false);
}
void SandBox::moveUp() {
	Eigen::Matrix3d xRot;
	xRot <<
		1, 0, 0,
		0, cos(-1 * rotateBy), -1 * sin(-1 * rotateBy),
		0, sin(-1 * rotateBy), cos(-1 * rotateBy);

	currentRot = xRot;

	//move(moveBy, xRot, false);
}

int SandBox::getScore() {
	return score;
}

int SandBox::getMaxScore() {
	return maxScore;
}

// ------------------------------------ COLISSION DETECTION ------------------------------------
bool box_intersect(Eigen::AlignedBox<double, 3> box_A, Eigen::AlignedBox<double, 3> box_B, Movable* data_A, Movable* data_B) {
	bool check = false;
	double num1;
	double num2;

	Eigen::Matrix4d transd_A = data_A->MakeTransd().cast<double>();
	Eigen::Matrix4d transd_B = data_B->MakeTransd().cast<double>();


	Eigen::Vector3d extents_a = box_A.sizes() / 2;
	Eigen::Vector4d center_A_temp;
	center_A_temp << box_A.center(), 1;
	Eigen::Vector3d center_A = (transd_A * center_A_temp).block<3, 1>(0, 0);
	Eigen::Matrix3d rotation_A = data_A->GetRotation();

	double a0 = extents_a[0];
	double a1 = extents_a[1];
	double a2 = extents_a[2];

	Eigen::Vector3d extents_b = box_B.sizes() / 2;
	Eigen::Vector4d center_B_temp;
	center_B_temp << box_B.center(), 1;
	Eigen::Vector3d center_B = (transd_B * center_B_temp).block<3, 1>(0, 0);
	Eigen::Matrix3d rotation_B = data_B->GetRotation();

	double b0 = extents_b[0];
	double b1 = extents_b[1];
	double b2 = extents_b[2];

	Eigen::Vector3d D = center_A - center_B;

	Eigen::Vector3d axis_centerA = box_A.corner(box_A.BottomLeftFloor);
	Eigen::Vector3d A0 = (rotation_A * (axis_centerA - box_A.corner(box_A.BottomRightFloor))).normalized();
	Eigen::Vector3d A1 = (rotation_A * (axis_centerA - box_A.corner(box_A.TopLeftFloor))).normalized();
	Eigen::Vector3d A2 = (rotation_A * (axis_centerA - box_A.corner(box_A.BottomLeftCeil))).normalized();
	Eigen::Matrix3d A;
	A.resize(3, 3);
	A << A0, A1, A2;

	Eigen::Vector3d axis_centerB = box_B.corner(box_B.BottomLeftFloor);
	Eigen::Vector3d B0 = (rotation_B * (axis_centerB - box_B.corner(box_A.BottomRightFloor))).normalized();
	Eigen::Vector3d B1 = (rotation_B * (axis_centerB - box_B.corner(box_A.TopLeftFloor))).normalized();
	Eigen::Vector3d B2 = (rotation_B * (axis_centerB - box_B.corner(box_A.BottomLeftCeil))).normalized();
	Eigen::Matrix3d B;
	B.resize(3, 3);
	B << B0, B1, B2;


	Eigen::Matrix3d C = A.transpose() * B;

	//A0, A1, A2
	num1 = a0 + b0 * std::abs(C(0, 0)) + b1 * std::abs(C(0, 1)) + b2 * std::abs(C(0, 2));
	num2 = std::abs((D.transpose() * A0)(0, 0));
	check = num1 < num2;
	if (check) {
		return false;
	}
	num1 = a1 + b0 * std::abs(C(1, 0)) + b1 * std::abs(C(1, 1)) + b2 * std::abs(C(1, 2));
	num2 = std::abs((D.transpose() * A1)(0, 0));
	check = num1 < num2;
	if (check) {
		return false;
	}
	num1 = a2 + b0 * std::abs(C(2, 0)) + b1 * std::abs(C(2, 1)) + b2 * std::abs(C(2, 2));
	num2 = std::abs((D.transpose() * A2)(0, 0));
	check = num1 < num2;
	if (check) {
		return false;
	}

	//B0, B1, B2
	num1 = a0 * std::abs(C(0, 0)) + a1 * std::abs(C(1, 0)) + a2 * std::abs(C(2, 0)) + b0;
	num2 = std::abs((D.transpose() * B0)(0, 0));
	check = num1 < num2;
	if (check) {
		return false;
	}
	check = a0 * std::abs(C(0, 1)) + a1 * std::abs(C(1, 1)) + a2 * std::abs(C(2, 1)) + b1 < std::abs((D.transpose()* B1)(0, 0));
	if (check) {
		return false;
	}
	check = a0 * std::abs(C(0, 2)) + a1 * std::abs(C(1, 2)) + a2 * std::abs(C(2, 2)) + b2 < std::abs((D.transpose()* B2)(0, 0));
	if (check) {
		return false;
	}

	//A0xB0, A0xB1, A0xB2
	check = a1 * std::abs(C(2, 0)) + a2 * std::abs(C(1, 0)) + b1 * std::abs(C(0, 2)) + b2 * std::abs(C(0, 1)) < std::abs((C(1, 0) * D.transpose() * A2)(0, 0) - (C(2, 0) * D.transpose() * A1)(0, 0));
	if (check) {
		return false;
	}
	check = a1 * std::abs(C(2, 1)) + a2 * std::abs(C(1, 1)) + b0 * std::abs(C(0, 2)) + b2 * std::abs(C(0, 0)) < std::abs((C(1, 1) * D.transpose() * A2)(0, 0) - (C(2, 1) * D.transpose() * A1)(0, 0));
	if (check) {
		return false;
	}
	check = a1 * std::abs(C(2, 2)) + a2 * std::abs(C(1, 2)) + b0 * std::abs(C(0, 1)) + b1 * std::abs(C(0, 0)) < std::abs((C(1, 2) * D.transpose() * A2)(0, 0) - (C(2, 2) * D.transpose() * A1)(0, 0));
	if (check) {
		return false;
	}

	//A1xB0, A1xB1, A1xB2
	check = a0 * std::abs(C(2, 0)) + a2 * std::abs(C(0, 0)) + b1 * std::abs(C(1, 2)) + b2 * std::abs(C(1, 1)) < std::abs((C(2, 0) * D.transpose() * A0)(0, 0) - (C(0, 0) * D.transpose() * A2)(0, 0));
	if (check) {
		return false;
	}
	check = a0 * std::abs(C(2, 1)) + a2 * std::abs(C(0, 1)) + b0 * std::abs(C(1, 2)) + b2 * std::abs(C(1, 0)) < std::abs((C(2, 1) * D.transpose() * A0)(0, 0) - (C(0, 1) * D.transpose() * A2)(0, 0));
	if (check) {
		return false;
	}
	check = a0 * std::abs(C(2, 2)) + a2 * std::abs(C(0, 2)) + b0 * std::abs(C(1, 1)) + b1 * std::abs(C(1, 0)) < std::abs((C(2, 2) * D.transpose() * A0)(0, 0) - (C(0, 2) * D.transpose() * A2)(0, 0));
	if (check) {
		return false;
	}

	//A2xB0, A2xB1, A2xB2
	check = a0 * std::abs(C(1, 0)) + a1 * std::abs(C(0, 0)) + b1 * std::abs(C(2, 2)) + b2 * std::abs(C(2, 1)) < std::abs((C(0, 0) * D.transpose() * A1)(0, 0) - (C(1, 0) * D.transpose() * A0)(0, 0));
	if (check) {
		return false;
	}
	check = a0 * std::abs(C(1, 1)) + a1 * std::abs(C(0, 1)) + b0 * std::abs(C(2, 2)) + b2 * std::abs(C(2, 0)) < std::abs((C(0, 1) * D.transpose() * A1)(0, 0) - (C(1, 1) * D.transpose() * A0)(0, 0));
	if (check) {
		return false;
	}
	check = a0 * std::abs(C(1, 2)) + a1 * std::abs(C(0, 2)) + b0 * std::abs(C(2, 1)) + b1 * std::abs(C(2, 0)) < std::abs((C(0, 2) * D.transpose() * A1)(0, 0) - (C(1, 2) * D.transpose() * A0)(0, 0));
	if (check) {
		return false;
	}

	return true;
}


bool insideBox(Eigen::AlignedBox<double, 3> box, Eigen::Vector3d boxCenter, Eigen::Matrix3d boxRotation, Eigen::Vector3d point) {
	Eigen::Vector3d axis_center = box.corner(box.BottomLeftFloor);
	Eigen::Vector3d dx = (boxRotation * (axis_center - box.corner(box.BottomRightFloor))).normalized();
	Eigen::Vector3d dy = (boxRotation * (axis_center - box.corner(box.TopLeftFloor))).normalized();
	Eigen::Vector3d dz = (boxRotation * (axis_center - box.corner(box.BottomLeftCeil))).normalized();

	Eigen::Vector3d half = box.sizes() / 2; // Box size in each dimension, divided by 2.

	Eigen::Vector3d d = point - boxCenter;
	return abs(d.dot(dx)) <= half.x() &&
		abs(d.dot(dy)) <= half.y() &&
		abs(d.dot(dz)) <= half.z();
}
int SandBox::intersectHead() {
	Eigen::Vector3d headPosFixedScale = getHeadPosFixed();
	for (int i = 0; i < data_list.size() - 1; i++)
	{
		if (&data_list[i] != snake) {
			if (ballMode[i] == MODE_NORMAL) {
				double diameter = data(i).getDiameter();
				if ((toCenter(data(i)) - headPosFixedScale).norm() < diameter / 2 + 0.1) {
					return i;
				}
			}
			else if(ballMode[i] != MODE_DEAD) {
				if (insideBox(data(i).tree.m_box, toCenter(data(i)), data(i).GetRotation(), headPosFixedScale)) {
					return i;
				}
				//Eigen::AlignedBox3d headBox = Eigen::AlignedBox3d(headPos, headPos + Eigen::Vector3d(0.1,0.1,0.1));
				//if (box_intersect(headBox, data(i).tree.m_box, &joints[0], &data(i))) {
				//	return i;
				//}
				/*if (data(i).tree.m_box.intersects(headBox)) {
					return i;
				}*/
			}
		}
	}
	return -1;
}

void SandBox::incScore() {
	score++;
}

void SandBox::restart() {
	this->score = 0;
	this->level = 0;
	this->maxScore = 0;
	this->firstTime = true;
	this->numOfNormals = 3;
	this->setupLevel();
	this->lost = false;
	this->playMusic();
}

void SandBox::eatSphere(int dataNum) {
	Eigen::Vector3f headPos = getHeadPosFixed().cast<float>();
	Eigen::Vector3f cameraPos = rndr->core().camera_eye;
	Eigen::Vector3f cameraDir = rndr->core().camera_center;
	irrklang::vec3df listenerPos = irrklang::vec3df(cameraPos.x(), cameraPos.y(), cameraPos.z());
	irrklang::vec3df listenerDir = irrklang::vec3df(cameraDir.x(), cameraDir.y(), cameraDir.z());
	SoundEngine->setListenerPosition(listenerPos, listenerDir);
	irrklang::vec3df soundPos = irrklang::vec3df(headPos.x(), headPos.y(), headPos.z());
	if (ballMode[dataNum] == MODE_SUPERSPEED) {
		//SoundEngine->play2D("C:/Dev/EngineForAnimationCourse/external/irrKlang/media/SuperSpeed.mpeg");
		SoundEngine->play3D("C:/Dev/EngineForAnimationCourse/external/irrKlang/media/SuperSpeed.mpeg", soundPos);

		superSpeedTimer += 50;
	}
	else if (ballMode[dataNum] == MODE_BOMB) {
		lostGame();
		return;
	}
	else if (ballMode[dataNum] == MODE_NORMAL) {
		//SoundEngine->play2D("C:/Dev/EngineForAnimationCourse/external/irrKlang/media/eatSphere.wav");
		SoundEngine->play3D("C:/Dev/EngineForAnimationCourse/external/irrKlang/media/eatSphere.wav", soundPos);
		incScore();
	}
	else if (ballMode[dataNum] == MODE_SNAIL) {
		SoundEngine->play2D("C:/Dev/EngineForAnimationCourse/external/irrKlang/media/incoming.mp3");
		launchBombs();
	}
	data(dataNum).MyTranslate(Eigen::Vector3d(100, 100, 100), false);
	ballMode[dataNum] = MODE_DEAD;

}



void SandBox::moveFPV(){
	const Eigen::Vector3f headPosFixedScale = getHeadPosFixed().cast<float>();
	Eigen::Vector3f rotationVector = (joints[0].GetRotation() * Eigen::Vector3d(0, 0, 1)).cast<float>();
	Eigen::Vector3f upVector = (joints[0].GetRotation() * Eigen::Vector3d(0, 1, 0)).cast<float>();


	Eigen::Vector3f zUnit = Eigen::Vector3f(0, 0, 1);
	Eigen::Vector3f yUnit = Eigen::Vector3f(0, 1, 0);


	Eigen::Vector3f originRot = zUnit;
	Eigen::Vector3f rotationVectorOnPlane = Eigen::Vector3f(rotationVector.x(), 0, rotationVector.z());

	Eigen::Vector3f planeNormal = yUnit;

	float yaw = -1 * (std::atan2(originRot.cross(rotationVectorOnPlane).norm(), originRot.dot(rotationVectorOnPlane)));
	float pitch = std::atan2(planeNormal.cross(rotationVector).norm(), planeNormal.dot(rotationVector));

	yaw = (yaw / M_PI) * 180;
	pitch = 90 - ((pitch / M_PI) * 180);

	if (rotationVectorOnPlane.x() < 0) {
		yaw = -yaw;
	}

	//if (rotationVector.y() < 0) {
	//	pitch = -pitch;
	//}

	//if (pitch < -90 || pitch > 90) {
	//	yaw += 180;
	//}

	//cout << "yaw: " << yaw << "    pitch: " << pitch << endl;

	display->moveFPVdisplay(yaw, pitch, upVector.y() < 0);

	rndr->core().camera_eye = headPosFixedScale;
	rndr->core().camera_center = headPosFixedScale + (joints[0].GetRotation() * Eigen::Vector3d(0, 0, 1)).cast<float>();
	rndr->core().camera_up = upVector;

	//cout << "camera_eye:    " << rndr->core().camera_eye.transpose() << endl;
	//cout << "camera_center:    " << rndr->core().camera_center.transpose() << endl;
	//rndr->core().camera_translation = Eigen::Vector3f(0, 0, 0);

	if (rndr->core().camera_eye != headPosFixedScale) {
		cout << "Titzak Alai" << endl;
	}
}

void SandBox::Animate()
{
	if (!lost && maxScore != score && !paused) {
		moveBalls();
		moveBombs();
		double currentMoveSpeed = superSpeedTimer > 0 ? moveBy * 2 : moveBy;
		superSpeedTimer = max(0.0, superSpeedTimer - 1);
		move(currentMoveSpeed, currentRot, false);
		currentRot = Eigen::Matrix3d::Identity();
		skin();
		debugSnake();
		setIKOverlay();
		int intersectedWith = intersectHead();
		if (intersectedWith != -1) {
			eatSphere(intersectedWith);
		}
		bombsCountdown = max(0, bombsCountdown - 1);
		if (bombsCountdown % 10 == 0 && bombsCountdown != 0 && bombsToLaunch > 0) {
			bombsToLaunch--;
			launchBomb();
		}

	}
	
	if (firstPersonView) {
		moveFPV();
	}
	if (initBM) {
		backgroundMusic->setVolume(muted ? 0 : 10);
	}
	if (score == maxScore && notPlayed) {
		SoundEngine->play2D("C:/Dev/EngineForAnimationCourse/external/irrKlang/media/levelComplete.wav");
		notPlayed = false;
	}

	if (hard && countFrames >= 25) {
		timer = max(0, timer + 1);
		countFrames = 0;
	}
	if (!paused) {
		countFrames++;
	}
}

void SandBox::test() {
	Movable head = joints[0];
	Eigen::Vector3d headPos = toCenter(head);

	const double rotateBy = 0.05;
	Eigen::Matrix4d yRot;
	yRot <<
		cos(rotateBy), 0, sin(rotateBy), 0,
		0, 1, 0, 0,
		-1 * sin(rotateBy), 0, cos(rotateBy), 0,
		0, 0, 0, 1;

	Eigen::Matrix4d translationMatrix;
	translationMatrix <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		headPos.x(), headPos.y(), headPos.z(), 1;

	Eigen::Matrix3d currentRotation = head.GetRotation();
	//Eigen::Matrix3d translationMatrix = joints[0].MakeTransd().block<3, 3>(0,0);
	Eigen::Matrix4d temp = translationMatrix * yRot;
	vQ[0] = Eigen::Quaterniond(temp.block<3, 3>(0, 0).transpose());
}