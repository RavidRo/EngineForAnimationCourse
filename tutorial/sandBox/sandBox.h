#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/opengl/Movable.h"
#include "igl/opengl/ViewerData.h"
#include "igl/aabb.h"
#include <vector>
#include <queue>
#include <igl/opengl/glfw/renderer.h>
#include <external/irrKlang/include/irrKlang.h>

typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > RotationList;

class SandBox : public igl::opengl::glfw::Viewer
{
public:
	SandBox(Display* disp,Renderer* rend);
	~SandBox();
	void Init(const std::string& config);
	igl::opengl::ViewerData* snake = 0;
	std::vector<Movable> joints;
	Eigen::MatrixXi BE;
	Eigen::VectorXi P;
	Eigen::MatrixXd W;
	std::vector<Eigen::Vector3d> vT;
	RotationList vQ;
	std::vector<std::queue<Eigen::Matrix3d>> rotationQueues;
	std::vector<std::queue<Eigen::Vector3d>> translationQueues;
	irrklang::ISoundEngine* SoundEngine;
	irrklang::ISound* backgroundMusic;
	bool ObjectIntersectsSnake(Eigen::Vector3d headPos, Eigen::Vector3d objectPos,int i);

	std::vector<double> ballsSpeeds;
	std::vector<double> ballsDistances;
	std::vector<double> ballsMaxDistances;
	std::vector<int> ballMode;

	void setupLevel();
	int score;
	int maxScore;
	int numOfNormals;

	Eigen::MatrixXd calculateJointsMatrix();
	void setupSnake();
	void setupObjects();

	void setupTexture();

	bool checkV(Eigen::Vector3d vRow);

	bool checkF(Eigen::Vector3i fRow);

	void linearTexture(Eigen::MatrixXd* V_uv, Eigen::MatrixXd F);

	double linearToCenter(Eigen::Vector3d V_uvRow);

	void debugSnake();
	void paintWeights();
	void setupJoints();
	void skin();
	void resetTranspose();
	void resetRotation();
	void setIKOverlay();

	void moveBalls();

	void moveFPV();

	bool firstTime;

	void SandBox::test();

	void moveLeft();
	void moveRight();
	void moveDown();
	void moveUp();
	void changeView();
	bool moved = false;
	bool firstPersonView;
	Eigen::Vector3d getHeadPos();
	Eigen::Vector3d SandBox::getHeadPosFixed();

	void SandBox::moveBombs();
	std::vector<double> bombsSpeeds;
	void SandBox::launchBomb();
	void SandBox::launchBombs();

	int nextBomb;

	int getScore();
	int getMaxScore();

	bool getFirstTime();
	bool notPlayed;
	void playMusic();
	void restart();

	bool muted;
	bool lost;

	bool paused;
	bool initBM;
	void pauseGame();
	
	bool debug;

	int numOfBombs;
	int bombsToLaunch;


private:
	// Prepare array-based edge data structures and priority queue
	void Animate();
	void move(const double moveBy, Eigen::Matrix3d rotMatrix, bool LR);
	int SandBox::intersect(igl::opengl::ViewerData* data);
	int SandBox::intersectHead();
	void eatSphere(int i);
	void incScore();
	Renderer* rndr;
	Display* display;
};