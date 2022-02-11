#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/opengl/Movable.h"
#include "igl/opengl/ViewerData.h"
#include "igl/aabb.h"
#include <vector>
#include <queue>
#include <igl/opengl/glfw/renderer.h>

typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > RotationList;


class SandBox : public igl::opengl::glfw::Viewer
{
public:
	SandBox(Renderer *rend);
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
	int score;
	Eigen::MatrixXd calculateJointsMatrix();
	void setupSnake();
	void setupObjects();
	void debugSnake();
	void paintWeights();
	void setupJoints();
	void skin();
	void resetTranspose();
	void resetRotation();
	void setIKOverlay();

	void SandBox::test();
	void SandBox::loadCubemap();

	void moveLeft();
	void moveRight();
	void moveDown();
	void moveUp();
	void changeView();
	bool moved = false;
	bool firstPersonView;
	Eigen::Vector3d getHeadPos();

private:
	// Prepare array-based edge data structures and priority queue
	void Animate();
	void move(const double moveBy, Eigen::Matrix3d rotMatrix);
	int SandBox::intersect(igl::opengl::ViewerData* data);
	int SandBox::intersectHead();
	void eatSphere(int i);
	void incScore();
	Renderer *rndr;
};

