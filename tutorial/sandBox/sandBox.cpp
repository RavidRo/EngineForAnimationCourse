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
using namespace std;

// Settings
double scale_num = 1;
double snake_length = 1.6;
double num_of_joints = 16;
double tail = (-1) * (snake_length / 2);
double head = snake_length / 2;
double dist_joint = snake_length / num_of_joints;
const double moveBy = 0.05;
const double rotateBy = 0.1;

vector<std::string> faces
{
	"C:/Users/romra/OneDrive/Desktop/faces/Daylight Box_Right.bmp",
	"C:/Users/romra/OneDrive/Desktop/faces/Daylight Box_Left.bmp",
	"C:/Users/romra/OneDrive/Desktop/faces/Daylight Box_Top.bmp",
	"C:/Users/romra/OneDrive/Desktop/faces/Daylight Box_Bottom.bmp",
	"C:/Users/romra/OneDrive/Desktop/faces/Daylight Box_Front.bmp",
	"C:/Users/romra/OneDrive/Desktop/faces/Daylight Box_Back.bmp"
};
string snakeName = "C:/Dev/EngineForAnimationCourse/tutorial/data/snake2.obj";

Eigen::Vector3d toCenter(Movable transd) {
	Eigen::Vector4d notSure = Eigen::Vector4d(0, 0, 0, 1);
	return (transd.MakeTransd().cast<double>() * notSure).block<3, 1>(0, 0);
}


SandBox::SandBox(Renderer *rend)
{
	this->rndr = rend;
	firstPersonView = false;
	score = 0;
	rndr->core().camera_eye = Eigen::Vector3f(0.00001, 5*scale_num, 0);
	rndr->core().camera_center = Eigen::Vector3f(0, 0, 0);
	rndr->core().camera_up = Eigen::Vector3f(0, 1, 0);
}

void SandBox::changeView() {
	firstPersonView = !firstPersonView;
	if (!firstPersonView) {
		rndr->core().camera_eye = Eigen::Vector3f(0.00001, 5*scale_num, 0);
		rndr->core().camera_center = Eigen::Vector3f(0, 0, 0);
		rndr->core().camera_up = Eigen::Vector3f(0, 1, 0);
	}
}

//void SandBox::loadCubemap(){
//	unsigned int textureID;
//	glGenTextures(1, &textureID);
//	glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);
//
//	int width, height, nrChannels;
//	for (unsigned int i = 0; i < faces.size(); i++)
//	{
//		unsigned char* data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
//		if (data)
//		{
//			glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
//				0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data
//			);
//			stbi_image_free(data);
//		}
//		else
//		{
//			std::cout << "Cubemap tex failed to load at path: " << faces[i] << std::endl;
//			stbi_image_free(data);
//		}
//	}
//	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
//	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
//	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
//
//	//return textureID;
//}


Eigen::Vector3d SandBox::getHeadPos(){
	return toCenter(joints[0]);
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
				double nextJoint = toCenter(joints.at(j+1)).z();
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
	for (size_t i = 0; i < num_of_joints-1; i++)
	{
		BE.row(i) = Eigen::Vector2i(i, i + 1);
	}
	igl::directed_edge_parents(BE, P);

	for (size_t i = 0; i < num_of_joints; i++)
	{
		rotationQueues.push_back(queue<Eigen::Matrix3d>());
		translationQueues.push_back(queue<Eigen::Vector3d>());
		translationQueues[i].push(Eigen::Vector3d(0, 0, dist_joint/2));
		translationQueues[i].push(Eigen::Vector3d(0, 0, dist_joint/2));
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


void SandBox::debugSnake() {
	Eigen::RowVector3d tail = toCenter(joints.at(joints.size()-1)).transpose();
	Eigen::RowVector3d head = toCenter(joints[0]).transpose();

	Eigen::MatrixXd debugPoints;
	debugPoints.resize(joints.size(), 3);

	for (int i = 0; i < joints.size(); i++)
	{
		Movable current_joint = joints.at(i);
		Eigen::Vector3d center = toCenter(current_joint);
		debugPoints.row(i) = center.transpose();
		//snake->add_points(center.transpose(), Eigen::RowVector3d(0, 0, 1));
	}

	snake->set_points(debugPoints, Eigen::RowVector3d(0, 0, 1));

	// Debuging important points
	snake->add_points(tail, Eigen::RowVector3d(0, 1, 0));
	snake->add_points(head, Eigen::RowVector3d(0.8, 0.4, 0.4));
	snake->add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(1, 1, 1));
}

void SandBox::setupSnake() {
	std::cout << "setting up snake bruv" << std::endl;

	snake = &data();

	// Scaling the snake
	Eigen::Vector3d scale = Eigen::Vector3d(1, 1, scale_num);
	snake->MyScale(scale);

	setupJoints();
	paintWeights();
	resetTranspose();
	resetRotation();
	debugSnake();
}

void SandBox::Init(const std::string &config)
{
	setupObjects();
	std::string item_name;
	std::ifstream nameFileout;
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file "<<config << std::endl;
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

void SandBox::setupObjects() {
	string spherePath = "C:/Dev/EngineForAnimationCourse/tutorial/data/sphere.obj";
	int numOfSpheres = 3;
	int num1 = 6 * scale_num;
	int num2 = 3 * scale_num;
	srand(time(0));
	for (int i = 0; i < numOfSpheres; i++) {
		double randX = (rand() % num1) - num2;
		double randY = 0;
		double randZ = (rand() % num1) - num2;
		load_mesh_from_file(spherePath);

		parents.push_back(-1);
		data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(1, 1, 1));
		data().show_overlay_depth = false;
		data().point_size = 10;
		data().line_width = 2;
		data().set_visible(false, 1);
		data().MyTranslate(Eigen::Vector3d(randX, randY, randZ),true);

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
	Eigen::MatrixXd U;
	
	igl::dqs(snake->V, W, vQ, vT, U);
	snake->set_vertices(U);
	resetTranspose();
	resetRotation();
	moved = false;
	
}
void SandBox::move(const double moveBy, Eigen::Matrix3d rotMatrix) {
	if (!moved) {
		Eigen::Matrix3d newRotation = joints[0].GetRotation();
		Eigen::Vector3d yTranslate = newRotation * Eigen::Vector3d(0, 0, moveBy);

		translationQueues[0].push(yTranslate);
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

			if (i + 1 < num_of_joints) {
				translationQueues[i + 1].push(oldTranslation);
				rotationQueues[i + 1].push(oldRotation);
			}
		}
		/*headTranslation = yTranslate;*/
		moved = true;
	}
	else {
		// TODO: Fix this
		cout << "MOVED TWICE" << endl;
	}

}

void SandBox::moveLeft() {
	Eigen::Matrix3d yRot;
	yRot <<
		cos(rotateBy), 0, sin(rotateBy),
		0, 1, 0,
		-1 * sin(rotateBy), 0, cos(rotateBy);

	move(moveBy, yRot);
}
void SandBox::moveRight() {
	Eigen::Matrix3d yRot;
	yRot <<
		cos(-1*rotateBy), 0, sin(-1*rotateBy),
		0, 1, 0,
		-1 * sin(-1*rotateBy), 0, cos(-1*rotateBy);

	move(moveBy, yRot);
}
void SandBox::moveDown() {
	Eigen::Matrix3d xRot;
	xRot <<
		1, 0, 0,
		0, cos(rotateBy), -1*sin(rotateBy),
		0, sin(rotateBy), cos(rotateBy);

	move(moveBy, xRot);
}
void SandBox::moveUp() {
	Eigen::Matrix3d xRot;
	xRot <<
		1, 0, 0,
		0, cos(-1*rotateBy), -1 * sin(-1*rotateBy),
		0, sin(-1*rotateBy), cos(-1*rotateBy);

	move(moveBy, xRot);
}

// ------------------------------------ COLISSION DETECTION ------------------------------------
//bool box_intersect(Eigen::AlignedBox<double, 3> box_A, Eigen::AlignedBox<double, 3> box_B, igl::opengl::ViewerData* data_A, igl::opengl::ViewerData* data_B) {
//	bool check = false;
//	double num1;
//	double num2;
//
//	Eigen::Matrix4d transd_A = data_A->MakeTransd().cast<double>();
//	Eigen::Matrix4d transd_B = data_B->MakeTransd().cast<double>();
//
//
//	Eigen::Vector3d extents_a = box_A.sizes() / 2;
//	Eigen::Vector4d center_A_temp;
//	center_A_temp << box_A.center(), 1;
//	Eigen::Vector3d center_A = (transd_A * center_A_temp).block<3, 1>(0, 0);
//	Eigen::Matrix3d rotation_A = data_A->GetRotation();
//
//	double a0 = extents_a[0];
//	double a1 = extents_a[1];
//	double a2 = extents_a[2];
//
//	Eigen::Vector3d extents_b = box_B.sizes() / 2;
//	Eigen::Vector4d center_B_temp;
//	center_B_temp << box_B.center(), 1;
//	Eigen::Vector3d center_B = (transd_B * center_B_temp).block<3, 1>(0, 0);
//	Eigen::Matrix3d rotation_B = data_B->GetRotation();
//
//	double b0 = extents_b[0];
//	double b1 = extents_b[1];
//	double b2 = extents_b[2];
//
//	Eigen::Vector3d D = center_A - center_B;
//
//	Eigen::Vector3d axis_centerA = box_A.corner(box_A.BottomLeftFloor);
//	Eigen::Vector3d A0 = (rotation_A * (axis_centerA - box_A.corner(box_A.BottomRightFloor))).normalized();
//	Eigen::Vector3d A1 = (rotation_A * (axis_centerA - box_A.corner(box_A.TopLeftFloor))).normalized();
//	Eigen::Vector3d A2 = (rotation_A * (axis_centerA - box_A.corner(box_A.BottomLeftCeil))).normalized();
//	Eigen::Matrix3d A;
//	A.resize(3, 3);
//	A << A0, A1, A2;
//
//	Eigen::Vector3d axis_centerB = box_B.corner(box_B.BottomLeftFloor);
//	Eigen::Vector3d B0 = (rotation_B * (axis_centerB - box_B.corner(box_A.BottomRightFloor))).normalized();
//	Eigen::Vector3d B1 = (rotation_B * (axis_centerB - box_B.corner(box_A.TopLeftFloor))).normalized();
//	Eigen::Vector3d B2 = (rotation_B * (axis_centerB - box_B.corner(box_A.BottomLeftCeil))).normalized();
//	Eigen::Matrix3d B;
//	B.resize(3, 3);
//	B << B0, B1, B2;
//
//
//	Eigen::Matrix3d C = A.transpose() * B;
//
//	//A0, A1, A2
//	num1 = a0 + b0 * std::abs(C(0, 0)) + b1 * std::abs(C(0, 1)) + b2 * std::abs(C(0, 2));
//	num2 = std::abs((D.transpose() * A0)(0, 0));
//	check = num1 < num2;
//	if (check) {
//		return false;
//	}
//	num1 = a1 + b0 * std::abs(C(1, 0)) + b1 * std::abs(C(1, 1)) + b2 * std::abs(C(1, 2));
//	num2 = std::abs((D.transpose() * A1)(0, 0));
//	check = num1 < num2;
//	if (check) {
//		return false;
//	}
//	num1 = a2 + b0 * std::abs(C(2, 0)) + b1 * std::abs(C(2, 1)) + b2 * std::abs(C(2, 2));
//	num2 = std::abs((D.transpose() * A2)(0, 0));
//	check = num1 < num2;
//	if (check) {
//		return false;
//	}
//
//	//B0, B1, B2
//	num1 = a0 * std::abs(C(0, 0)) + a1 * std::abs(C(1, 0)) + a2 * std::abs(C(2, 0)) + b0;
//	num2 = std::abs((D.transpose() * B0)(0, 0));
//	check = num1 < num2;
//	if (check) {
//		return false;
//	}
//	check = a0 * std::abs(C(0, 1)) + a1 * std::abs(C(1, 1)) + a2 * std::abs(C(2, 1)) + b1 < std::abs((D.transpose()* B1)(0, 0));
//	if (check) {
//		return false;
//	}
//	check = a0 * std::abs(C(0, 2)) + a1 * std::abs(C(1, 2)) + a2 * std::abs(C(2, 2)) + b2 < std::abs((D.transpose()* B2)(0, 0));
//	if (check) {
//		return false;
//	}
//
//	//A0xB0, A0xB1, A0xB2
//	check = a1 * std::abs(C(2, 0)) + a2 * std::abs(C(1, 0)) + b1 * std::abs(C(0, 2)) + b2 * std::abs(C(0, 1)) < std::abs((C(1, 0) * D.transpose() * A2)(0, 0) - (C(2, 0) * D.transpose() * A1)(0, 0));
//	if (check) {
//		return false;
//	}
//	check = a1 * std::abs(C(2, 1)) + a2 * std::abs(C(1, 1)) + b0 * std::abs(C(0, 2)) + b2 * std::abs(C(0, 0)) < std::abs((C(1, 1) * D.transpose() * A2)(0, 0) - (C(2, 1) * D.transpose() * A1)(0, 0));
//	if (check) {
//		return false;
//	}
//	check = a1 * std::abs(C(2, 2)) + a2 * std::abs(C(1, 2)) + b0 * std::abs(C(0, 1)) + b1 * std::abs(C(0, 0)) < std::abs((C(1, 2) * D.transpose() * A2)(0, 0) - (C(2, 2) * D.transpose() * A1)(0, 0));
//	if (check) {
//		return false;
//	}
//
//	//A1xB0, A1xB1, A1xB2
//	check = a0 * std::abs(C(2, 0)) + a2 * std::abs(C(0, 0)) + b1 * std::abs(C(1, 2)) + b2 * std::abs(C(1, 1)) < std::abs((C(2, 0) * D.transpose() * A0)(0, 0) - (C(0, 0) * D.transpose() * A2)(0, 0));
//	if (check) {
//		return false;
//	}
//	check = a0 * std::abs(C(2, 1)) + a2 * std::abs(C(0, 1)) + b0 * std::abs(C(1, 2)) + b2 * std::abs(C(1, 0)) < std::abs((C(2, 1) * D.transpose() * A0)(0, 0) - (C(0, 1) * D.transpose() * A2)(0, 0));
//	if (check) {
//		return false;
//	}
//	check = a0 * std::abs(C(2, 2)) + a2 * std::abs(C(0, 2)) + b0 * std::abs(C(1, 1)) + b1 * std::abs(C(1, 0)) < std::abs((C(2, 2) * D.transpose() * A0)(0, 0) - (C(0, 2) * D.transpose() * A2)(0, 0));
//	if (check) {
//		return false;
//	}
//
//	//A2xB0, A2xB1, A2xB2
//	check = a0 * std::abs(C(1, 0)) + a1 * std::abs(C(0, 0)) + b1 * std::abs(C(2, 2)) + b2 * std::abs(C(2, 1)) < std::abs((C(0, 0) * D.transpose() * A1)(0, 0) - (C(1, 0) * D.transpose() * A0)(0, 0));
//	if (check) {
//		return false;
//	}
//	check = a0 * std::abs(C(1, 1)) + a1 * std::abs(C(0, 1)) + b0 * std::abs(C(2, 2)) + b2 * std::abs(C(2, 0)) < std::abs((C(0, 1) * D.transpose() * A1)(0, 0) - (C(1, 1) * D.transpose() * A0)(0, 0));
//	if (check) {
//		return false;
//	}
//	check = a0 * std::abs(C(1, 2)) + a1 * std::abs(C(0, 2)) + b0 * std::abs(C(2, 1)) + b1 * std::abs(C(2, 0)) < std::abs((C(0, 2) * D.transpose() * A1)(0, 0) - (C(1, 2) * D.transpose() * A0)(0, 0));
//	if (check) {
//		return false;
//	}
//
//	return true;
//}
//
//bool tree_intersect(igl::AABB<Eigen::MatrixXd, 3>* tree_A, igl::AABB<Eigen::MatrixXd, 3>* tree_B, igl::opengl::ViewerData* data_A, igl::opengl::ViewerData* data_B) {
//	if (!box_intersect(tree_A->m_box, tree_B->m_box, data_A, data_B)) {
//		return false;
//	}
//	else if (tree_A->is_leaf() && tree_B->is_leaf()) {
//		/*display_box(tree_A->m_box, data_A);
//		display_box(tree_B->m_box, data_B);*/
//		return true;
//	}
//	else if (!tree_A->is_leaf() && tree_B->is_leaf()) {
//		bool left = tree_intersect(tree_A->m_left, tree_B, data_A, data_B);
//		bool right = tree_intersect(tree_A->m_right, tree_B, data_A, data_B);
//
//		return left || right;
//	}
//	else if (tree_A->is_leaf() && !tree_B->is_leaf()) {
//		bool left = tree_intersect(tree_A, tree_B->m_left, data_A, data_B);
//		bool right = tree_intersect(tree_A, tree_B->m_right, data_A, data_B);
//		return left || right;
//	}
//	else {
//		bool left_left = tree_intersect(tree_A->m_left, tree_B->m_left, data_A, data_B);
//		bool right_left = tree_intersect(tree_A->m_right, tree_B->m_left, data_A, data_B);
//		bool left_right = tree_intersect(tree_A->m_left, tree_B->m_right, data_A, data_B);
//		bool right_right = tree_intersect(tree_A->m_right, tree_B->m_right, data_A, data_B);
//		return left_left || right_left || left_right || right_right;
//	}
//
//}
//
//int SandBox::intersect(igl::opengl::ViewerData* data) {
//	for (int i = 0; i < data_list.size() - 1; i++)
//	{
//		if (&data_list[i] != data) {
//			igl::AABB<Eigen::MatrixXd, 3> tree_A = data->tree;
//			igl::AABB<Eigen::MatrixXd, 3> tree_B = data_list[i].tree;
//
//			if (tree_intersect(&tree_A, &tree_B, data, &data_list[i])) {
//				return i;
//			}
//		}
//	}
//	
//	return -1;
//}
//

int SandBox::intersectHead() {
	Eigen::Vector3d headPos = toCenter(joints[0]);
	for (int i = 0; i < data_list.size() - 1; i++)
	{
		if (&data_list[i] != snake) {
			double diameter = data(i).getDiameter();
			if ((toCenter(data(i)) - headPos).norm() < diameter/2 + 0.1) {
				return i;
			}
		}
	}
	return -1;
}

void SandBox::incScore() {
	score++;
}

void SandBox::eatSphere(int dataNum) {
	data(dataNum).clear();
	data(dataNum).MyTranslate(Eigen::Vector3d(100, 100, 100),false);
	incScore();
	cout << "Score: " << score << endl;
}

void SandBox::Animate()
{
	skin();
	debugSnake();
	setIKOverlay();
	int intersectedWith = intersectHead();
	if (intersectedWith != -1) {
		cout << "YUM" << endl;
		eatSphere(intersectedWith);
	}
	if (firstPersonView) {
		const Eigen::Vector3f headPos = getHeadPos().cast<float>();
		rndr->core().camera_eye = headPos;
		rndr->core().camera_center = headPos + (joints[0].GetRotation() * Eigen::Vector3d(0, 0, 1)).cast<float>();
		rndr->core().camera_up = Eigen::Vector3f(0, 1, 0);
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
	vQ[0] = Eigen::Quaterniond(temp.block<3,3>(0,0).transpose());
}