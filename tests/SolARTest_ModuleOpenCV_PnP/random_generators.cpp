#include "random_generators.hpp"
#include <math.h>

# define M_PI   3.14159265358979323846
using namespace Eigen;

void SolAR::PnPTest::initializeRandomSeed()
{
	srand(time(nullptr));
}

Eigen::Vector3f SolAR::PnPTest::generateRandom3DPointInFrontOfCam(const SolAR::datastructure::Transform3Df& pose)
{
	Eigen::Vector3f cleanPoint;
	cleanPoint[0] = (((float) rand())/ ((float) RAND_MAX)-0.5)*2.0;
	cleanPoint[1] = (((float) rand())/ ((float) RAND_MAX)-0.5)*2.0;
	cleanPoint[2] = (((float) rand())/ ((float) RAND_MAX))*2.0;
	return pose * cleanPoint;
}

Eigen::Vector3f SolAR::PnPTest::generateRandomTranslation( float maximumParallax )
{
	Eigen::Vector3f translation;
	translation[0] = (((float) rand())/ ((float) RAND_MAX)-0.5)*2.0;
	translation[1] = (((float) rand())/ ((float) RAND_MAX)-0.5)*2.0;
	translation[2] = (((float) rand())/ ((float) RAND_MAX)-0.5)*2.0;
	return maximumParallax * translation;
}

Eigen::Matrix3f SolAR::PnPTest::generateRandomRotation( float maxAngle )
{
	Eigen::Vector3f rpy;
	rpy[0] = ((float) rand())/ ((float) RAND_MAX);
	rpy[1] = ((float) rand())/ ((float) RAND_MAX);
	rpy[2] = ((float) rand())/ ((float) RAND_MAX);

	rpy[0] = maxAngle*2.0*(rpy[0]-0.5);
	rpy[1] = maxAngle*2.0*(rpy[1]-0.5);
	rpy[2] = maxAngle*2.0*(rpy[2]-0.5);

	Eigen::Matrix3f R1;
	R1(0,0) = 1.0;
	R1(0,1) = 0.0;
	R1(0,2) = 0.0;
	R1(1,0) = 0.0;
	R1(1,1) = cos(rpy[0]);
	R1(1,2) = -sin(rpy[0]);
	R1(2,0) = 0.0;
	R1(2,1) = -R1(1,2);
	R1(2,2) = R1(1,1);

	Eigen::Matrix3f R2;
	R2(0,0) = cos(rpy[1]);
	R2(0,1) = 0.0;
	R2(0,2) = sin(rpy[1]);
	R2(1,0) = 0.0;
	R2(1,1) = 1.0;
	R2(1,2) = 0.0;
	R2(2,0) = -R2(0,2);
	R2(2,1) = 0.0;
	R2(2,2) = R2(0,0);

	Eigen::Matrix3f R3;
	R3(0,0) = cos(rpy[2]);
	R3(0,1) = -sin(rpy[2]);
	R3(0,2) = 0.0;
	R3(1,0) =-R3(0,1);
	R3(1,1) = R3(0,0);
	R3(1,2) = 0.0;
	R3(2,0) = 0.0;
	R3(2,1) = 0.0;
	R3(2,2) = 1.0;

	Eigen::Matrix3f rotation = R3 * R2 * R1;

	rotation.col(0) = rotation.col(0) / rotation.col(0).norm();
	rotation.col(2) = rotation.col(0).cross(rotation.col(1));
	rotation.col(2) = rotation.col(2) / rotation.col(2).norm();
	rotation.col(1) = rotation.col(2).cross(rotation.col(0));
	rotation.col(1) = rotation.col(1) / rotation.col(1).norm();

	return rotation;
}