//
//  integration_imu.hpp
//  
//
//  Created by Shin Fang on 29/11/17.
//

#ifndef integration_imu_hpp
#define integration_imu_hpp

#include <istream>
#include <map>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "eigen3/unsupported/Eigen/src/MatrixFunctions/MatrixExponential.h"



struct Pose3d{

	// measurements
	Eigen::Vector3d vec_x;
	Eigen::Vector3d vec_y;
	Eigen::Vector3d velocity_gps;
	Eigen::Vector3d position_gps;

	// initialization of state
	Eigen::Vector3d bias_G;
	Eigen::Vector3d bias_A;
	Eigen::Quaterniond q;
	Eigen::Vector3d v;
	Eigen::Vector3d p;

	static std::string name(){
		return "VERTEX_SE3";
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

 // ----- Vectorial Measurement X -> Vectorial Measurement Y -> Bias Gyroscope -> Bias Accelerometer -> Attitude -> Velocity -> Position
std::istream& operator>>(std::istream& input, Pose3d& pose){
	input >> 	pose.vec_x.x() >> pose.vec_x.y() >>pose.vec_x.z() >> pose.vec_y.x() >> pose.vec_y.y()>> pose.vec_y.z()>>
				pose.velocity_gps.x() >> pose.velocity_gps.y() >> pose.velocity_gps.z() >>
				pose.position_gps.x() >> pose.position_gps.y() >> pose.position_gps.z() >>
					pose.q.w() >> pose.q.x() >> pose.q.y() >> pose.q.z() >>
					pose.v.x() >> pose.v.y() >> pose.v.z()>>
					pose.p.x() >> pose.p.y() >> pose.p.z()>>
					pose.bias_G.x()>> pose.bias_G.y() >> pose.bias_G.z();

	return input;
}

typedef std::map<int,Pose3d, std::less<int>,
				Eigen::aligned_allocator<std::pair<const int, Pose3d> > > MapOfPoses;


struct Constraint3d{

	Eigen::Vector3d ang_velocity;
	Eigen::Vector3d acceleration;

	static::std::string name(){
		return "EDGE_SE3:VECTOR";
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


 // ------ Measured angular velocity, Measured linear acceleration
std::istream& operator>>(std::istream& input, Constraint3d& constraint){
	input>>constraint.ang_velocity.x() >>constraint.ang_velocity.y()>>constraint.ang_velocity.z() >> constraint.acceleration.x() >>constraint.acceleration.y() >> constraint.acceleration.z();

	return input;
}

typedef std::vector<Constraint3d,Eigen::aligned_allocator<Constraint3d> >VectorOfConstraints;




#endif /* integration_imu_hpp */
