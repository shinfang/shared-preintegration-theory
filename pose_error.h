
#ifndef EXAMPLES_ERROR
#define EXAMPLES_ERROR

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/autodiff_cost_function.h"
#include <fstream>

#include "integration_imu.hpp"
//#include "preintegration_func.hpp"
//#include "SO3_lib.h"

namespace ceres {
namespace examples {

class IMUFunctor_Bias{

	public:
	IMUFunctor_Bias(const VectorOfConstraints& a, int index): omega_vector(a), index_w(index){}

	template <typename T>

	bool operator() (const T* const attitude_i_hat, const T* const attitude_i_1_hat,
					const T* const velocity_i_hat, const T* const velocity_i_1_hat,
					const T* const position_i_hat, const T* const position_i_1_hat,
					const T* const bias_hat_gyro,
					T* residual)const{

	/*	bool operator() (const T* const attitude_i_hat, const T* const attitude_i_1_hat,
							const T* const velocity_i_hat, const T* const velocity_i_1_hat,
							const T* const position_i_hat, const T* const position_i_1_hat,
							const T* const bias_hat_gyro, const T* const bias_hat_acc, T* residual)const{  */

		int c = 0;
		int index_preint = 1;
		double dt = 0.005;
		double dtij = dt * 4;

		Eigen::Matrix<T,3,1> gravity_vector;
		gravity_vector << T(0.0),T(0.0),T(9.8);

		Eigen::Map<const Eigen::Quaternion<T> > atti_hat_i(attitude_i_hat);
		Eigen::Map<const Eigen::Quaternion<T> > atti_hat_j(attitude_i_1_hat);

		Eigen::Map<const Eigen::Matrix<T,3,1> > velocity_hat_i(velocity_i_hat);
		Eigen::Map<const Eigen::Matrix<T,3,1> > velocity_hat_j(velocity_i_1_hat);

		Eigen::Map<const Eigen::Matrix<T,3,1> > position_hat_i(position_i_hat);
		Eigen::Map<const Eigen::Matrix<T,3,1> > position_hat_j(position_i_1_hat);


		std::map<int,Eigen::Quaternion<T> > Preintegrated_omega_bias;
		std::map<int,Eigen::Matrix<T,3,1> > Preintegrated_velocity;
		std::map<int, Eigen::Matrix<T,3,1> > Preintegrated_position;

		Eigen::Map<const Eigen::Matrix<T,3,1> > bias_hat_gyro_(bias_hat_gyro);
		//Eigen::Map<const Eigen::Matrix<T,3,1> > bias_hat_acc_(bias_hat_acc);

		Eigen::Matrix<T,3,3> delta_Rij = Eigen::Matrix<T,3,3>::Identity();
		Eigen::Matrix<T,3,1> delta_Rik = Eigen::Matrix<T,3,1>::Zero();
		Eigen::Matrix<T,3,1> relative_p = Eigen::Matrix<T,3,1>::Zero();


//Recompute preintegrated omega with optimized bias
for (VectorOfConstraints::const_iterator constraints_iter = omega_vector.begin();constraints_iter!=omega_vector.end();++constraints_iter)
	{

	// ----- estimated bias gyroscope, bias accelerometer
	c = c +1;
	const Constraint3d& gyro = *constraints_iter;



	Eigen::Matrix<T,3,1> accCorrect = gyro.acceleration.template cast<T>() ;//- bias_hat_acc_ ;
	//	std::cout<<"ACCELEROMETER MEASUREMENT" << accCorrect<<std::endl;

	Eigen::Matrix<T,3,1> gyroCorrect =  ( gyro.ang_velocity.template cast<T>() - bias_hat_gyro_) * dt;
	//std::cout<<"Prior to correction"<<gyro.ang_velocity.template cast<T>()<<std::endl;
	//std::cout<<"Corrected gyro measurement"<<gyroCorrect<<std::endl;


	// relative velocity
	delta_Rik = delta_Rik + ( delta_Rij*accCorrect * dt );


	// relative position
	relative_p = relative_p +  (T(3/2) * delta_Rij * accCorrect * dt * dt);


    Eigen::Matrix<T,3,3> skew_gyro = Eigen::Matrix<T,3,3>::Zero();
			skew_gyro(0,1) = -gyroCorrect[2];
			skew_gyro(0,2) = gyroCorrect[1];
			skew_gyro(1,2) = -gyroCorrect[0];

			skew_gyro(1,0) = gyroCorrect[2];
			skew_gyro(2,0) = -gyroCorrect[1];
			skew_gyro(2,1) = gyroCorrect[0];

			T phi = gyroCorrect.norm();
			T phi_inv = T(1.0) /phi;

		Eigen::Matrix<T,3,3> iden = Eigen::Matrix<T,3,3>::Identity();
		Eigen::Matrix<T,3,3> exp_gyro;

		if (phi>0.000174)
		{

			Eigen::Matrix<T,3,3> first_term = sin(phi) * phi_inv * skew_gyro;
			Eigen::Matrix<T,3,3> second_term = (T(1.0) - cos(phi)) * phi_inv * phi_inv  * skew_gyro * skew_gyro;
			exp_gyro = iden + first_term + second_term;
		}
		else
		{
			T const_1 = T(1.0) - T(1/6)* phi* phi + T(1/120)*phi*phi*phi*phi;
			T const_2 = T(0.5) - T(1/24)* phi*phi + T(1/720) * phi*phi*phi*phi;
			exp_gyro = iden + const_1*skew_gyro + const_2 * skew_gyro* skew_gyro;
		}

			delta_Rij = delta_Rij * exp_gyro;
				//std::cout<<"delta Rij"<<delta_Rij<<std::endl;
				//std::cout<<"Next delta"<<std::endl;

		if (c % 4 == 0)
		{
				//std::cout<<"Enter loop c = 4"<<delta_Rij<<std::endl;
				//std::cout<<"delta Rik"<<delta_Rik<<std::endl;
			Eigen::Quaternion<T> omega_q(delta_Rij);
			omega_q.normalize();

			// ----- store resulting pre-integrated measurement to respective map
			Preintegrated_omega_bias[index_preint] = omega_q;
			Preintegrated_velocity[index_preint] = delta_Rik;
			Preintegrated_position[index_preint] = relative_p;

			index_preint = index_preint + 1;

			delta_Rij = Eigen::Matrix<T,3,3>::Identity();
			delta_Rik = Eigen::Matrix<T,3,1>::Zero();
			relative_p = Eigen::Matrix<T,3,1>::Zero();



		} //end if loop for the computation
	}//end for loop preintegrated measurement

	// reset
	index_preint = 1;



	typename std::map<int,Eigen::Quaternion<T> >::iterator it = Preintegrated_omega_bias.find(index_w);
	typename std::map<int,Eigen::Matrix<T,3,1> >::iterator it_vel = Preintegrated_velocity.find(index_w);
	typename std::map<int,Eigen::Matrix<T,3,1> >::iterator it_pos = Preintegrated_position.find(index_w);
	Eigen::Quaternion<T> constraint_from_map;
	Eigen::Matrix<T,3,1> preint_vel;
	Eigen::Matrix<T,3,1> preint_pos;

			//std::cout<<"constraint"<<constraint_from_map.w()<<std::endl;

			constraint_from_map.w() = it->second.w();
			constraint_from_map.x() = it->second.x();
			constraint_from_map.y() = it->second.y();
			constraint_from_map.z() = it->second.z();

			preint_vel.x() = it_vel->second.x();
			preint_vel.y() = it_vel->second.y();
			preint_vel.z() = it_vel->second.z();

			preint_pos.x() = it_pos->second.x();
			preint_pos.y() = it_pos->second.y();
			preint_pos.z() = it_pos->second.z();

			//std::cout<<"index: "<<index_w<<" "<<preint_vel.x()<<std::endl;
			//std::cout<<"index: "<<index_w<<" "<<preint_vel.y()<<std::endl;
			//std::cout<<"index: "<<index_w<<" "<<preint_vel.z()<<std::endl;

		//std::cout<<"C:"<<c<<" "<<"IMU"<<constraint_from_map.w()<<std::endl;
// ------ Preintegrated measurement ( p, v, q )
			Eigen::Matrix<T,3,1> preintegrated_pos_ij = preint_pos.template cast<T>();
			Eigen::Matrix<T,3,1> preintegrated_vel_ij = preint_vel.template cast<T>();
			Eigen::Quaternion<T> preintegrated_q_ij  = constraint_from_map.template cast<T>();

// ------ Represent the displacement between the two attitudes in the i frame
			Eigen::Quaternion<T> q_i_inverse = atti_hat_i.conjugate();
			/*
			std::cout<<"inverse of q in cost function"<< q_i_inverse.w()<<std::endl;
			std::cout<<"inverse of q in cost function"<< q_i_inverse.x()<<std::endl;
			std::cout<<"inverse of q in cost function"<< q_i_inverse.y()<<std::endl;
			std::cout<<"inverse of q in cost function"<< q_i_inverse.z()<<std::endl;*/
			Eigen::Matrix<T,3,1> p_ij_estimated = q_i_inverse * (position_hat_j - position_hat_i - velocity_hat_i * dtij - 0.5 * gravity_vector * dtij * dtij);

// ------- Represent the velocity between two attitudes in the i frame
			Eigen::Matrix<T,3,1> v_ij_estimated = q_i_inverse * ( velocity_hat_j - velocity_hat_i - gravity_vector * dtij);

			/*std::cout<<"vij estimated"<<v_ij_estimated.x()<<std::endl;
			std::cout<<"vij estimated"<<v_ij_estimated.y()<<std::endl;
			std::cout<<"vij estimated"<<v_ij_estimated.z()<<std::endl; */

// ------- Compute the relative transformation between two attitudes
			Eigen::Quaternion<T> q_ij_estimated = q_i_inverse * atti_hat_j;

// ------ Compare the error between two orientation estimates
			Eigen::Quaternion<T> delta_q = preintegrated_q_ij.conjugate() * q_ij_estimated;

// ------- Compute residuals ( p, v, q )
			Eigen::Map<Eigen::Matrix<T,9,1> > residuals(residual);
			residuals.template block<3,1>(0,0) = T(8.0) *(p_ij_estimated  - preintegrated_pos_ij);
			residuals.template block<3,1>(3,0) = T(8.0) *(v_ij_estimated - preintegrated_vel_ij);
			residuals.template block<3,1>(6,0) = T(1.0) * delta_q.vec();


		return true;
	}
	static ceres::CostFunction* Create(const VectorOfConstraints& a, int index){
			return new ceres::AutoDiffCostFunction<IMUFunctor_Bias,9,4,4,3,3,3,3,3>(
					new IMUFunctor_Bias(a,index));
		}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:


	const VectorOfConstraints& omega_vector;
	//MapOfPoses* bias_map;
	int index_w;


};


//int i=0;

class AttitudeError_x{
	public:


	AttitudeError_x(const Eigen::Quaternion<double> vec_b_x, int index) : vec_b_x_ (vec_b_x) , index_(index){ }

      template <typename T>
            bool operator() (const T* const robot_pose_estimated, T* residual) const{



    	  Eigen::Matrix<T,3,1> p_x_world;
    	  //p_x_world << T(0.0897),T(0.4),T(-0.9121);

    	  p_x_world << T(1),T(0),T(0);

    	  Eigen::Map<const Eigen::Quaternion<T> > pose_hat(robot_pose_estimated);
    	  Eigen::Quaternion<T> vec_b_constant = vec_b_x_.template cast<T>();

/*
    	  std::cout<<"current robot pose R vertex"<<robot_pose_estimated[0]<<std::endl;
    	  std::cout<<robot_pose_estimated[1]<<std::endl;
    	  std::cout<<robot_pose_estimated[2]<<std::endl;
    	  std::cout<<robot_pose_estimated[3]<<std::endl;*/



    	  Eigen::Quaternion<T> vector_rotated = pose_hat * vec_b_constant * pose_hat.conjugate();
    	  vector_rotated.normalize();
    	  Eigen::Map<Eigen::Matrix<T,3,1> >residuals(residual);
    	  residuals = T(10.0)*(p_x_world - vector_rotated.vec());

/*
    			std::cout<<"current robot pose R vertex"<<robot_pose_estimated[0]<<std::endl;
    			std::cout<<robot_pose_estimated[1]<<std::endl;
    			std::cout<<robot_pose_estimated[2]<<std::endl;
    			std::cout<<robot_pose_estimated[3]<<std::endl;*/

    	  	    std::fstream residual_file;
    	  	    residual_file.open("print_poses.txt",std::fstream::out | std::fstream::app);
    	  	    residual_file<<"Residuals x "<<index_<<": "<<residuals[0]<<std::endl;
    	  	    	residual_file<<"Residuals y "<<index_<<": "<<residuals[1]<<std::endl;
    	  	    	residual_file<<"Residuals z "<<index_<<": "<<residuals[2]<<std::endl;

        	  	//std::cout<<"Residuals x "<<index_<<": "<<residuals[0]<<std::endl;
        	  	//std::cout<<"Residuals y "<<index_<<": "<<residuals[1]<<std::endl;
        	  	//std::cout<<"Residuals z "<<index_<<": "<<residuals[2]<<std::endl;
    	  	    	return true;

            }


            static ceres::CostFunction* Create(const Eigen::Quaternion<double> vec_b_x, int pointer_index)
            {
                return new ceres::AutoDiffCostFunction<AttitudeError_x,3,4>(new AttitudeError_x(vec_b_x, pointer_index));

            }
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW


private:
		//members of the class
       const Eigen::Quaternion<double> vec_b_x_;
       int index_;


        };



class AttitudeError_y{
	public:
	AttitudeError_y(const Eigen::Quaternion<double> vector_b_y):vector_b_y_(vector_b_y){}

		template <typename T>
		bool operator() (const T* const robot_pose_estimated, T* residual) const{

			Eigen::Map<const Eigen::Quaternion<T> > attitude_hat(robot_pose_estimated);
			Eigen::Matrix<T,3,1> p_y_world;
			//p_y_world << T(0),T(0),T(-1);

			p_y_world << T(0),T(1),T(0);

			Eigen::Map<const Eigen::Quaternion<T> > pose_hat(robot_pose_estimated);
			Eigen::Quaternion<T> vector_b_y_constant = vector_b_y_.template cast<T>();


			Eigen::Quaternion<T> vector_rotated = pose_hat * vector_b_y_constant * pose_hat.conjugate();


			Eigen::Map<Eigen::Matrix<T,3,1> > residuals(residual);
			residuals = T(10.0)*(p_y_world - vector_rotated.vec());

			return true;
		}

		static ceres::CostFunction* Create(
		const Eigen::Quaternion<double> vector_b_y){
			return new ceres::AutoDiffCostFunction<AttitudeError_y,3,4>(new AttitudeError_y(vector_b_y));
		}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
		const Eigen::Quaternion<double> vector_b_y_;

};

class VelocityError{
	public:
		VelocityError(const Eigen::Vector3d velocity_measurement):velocity_measurement_(velocity_measurement){}

	template <typename T>
	bool operator() (const T* const velocity_hat, T* residual) const{

		Eigen::Map<const Eigen::Matrix<T,3,1> > velocity_hat_(velocity_hat);
		Eigen::Map<Eigen::Matrix<T,3,1> > residuals(residual);
		Eigen::Matrix<T,3,1> Vm = velocity_measurement_.template cast<T>();

		//std::cout<<"velocity measurement in the cost function"<<Vm<<std::endl;

		residuals = T(1.0) * (Vm - velocity_hat_);
		return true;

	}

	static ceres::CostFunction* Create(const Eigen::Vector3d velocity_measurement){
		return new ceres::AutoDiffCostFunction<VelocityError,3,3>(new VelocityError(velocity_measurement));
	}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
		const Eigen::Vector3d velocity_measurement_;
};

class PositionError{
	public:
		PositionError(const Eigen::Vector3d position_measurement):position_measurement_(position_measurement){}

	template <typename T>
	bool operator() (const T* const position_hat, T* residual) const{

		Eigen::Map<const Eigen::Matrix<T,3,1> > position_hat_(position_hat);
		Eigen::Map<Eigen::Matrix<T,3,1> > residuals(residual);
		Eigen::Matrix<T,3,1> Pm = position_measurement_.template cast<T>();

		//std::cout<< "position measurement in the cost function"<<Pm<<std::endl;

		residuals = T(1.0)*(Pm - position_hat_);
		return true;
	}

	static ceres::CostFunction* Create(const Eigen::Vector3d position_measurement) {
		return new ceres::AutoDiffCostFunction<PositionError,3,3>(new PositionError(position_measurement));
	}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
		const Eigen::Vector3d position_measurement_;
};

class Bias_Gyro{
	public:
	Bias_Gyro(const Eigen::Vector3d bias):bias_(bias){}

	template <typename T>
	bool operator() (const T* const bias_G_hat_i, const T* const bias_G_hat_j, T* residual) const{


		Eigen::Map<const Eigen::Matrix<T,3,1> > bias_G_hat_i_(bias_G_hat_i);
		Eigen::Map<const Eigen::Matrix<T,3,1> > bias_G_hat_j_(bias_G_hat_j);


		Eigen::Map<Eigen::Matrix<T,3,1> > residuals(residual);


		residuals = T(30.0)* (bias_G_hat_i_- bias_G_hat_j_);

		return true;
	}



	static ceres::CostFunction* Create(
		const Eigen::Vector3d bias_	){
			return new ceres::AutoDiffCostFunction<Bias_Gyro,3,3,3>(new Bias_Gyro(bias_));

	}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	const Eigen::Vector3d bias_;
};

class Bias_Acc{
	public:
	Bias_Acc(const Eigen::Vector3d bias):bias_(bias){}

	template <typename T>
	bool operator() (const T* const bias_A_hat_i, const T* const bias_A_hat_j, T* residual) const{


		Eigen::Map<const Eigen::Matrix<T,3,1> > bias_A_hat_i_(bias_A_hat_i);
		Eigen::Map<const Eigen::Matrix<T,3,1> > bias_A_hat_j_(bias_A_hat_j);
		Eigen::Map<Eigen::Matrix<T,3,1> > residuals(residual);

		std::cout<<"Bias hat i"<< bias_A_hat_i_<<std::endl;
		std::cout<<"Bias hat j"<< bias_A_hat_j_<<std::endl;

		residuals = T(1.0)* (bias_A_hat_i_- bias_A_hat_j_);

		return true;
	}



	static ceres::CostFunction* Create(
		const Eigen::Vector3d bias_	){
			return new ceres::AutoDiffCostFunction<Bias_Acc,3,3,3>(new Bias_Acc(bias_));

	}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	const Eigen::Vector3d bias_;
};



}  // namespace examples
}  // namespace ceres

#endif  // EXAMPLES_CERES_READ_G2O_H_
