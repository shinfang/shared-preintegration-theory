// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2016 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: vitus@google.com (Michael Vitus)
//
// Reads a file in the g2o filename format that describes a pose graph problem.

#ifndef EXAMPLES_CERES_READ_FILE
#define EXAMPLES_CERES_READ_FILE

#include <fstream>
#include <string>
#include <map>
#include <vector>
#include "eigen3/unsupported/Eigen/src/MatrixFunctions/MatrixExponential.h"


#include "glog/logging.h"

namespace ceres {
namespace examples {



template <typename Pose, typename Allocator>
    void ReadVertex (std::ifstream* infile,
                        std::map<int, Pose, std::less<int>, Allocator>* poses){

           int id;
           Pose pose;
           *infile >> id >> pose;

           //Ensure no duplicate poses

           (*poses)[id] = pose;
}


//Reads the constraint between two vertices
//vector
template <typename Constraint, typename Allocator>
	void ReadConstraint (std::ifstream* infile,
						std::vector<Constraint, Allocator>* constraints){

	Constraint constraint;
	*infile >> constraint;

	constraints -> push_back(constraint);
}




template <typename Pose, typename Constraint, typename MapAllocator,  typename VectorAllocator>
bool ReadFile(const std::string& filename,
				std::map<int,Pose,std::less<int>, MapAllocator>* poses,
				std::vector<Constraint,VectorAllocator>* constraints)
{
	CHECK(poses!=NULL);
	CHECK(constraints!=NULL);

	poses->clear();
	constraints ->clear();

	std::ifstream infile(filename.c_str());

	if (!infile){
		return false;
	}

	std::string data_type;
	while(infile.good()){

		//Read whether the type is a node or constraint
		infile >> data_type;

		if(data_type == Pose::name()){
			ReadVertex(&infile, poses);
		}

		else if (data_type == Constraint::name()){
			ReadConstraint(&infile,constraints);
		}
		else
		{
			LOG(ERROR)<<"invalid"<<std::endl;
			return false;
		}

		infile >> std::ws;
	}
		 //while loop
	return true;
}//read file function





}  // namespace examples
}  // namespace ceres

#endif  // EXAMPLES_CERES_READ_G2O_H_
