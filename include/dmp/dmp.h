/*
 * @Author: Boris.Peng
 * @Date: 2020-06-18 14:58:26
 * @LastEditors: Boris.Peng
 * @LastEditTime: 2020-06-19 15:03:36
 */ 
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Scott Niekum
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

/**
  * \author Scott Niekum
  */

#ifndef DMP_H_
#define DMP_H_

// #include "ros/ros.h"
// #include "dmp/LearnDMPFromDemo.h"
// #include "dmp/GetDMPPlan.h"
// #include "dmp/SetActiveDMP.h"
#include "dmp/radial_approx.h"
#include "dmp/fourier_approx.h"
#include "dmp/linear_approx.h"
#include <math.h>
#include <string>
#include <fstream>
#include <sstream>

#define MAX_PLAN_LENGTH 1000
#define POSE_DIMENTION_SIZE 7

namespace dmp{

struct DMPPoint {
  std::vector<double> positions;
  std::vector<double> velocities;
};

struct DMPTraj {
  std::vector<DMPPoint> points;
  std::vector<double> times;
};

struct DMPData {
  double k_gain;
  double d_gain;
  std::vector<double> weights;
  std::vector<double> f_domain;
  std::vector<double> f_targets;
};

class dmpClass{

public:
dmpClass(std::string dataPath);

void learnFromDemo(const DMPTraj &demo, const std::vector<double> &k_gains,
				   const std::vector<double> &d_gains, const int &num_bases,
				   std::vector<DMPData> &dmp_list);

void generatePlan(const std::vector<DMPData> &dmp_list, const std::vector<double> &x_0,
				  const std::vector<double> &x_dot_0, const double &t_0,
				  const std::vector<double> &goal, const std::vector<double> &goal_thresh,
				  const double &seg_length, const double &tau, const double &total_dt,
				  const int &integrate_iter, DMPTraj &plan, uint8_t &at_goal);

bool dmp_service(short armSelect, const std::vector<double> &startPose, const std::vector<double> &goalPose,
				 std::vector<std::vector<double>> &plan);
public:
	double calcPhase(double curr_time, double tau);
	int chooseData(short armSelct, const std::vector<double> &startPos, const std::vector<double> &goalPos);
	std::string getLastLine(std::ifstream& in);
	double vecDistance(const std::vector<double> &v1, const std::vector<double> &v2);


	double alpha = -log(0.01); //Ensures 99% phase convergence at t=tau
	std::vector<std::vector<double>> leftArmData, rightArmData;
};

}
#endif /* DMP_H_ */
