#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000; // number of landmarks in the whole window i.e. total number of features observed in 10 frames

// TO DO: barza: make these all externs
const double MASS = 1.56779;
extern Eigen::Matrix3d INERTIA;
const double PROPELLER_FORCE_N = 0.01; //prop force std
const bool APPLY_MODEL_PREINTEGRATION = 1;


//#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W; //std_dev of accel noise and accel bias random walk noise
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC; // rotation of camera wrt IMU
extern std::vector<Eigen::Vector3d> TIC; // translation of camera wrt IMU
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string IMU_TOPIC;
extern double TD;
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern double ROW, COL; //image height 480, width 752


void readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSITION = 3, // 3 translation 
    SIZE_ATTITUDE = 4, //  4 quaternion
    SIZE_SPEED = 3, // 3v
    SIZE_BIAS = 6, // 3ba, 3bg
    //SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1, // 1/Zc for each landmark
    SIZE_FORCES = 3

    //SIZE_POSE = 7 // for extrinsic pose

};

enum StateOrder
{
    O_P = 0,
    O_R = 3, // 3,4,5 for rotation? so here we mean minimal state
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

/*enum NoiseOrder
{   //these are not used
    O_AN = 0, // noise
    O_GN = 3,
    O_AW = 6, // bias random walk
    O_GW = 9,
    O_UN = 12 // change this, index for noise in utot i.e. in body Fx

};*/
