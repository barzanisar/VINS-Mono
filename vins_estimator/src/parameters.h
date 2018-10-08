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
const int NUM_OF_F = 1000;

const double MASS = 1.56779;
extern Eigen::Matrix3d INERTIA;
const double PROPELLER_FORCE_N = 0.1; //prop force std
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
    SIZE_POSE = 7, // 3 translation, 4 quaternion 
    SIZE_SPEEDBIAS = 9, // 3v,3ba,3bg
    SIZE_FEATURE = 1, // 1/Zc for each landmark
    SIZE_FORCES = 3

};

enum StateOrder
{
    O_P = 0,
    O_R = 3, // 3,4,5 for rotation? so here we mean minimal state
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9,
    O_UN = 10 // change this, index for noise in utot i.e. in Fz

};
