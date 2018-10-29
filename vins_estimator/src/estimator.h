#pragma once

#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
//#include <mav_msgs/TorqueThrust.h>
#include <quadrotor_msgs/ControlCommand.h>

#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/model_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/projection_td_factor.h"
#include "factor/marginalization_factor.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>


class Estimator
{
  public:
    Estimator();

    void setParameter();

    // interface
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity, double Fz, const Vector3d &torque);
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header);
    void setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r);

    // internal
    void clearState();
    bool initialStructure();
    bool visualInitialAlign();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();
    void solveOdometry();
    void slideWindowNew();
    void slideWindowOld();
    void optimization();
    void vector2double(bool print_debug);
    void double2vector();
    bool failureDetection();


    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g; //gravity
    MatrixXd Ap[2], backup_A;
    VectorXd bp[2], backup_b;

    Matrix3d ric[NUM_OF_CAM]; //NUM of cam is 1 bcz mono! Rot_imu_camera 
    Vector3d tic[NUM_OF_CAM];

    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)]; // array of 11 matrices, each 3x3 matrix carries rotation of each state/frame in the window
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    Vector3d Fexts[(WINDOW_SIZE + 1)]; //external disturbance forces
    double td; // time offset between camera frame and imu meas 

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    std_msgs::Header Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)]; // 11 preinteg factors?
    Vector3d acc_0, gyr_0, torque_0;
    double Fz_0; 


    vector<double> dt_buf[(WINDOW_SIZE + 1)]; // Delta t between 2 frames in the window?
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)]; // acc at frames
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)]; // ang vel at frames
    vector<double> Fz_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> torque_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu; // bool first_control
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double para_Position[WINDOW_SIZE + 1][SIZE_POSITION];
    double para_Attitude[WINDOW_SIZE + 1][SIZE_ATTITUDE];
    double para_Speed[WINDOW_SIZE + 1][SIZE_SPEED];
    double para_Bias[WINDOW_SIZE + 1][SIZE_BIAS];
    double para_Fext[WINDOW_SIZE + 1][SIZE_FORCES]; //Fx,Fy,Fz at frames
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Position[NUM_OF_CAM][SIZE_POSITION];
    double para_Ex_Attitude[NUM_OF_CAM][SIZE_ATTITUDE];
    //double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    //double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    //relocalization variable
    bool relocalization_info;
    double relo_frame_stamp;
    double relo_frame_index;
    int relo_frame_local_index;
    vector<Vector3d> match_points;
    //double relo_Pose[SIZE_POSE];
    double relo_Position[SIZE_POSITION];
    double relo_Attitude[SIZE_ATTITUDE];
    Matrix3d drift_correct_r;
    Vector3d drift_correct_t;
    Vector3d prev_relo_t;
    Matrix3d prev_relo_r;
    Vector3d relo_relative_t;
    Quaterniond relo_relative_q;
    double relo_relative_yaw;
};
