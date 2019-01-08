#include "parameters.h"

double THRUST_Z_N;
double THRUST_X_Y_N;
double F_EXT_NORM_WEIGHT;
int APPLY_MODEL_PREINTEGRATION;
int EULER_INTEGRATION;

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W; 

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string RPG_RESULT_EVAL_PATH;
std::string VINS_GT_PATH;
std::string RPG_GT_EVAL_PATH;
std::string EXT_F_GT_PATH;
std::string PREINTEG_PATH;

std::string IMU_TOPIC;
std::string CONTROL_TOPIC;
double ROW, COL;
double TD, TR; // TR is rolling shutter

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["imu_topic"] >> IMU_TOPIC;
    fsSettings["control_topic"] >> CONTROL_TOPIC;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    APPLY_MODEL_PREINTEGRATION = fsSettings["apply_model_preintegration"];
    if (APPLY_MODEL_PREINTEGRATION == 1)
    {
        ROS_INFO("APPLY_MODEL_PREINTEGRATION !!!");
    }

    EULER_INTEGRATION = 0; //fsSettings["euler_integration"];

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;

    std::string RPG_EVAL_PATH;
    fsSettings["rpg_eval_path"] >> RPG_EVAL_PATH;

    std::string SIMULATION_NAME;
    fsSettings["simulation_name"] >> SIMULATION_NAME;

    VINS_GT_PATH = OUTPUT_PATH + SIMULATION_NAME + "/groundtruth.csv" ;
    EXT_F_GT_PATH = OUTPUT_PATH + SIMULATION_NAME + "/external_force_gt.csv";
    PREINTEG_PATH = OUTPUT_PATH + SIMULATION_NAME + "/preintegrations.csv";

    if (APPLY_MODEL_PREINTEGRATION)
    {
        VINS_RESULT_PATH = OUTPUT_PATH + SIMULATION_NAME + "/extFNnBcmodel_result.csv"; //no bias repropagation in imu factor
        RPG_RESULT_EVAL_PATH = RPG_EVAL_PATH + "/extFNnBcmodel_sim/laptop_extFNnBcmodel_sim_" + SIMULATION_NAME + "/stamped_traj_estimate.txt";
        RPG_GT_EVAL_PATH = RPG_EVAL_PATH + "/extFNnBcmodel_sim/laptop_extFNnBcmodel_sim_" + SIMULATION_NAME + "/stamped_groundtruth.txt";
    }
    else
    {
        VINS_RESULT_PATH = OUTPUT_PATH + SIMULATION_NAME + "/vins_result.csv";
        RPG_RESULT_EVAL_PATH = RPG_EVAL_PATH + "/vins_sim/laptop_vins_sim_" + SIMULATION_NAME + "/stamped_traj_estimate.txt";
        RPG_GT_EVAL_PATH = RPG_EVAL_PATH + "/vins_sim/laptop_vins_sim_" + SIMULATION_NAME + "/stamped_groundtruth.txt";
    }
    
    std::cout << "matlab result path " << VINS_RESULT_PATH << std::endl;
    std::cout << "matlab groundtruth path " << VINS_GT_PATH << std::endl;
    std::cout << "rpg result path " << RPG_RESULT_EVAL_PATH << std::endl;
    std::cout << "rpg groundtruth path " << RPG_GT_EVAL_PATH << std::endl;

    //can check for other paths as well here: 0 means all is good. 1 means error: file path must not exist.
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    std::cout << "VINS_RESULT_PATH is opened? " << !fout << " "<< fout.bad() << " "<< fout.fail()<< std::endl;
    fout.close();

    std::ofstream fout1(VINS_GT_PATH, std::ios::out);
    std::cout << "VINS_GT_PATH is opened? " << !fout1 << " "<< fout1.bad() << " "<< fout1.fail()<< std::endl;
    fout1.close();

    if (APPLY_MODEL_PREINTEGRATION)
    {
        std::ofstream fout4(EXT_F_GT_PATH, std::ios::out);
        std::cout << "external force matlab path is opened? " << !fout4 << " "<< fout4.bad() << " "<< fout4.fail()<< std::endl;
        fout4.close();

        std::ofstream foutC(PREINTEG_PATH, std::ios::out);
        std::cout << "preintegrations is opened? " << !foutC << " "<< foutC.bad() << " "<< foutC.fail()<< std::endl;
        foutC.close();
    }

    std::ofstream fout2(RPG_RESULT_EVAL_PATH, std::ios::out);
    std::cout << "RPG_RESULT_EVAL_PATH is opened? " << !fout2 << " "<< fout2.bad() << " "<< fout2.fail()<< std::endl;
    fout2.close();

    std::ofstream fout3(RPG_GT_EVAL_PATH, std::ios::out);
    std::cout << "RPG_GT_EVAL_PATH is opened? " << !fout3 << " "<< fout3.bad() << " "<< fout3.fail()<< std::endl;
    fout3.close();

    THRUST_Z_N = fsSettings["control_thrust_z_n"];
    THRUST_X_Y_N = fsSettings["control_thrust_x_y_n"];
    F_EXT_NORM_WEIGHT = fsSettings["fext_norm_weight"];

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "extrinsic_parameter.csv";

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN("Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());   
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }
    
    fsSettings.release();
}
