#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"


Estimator estimator;

std::condition_variable con;
double current_time = -1;
double current_control_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<quadrotor_msgs::ControlCommand::ConstPtr> control_buf; //quadrotor_msgs/ControlCommand
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::PointCloudConstPtr> relo_buf;
quadrotor_msgs::ControlCommandPtr last_control_msg_ptr = nullptr;
bool start_recording = true;
//quadrotor_common::ControlCommand last_control_cmd;
//quadrotor_msgs::ControlCommand last_control_msg;
int sum_of_wait = 0;
int count_debug = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;
bool beginning = true;
double last_Fz=0;

double last_control_t = 0;
double first_control_t = 0;
bool first_imu_control_match = false;
bool first_control_msg_received = false;
//bool first_img_removed=false;

void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time; // sampling time between two imu msgs
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;
    //ROS_INFO_STREAM_ONCE("tmpQ" << tmp_Q << estimator g << estimator.g.transpose());

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration; //old measurement = new meas
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front()); //oldest imu msg in the buf that is predicted and popped out

}


std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>,std::vector<quadrotor_msgs::ControlCommand::ConstPtr>>, sensor_msgs::PointCloudConstPtr>>
getMeasurements1()
{
    //std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
    std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>,std::vector<quadrotor_msgs::ControlCommand::ConstPtr>>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty() || control_buf.empty())
            return measurements;

        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td) || !(control_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td)) // || !(control_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            //ROS_WARN("wait for imu or control, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (false && control_buf.front()->header.stamp.toSec() < imu_buf.front()->header.stamp.toSec() && beginning)
        {
            ROS_WARN_STREAM("throw controls with ts, only should happen at the beginning" << control_buf.front()->header.stamp.toSec());
            control_buf.pop(); //pops front
            last_Fz = control_buf.front()->collective_thrust;
            continue;
        }
        else if (imu_buf.front()->header.stamp.toSec() < control_buf.front()->header.stamp.toSec() && beginning)
        {
            ROS_WARN_STREAM("throw IMUs with ts, only should happen at the beginning" << imu_buf.front()->header.stamp.toSec());
            imu_buf.pop(); //pops front
            continue;
        }

        beginning =  false;

        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop(); //pops front
            continue;
        }

        
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        std::vector<quadrotor_msgs::ControlCommand::ConstPtr> controls;
        
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            ROS_INFO_STREAM_ONCE("filling IMU buf" << imu_buf.front()->header.stamp.toSec());
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        //ROS_INFO_STREAM("filled IMU buf" << imu_buf.front()->header.stamp.toSec());

        while (control_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            ROS_INFO_STREAM_ONCE("filling control buf" << control_buf.front()->header.stamp.toSec());
            controls.emplace_back(control_buf.front());
            control_buf.pop();
        }
        controls.emplace_back(control_buf.front());

/*
        if (last_control_msg_ptr)
        {
            //ROS_INFO_STREAM("last_control_msg not null!");
            controls.emplace_back(last_control_msg_ptr);
        }

        while (control_buf.front()->header.stamp.toSec() <= img_msg->header.stamp.toSec() + estimator.td)
        {
            //ROS_INFO_STREAM_ONCE("filling control buf" << control_buf.front()->header.stamp.toSec());
            controls.emplace_back(control_buf.front());
            control_buf.pop();
        }


        quadrotor_msgs::ControlCommandPtr last_control_msg_new_ptr (new quadrotor_msgs::ControlCommand);
        ROS_ASSERT(last_control_msg_new_ptr != NULL);
        last_control_msg_new_ptr->header.stamp = img_msg->header.stamp; //+ estimator.td;
        last_control_msg_new_ptr->collective_thrust = controls.back()->collective_thrust;

        last_control_msg_ptr = last_control_msg_new_ptr;

        if (controls.back()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {   
            controls.emplace_back(last_control_msg_new_ptr);
            //ROS_INFO_STREAM_ONCE("filled control buf" << controls.back()->header.stamp.toSec());
        }
*/
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        if (controls.empty())
            ROS_WARN("no controls between two image");
        measurements.emplace_back(std::make_pair(IMUs, controls), img_msg);
        //measurements.emplace_back(IMUs, img_msg);
        //ROS_INFO("IMUs and control front, back and img timestamp: %f, %f, %f, %f, %f \n", IMUs.front()->header.stamp.toSec(), controls.front()->header.stamp.toSec(), IMUs.back()->header.stamp.toSec(), controls.back()->header.stamp.toSec(), img_msg->header.stamp.toSec()); //buzz
        //printf("IMUs and control front, back and img timestamp: %f, %f, %f \n", IMUs.front()->header.stamp.toSec(), IMUs.back()->header.stamp.toSec(), img_msg->header.stamp.toSec());
    
    }
    return measurements; //never reached?
}


void imu_callback1(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();
    ROS_INFO_STREAM_ONCE("First imu_msg_stamp_sec: " << imu_msg->header.stamp.toSec());

        m_buf.lock();
        imu_buf.push(imu_msg);
        m_buf.unlock();
        con.notify_one();

        last_imu_t = imu_msg->header.stamp.toSec();

        {
            std::lock_guard<std::mutex> lg(m_state);
            predict(imu_msg);
            std_msgs::Header header = imu_msg->header;
            header.frame_id = "world";
            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
                pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
        }


}

void control_inputs_callback(const quadrotor_msgs::ControlCommand::ConstPtr& torque_thrust_msg)
{
    if (torque_thrust_msg->header.stamp.toSec() <= last_control_t)
    {
/*        ROS_WARN("torque_thrust message in disorder!");
            if (torque_thrust_msg->header.stamp.toSec() == last_control_t)
                ROS_WARN("skipping duplicate timestamped torque_thrust message!");*/
        
        return;
    }
    last_control_t = torque_thrust_msg->header.stamp.toSec();


    ROS_INFO_STREAM_ONCE("First control_msg_stamp_sec: "<< torque_thrust_msg->header.stamp.toSec());
    

    m_buf.lock();
    control_buf.push(torque_thrust_msg);
    //ROS_INFO("pushing control_msg_stamp_sec: %f \n", torque_thrust_msg->header.stamp.toSec());
    m_buf.unlock();
    con.notify_one();

}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg); //one frame msg
    m_buf.unlock();
    con.notify_one();
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        while(!control_buf.empty())
            control_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        current_control_time -1;
        last_imu_t = 0;
        last_control_t = 0;
    }
    return;
}

void relocalization_callback(const sensor_msgs::PointCloudConstPtr &points_msg)
{
    //printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

void groundtruth_callback(const nav_msgs::OdometryConstPtr &gt_msg)
{
    //ofstream foutA("/home/barza/barza-vins-out/model_loop_estimated.txt", ios::app);

        ofstream foutA(RPG_GT_EVAL_PATH, ios::app);
        foutA.setf(ios::fixed, ios::floatfield);
        if (start_recording)
        {
            foutA << "# time x y z qx qy qz qw" << endl;
            start_recording = false;
        }
        foutA.precision(12);
        foutA << gt_msg->header.stamp.toSec() << " ";
        foutA.precision(5);
        foutA << gt_msg->pose.pose.position.x << " "
              << gt_msg->pose.pose.position.y << " "
              << gt_msg->pose.pose.position.z << " "
              << gt_msg->pose.pose.orientation.x << " "
              << gt_msg->pose.pose.orientation.y << " "
              << gt_msg->pose.pose.orientation.z << " "
              << gt_msg->pose.pose.orientation.w << endl;
        foutA.close();

    // write result to file
        ofstream foutC(VINS_GT_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(0);
        foutC << gt_msg->header.stamp.toSec() * 1e9 << ",";
        foutC.precision(5);
        foutC << gt_msg->pose.pose.position.x << "," //2
              << gt_msg->pose.pose.position.y << ","
              << gt_msg->pose.pose.position.z << ","
              << gt_msg->pose.pose.orientation.w << "," //5
              << gt_msg->pose.pose.orientation.x << ","
              << gt_msg->pose.pose.orientation.y << ","
              << gt_msg->pose.pose.orientation.z << ","
              << gt_msg->twist.twist.linear.x << "," //9
              << gt_msg->twist.twist.linear.y << ","
              << gt_msg->twist.twist.linear.z << endl;
        foutC.close();

        
}

// thread: visual-inertial odometry
void VIO_MODEL_process()
{
    while (true)
    {
        //std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>,std::vector<quadrotor_msgs::ControlCommand::ConstPtr>>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements1()).size() != 0;
                 });
        lk.unlock();
        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0, Fz = last_Fz, Tx = 0, Ty = 0, Tz = 0; //imu meas container
            std::vector<sensor_msgs::ImuConstPtr>::iterator imu_it = measurement.first.first.begin(); 
            std::vector<quadrotor_msgs::ControlCommand::ConstPtr>::iterator control_it = measurement.first.second.begin();

            
            for (; imu_it!=measurement.first.first.end(); ++imu_it)//for (auto &imu_msg : measurement.first.first)
            {
                double t = (*imu_it)->header.stamp.toSec();
                

                if (imu_it == measurement.first.first.begin()){
                    ROS_DEBUG_STREAM("IMU begin time, acc: "<< t << "\t" << (*imu_it)->linear_acceleration.x << "\t" << (*imu_it)->linear_acceleration.y << "\t" << (*imu_it)->linear_acceleration.z);
                }

                if ((imu_it+1) == measurement.first.first.end()){
                    ROS_DEBUG_STREAM("IMU frame end time, acc: " << t << "\t" << (*imu_it)->linear_acceleration.x << "\t" << (*imu_it)->linear_acceleration.y << "\t" << (*imu_it)->linear_acceleration.z);
                }

                double img_t = img_msg->header.stamp.toSec() + estimator.td;
                if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;


                    double dt = t - current_time; //this imu mes time - last imu mes time i.e. sampling time
                    ROS_ASSERT(dt >= 0);
                    current_time = t; // update last imu mes time to be equal to this imu mes time for next iter
                    

                    dx = (*imu_it)->linear_acceleration.x;
                    dy = (*imu_it)->linear_acceleration.y;
                    dz = (*imu_it)->linear_acceleration.z;
                    rx = (*imu_it)->angular_velocity.x;
                    ry = (*imu_it)->angular_velocity.y;
                    rz = (*imu_it)->angular_velocity.z;


                    if (control_it!=measurement.first.second.end() && (*control_it)->header.stamp.toSec() <= current_time)
                    {
                        
                        //printf("New control_ts: %f control_thrust: %f \n", (*control_it)->header.stamp.toSec(), (*control_it)->collective_thrust);
                        Fz = (*control_it)->collective_thrust;
                        ++control_it;
                    }

                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz), Fz, Vector3d(Tx, Ty, Tz), img_msg->header);

                    //printf("imu_ts: %f imu: dt:%f acc_x: %f Fz: %f \n", t, dt, dx, Fz);


                }
                else // last imu meas whose time stamp is > img.timestamp + td 
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * (*imu_it)->linear_acceleration.x;
                    dy = w1 * dy + w2 * (*imu_it)->linear_acceleration.y;
                    dz = w1 * dz + w2 * (*imu_it)->linear_acceleration.z;
                    rx = w1 * rx + w2 * (*imu_it)->angular_velocity.x;
                    ry = w1 * ry + w2 * (*imu_it)->angular_velocity.y;
                    rz = w1 * rz + w2 * (*imu_it)->angular_velocity.z;

                    if (control_it!=measurement.first.second.end() && (*control_it)->header.stamp.toSec() < img_t)
                    {
                        
                        //printf("Here New control_ts: %f control_thrust: %f \n", (*control_it)->header.stamp.toSec(), (*control_it)->collective_thrust);
                        Fz = (*control_it)->collective_thrust;
                        ++control_it;
                    }

                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz), Fz, Vector3d(Tx, Ty, Tz), img_msg->header);
                    //printf("Here!! imu_ts: %f dt:%f Fz: %f \n",t, dt_1, Fz);
                }

                last_Fz = Fz;
            }
            // set relocalization frame
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)
            {
                vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image; // feature id, vector<camera id, xyz_uv_vel>
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM; //v bcz num of cam =1
                int camera_id = v % NUM_OF_CAM; // 0 bcz num of cam =1 
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            estimator.processImage(image, img_msg->header);

            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            if (relo_msg != NULL)
                pubRelocalization(estimator);
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info); //Debug Info
    readParameters(n);
    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback1, ros::TransportHints().tcpNoDelay()); // 2000 msgs queue size
    ros::Subscriber sub_control_inputs = n.subscribe(CONTROL_TOPIC, 2000, control_inputs_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);

    ros::Subscriber sub_ground_truth = n.subscribe("/hummingbird/ground_truth/odometry", 2000, groundtruth_callback);

    //std::thread measurement_process{VIO_process};
    std::thread measurement_process{VIO_MODEL_process};
    ros::spin();

    return 0;
}

/*

void imu_callback0(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();
   
    //ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header); // estimated pose and vel from last frame + predicted through imu meas if in between time frames
    }

}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements0()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
    
    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty() )
            return measurements;

        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop(); //pops front
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        
        measurements.emplace_back(IMUs, img_msg);
        printf("IMUs and control front, back and img timestamp: %f, %f, %f \n", IMUs.front()->header.stamp.toSec(), IMUs.back()->header.stamp.toSec(), img_msg->header.stamp.toSec());
    
    }
    return measurements; //never reached?
}

// thread: visual-inertial odometry
void VIO_process()
{
    while (true)
    {
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        //std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>,std::vector<quadrotor_msgs::ControlCommand::ConstPtr>>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements0()).size() != 0;
                 });
        lk.unlock();
        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0, Fz = 0, Tx = 0, Ty = 0, Tz = 0; //imu meas container
            std::vector<sensor_msgs::ImuConstPtr>::iterator imu_it = measurement.first.begin(); 
            //double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0, Fz = 0, Tx = 0, Ty = 0, Tz = 0; //imu meas container
            //std::vector<sensor_msgs::ImuConstPtr>::iterator imu_it = measurement.first.first.begin(); 
            //std::vector<quadrotor_msgs::ControlCommand::ConstPtr>::iterator control_it = measurement.first.second.begin();
            
            
            for (; imu_it!=measurement.first.end(); ++imu_it)//for (auto &imu_msg : measurement.first.first)
            {
                double t = (*imu_it)->header.stamp.toSec();
                
                if (imu_it == measurement.first.begin()){
                    ROS_DEBUG_STREAM("IMU begin time, acc: "<< t << "\t" << (*imu_it)->linear_acceleration.x << "\t" << (*imu_it)->linear_acceleration.y << "\t" << (*imu_it)->linear_acceleration.z);
                }

                if ((imu_it+1) == measurement.first.end()){
                    ROS_DEBUG_STREAM("IMU frame end time, acc: " << t << "\t" << (*imu_it)->linear_acceleration.x << "\t" << (*imu_it)->linear_acceleration.y << "\t" << (*imu_it)->linear_acceleration.z);
                }

                double img_t = img_msg->header.stamp.toSec() + estimator.td;
                if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time; //this imu mes time - last imu mes time i.e. sampling time
                    ROS_ASSERT(dt >= 0);
                    current_time = t; // update last imu mes time to be equal to this imu mes time for next iter
                    dx = (*imu_it)->linear_acceleration.x;
                    dy = (*imu_it)->linear_acceleration.y;
                    dz = (*imu_it)->linear_acceleration.z;
                    rx = (*imu_it)->angular_velocity.x;
                    ry = (*imu_it)->angular_velocity.y;
                    rz = (*imu_it)->angular_velocity.z;

                    // Fz = (*control_it)->thrust.z;
                    // Tx = (*control_it)->torque.x;
                    // Ty = (*control_it)->torque.y;
                    // Tz = (*control_it)->torque.z;
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz), Fz, Vector3d(Tx, Ty, Tz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                }
                else // last imu meas whose time stamp is > img.timestamp + td 
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * (*imu_it)->linear_acceleration.x;
                    dy = w1 * dy + w2 * (*imu_it)->linear_acceleration.y;
                    dz = w1 * dz + w2 * (*imu_it)->linear_acceleration.z;
                    rx = w1 * rx + w2 * (*imu_it)->angular_velocity.x;
                    ry = w1 * ry + w2 * (*imu_it)->angular_velocity.y;
                    rz = w1 * rz + w2 * (*imu_it)->angular_velocity.z;

                    // Fz = w1 * Fz + w2 * (*control_it)->thrust.z;
                    // Tx = w1 * Tx + w2 * (*control_it)->torque.x;
                    // Ty = w1 * Ty + w2 * (*control_it)->torque.y;
                    // Tz = w1 * Tz + w2 * (*control_it)->torque.z;

                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz), Fz, Vector3d(Tx, Ty, Tz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }
            // set relocalization frame
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)
            {
                vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image; // feature id, vector<camera id, xyz_uv_vel>
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM; //v bcz num of cam =1
                int camera_id = v % NUM_OF_CAM; // 0 bcz num of cam =1 
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            estimator.processImage(image, img_msg->header);

            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            if (relo_msg != NULL)
                pubRelocalization(estimator);
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}*/