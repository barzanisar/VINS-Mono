#include "estimator.h"

Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    clearState();
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }
    f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
}

void Estimator::clearState()
{
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity(); //R_wb initial is set to identity
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        Fexts[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();
        Fz_buf[i].clear();
        

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++) // remove loop cz only mono cam
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    solver_flag = INITIAL;
    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    initial_timestamp = 0;
    all_image_frame.clear();
    td = TD;


    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;
    relocalization_info = 0;

    drift_correct_r = Matrix3d::Identity();
    drift_correct_t = Vector3d::Zero();
}

void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity, double Fz, const std_msgs::Header &imgheader) // Fz is actually thrust in the body x axis which points upwards opposite gravity when quad is at hover
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
        Fz_0 = Fz;
    }

    if (!pre_integrations[frame_count]) // if preinteg at the frame count is nullptr i.e. preinteg has not happened yet 
    {
        //ROS_DEBUG_STREAM(" frame_count: " << frame_count << " am_0 " << acc_0.transpose());
        //ROS_DEBUG_STREAM(" Bas: " << Bas[frame_count].transpose());
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Fz_0, Bas[frame_count], Bgs[frame_count]}; 
    }
    if (frame_count != 0)
    {
        /*if (frame_count< 3){
            ROS_DEBUG_STREAM(" frame_count: " << frame_count << " am_1 " << linear_acceleration.transpose());
        }*/
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity, Fz);

        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity, Fz);
            


        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);
        Fz_buf[frame_count].push_back(Fz);
        

        int j = frame_count;         // initial guess propagation // should we average it with control inputs???, If yes, then uncomment 2 lines below
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g; 
        //Vector3d body_thrust_0(0.0, 0.0, Fz_0);
        //Vector3d control_acc_0 = Rs[j] * body_thrust_0 + Fexts[j]/MASS - g;
        ROS_DEBUG_STREAM_THROTTLE(0.2," Gravity " << g.transpose());
        //ROS_DEBUG_STREAM_ONCE(" Rs[j] " << Rs[j]);
        //ROS_DEBUG_STREAM_ONCE(" Rs[j] right col " << Rs[j].rightCols<1>());
        Vector3d control_acc_0 = Rs[j].rightCols<1>() * Fz_0 + Fexts[j] - g; // Fz_0 is the collective motor thrust in m/s2
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        //Vector3d body_thrust_1(0.0, 0.0, Fz);
        //Vector3d control_acc_1 = Rs[j] * body_thrust_1 + Fexts[j]/MASS - g;
        Vector3d control_acc_1 = Rs[j].rightCols<1>() * Fz + Fexts[j] - g; // assume Fexts as acc_exts
        //un_acc_0 = 0.5 * (un_acc_0 + control_acc_0);
        //un_acc_1 = 0.5 * (un_acc_1 + control_acc_1);
        //Vector3d un_acc = 0.5 * (control_acc_0 + control_acc_1);
        /*if (un_acc != (0.5 * (un_acc_0 + un_acc_1)))
        {
            ROS_DEBUG_STREAM_THROTTLE(0.2," Different un_acc_0 " << un_acc_0.transpose());
            ROS_DEBUG_STREAM_THROTTLE(0.2," un_acc_1 " << un_acc_1.transpose());
            ROS_DEBUG_STREAM_THROTTLE(0.2," control_acc_0 " << control_acc_0.transpose());
            ROS_DEBUG_STREAM_THROTTLE(0.2," control_acc_1 " << control_acc_1.transpose());
        }*/
    
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
    Fz_0 = Fz;
    
}


void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header)
{
    // write result to file
    if (APPLY_MODEL_PREINTEGRATION)
    {
        ofstream foutC(PREINTEG_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(0);
        foutC << header.stamp.toSec() * 1e9 << "," << frame_count << ",";
        foutC.precision(5);
        foutC << pre_integrations[frame_count]->delta_p.x() << "," //3
              << pre_integrations[frame_count]->delta_p.y() << ","
              << pre_integrations[frame_count]->delta_p.z() << ","
              << pre_integrations[frame_count]->delta_p_model.x() << "," //6
              << pre_integrations[frame_count]->delta_p_model.y() << ","
              << pre_integrations[frame_count]->delta_p_model.z() << ","
              << pre_integrations[frame_count]->delta_v.x() << "," //9
              << pre_integrations[frame_count]->delta_v.y() << ","
              << pre_integrations[frame_count]->delta_v.z() << ","
              << pre_integrations[frame_count]->delta_v_model.x() << "," //12
              << pre_integrations[frame_count]->delta_v_model.y() << ","
              << pre_integrations[frame_count]->delta_v_model.z() << ","
              << pre_integrations[frame_count]->delta_q.w() << "," //15
              << pre_integrations[frame_count]->delta_q.x() << "," 
              << pre_integrations[frame_count]->delta_q.y() << ","
              << pre_integrations[frame_count]->delta_q.z() << endl;
        foutC.close();
    }

    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size()); // number of features visible in image i.e. no. of uv points
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;

    ROS_DEBUG("this frame is--------------------%s timestamp: %f", marginalization_flag ? "reject" : "accept" , header.stamp.toSec());
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header.stamp.toSec());
    imageframe.pre_integration = tmp_pre_integration;// contains imu meas between previous image and this image
    all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Fz_0, Bas[frame_count], Bgs[frame_count]};


    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)
        {
            bool result = false;
            if( ESTIMATE_EXTRINSIC != 2 && (header.stamp.toSec() - initial_timestamp) > 0.1)
            {
               result = initialStructure();
               initial_timestamp = header.stamp.toSec();
            }
            if(result)
            {
                solver_flag = NON_LINEAR;
                solveOdometry();
                slideWindow();
                f_manager.removeFailures();
                ROS_INFO("Initialization finish!");
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];
                
            }
            else
                slideWindow();
        }
        else
            frame_count++;
    }
    else
    {
        TicToc t_solve;
        solveOdometry();
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        TicToc t_margin;
        slideWindow();
        f_manager.removeFailures();
        ROS_DEBUG("marginalization costs: %fms", t_margin.toc()); //Sliding window costs
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}
bool Estimator::initialStructure()
{
    TicToc t_sfm;
    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            //return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    } 
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    GlobalSFM sfm;
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i].stamp.toSec())
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i].stamp.toSec())
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
    {   ROS_INFO("Visual Inertial align is true!");
        return true;
    }
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }

}

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);


    if(!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i].stamp.toSec()].R;
        Vector3d Pi = all_image_frame[Headers[i].stamp.toSec()].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i].stamp.toSec()].is_key_frame = true;
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);

    //triangulat on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for(int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();
    ric[0] = RIC[0];
    f_manager.setRic(ric);
    f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_INFO_STREAM("g0     " << g.transpose());
    ROS_INFO_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose()); 

    return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::solveOdometry()
{
    if (frame_count < WINDOW_SIZE)
        return;
    if (solver_flag == NON_LINEAR)
    {
        TicToc t_tri;
        f_manager.triangulate(Ps, tic, ric);
        ROS_DEBUG("triangulation costs %f", t_tri.toc());

        optimization();
    }
}

void Estimator::vector2double(bool print_debug) // put estimates or initial guess into parameters
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Position[i][0] = Ps[i].x();
        para_Position[i][1] = Ps[i].y();
        para_Position[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Attitude[i][0] = q.x(); // para_Pose[i][3] , 4, 5, 6
        para_Attitude[i][1] = q.y();
        para_Attitude[i][2] = q.z();
        para_Attitude[i][3] = q.w();

        para_Speed[i][0] = Vs[i].x();
        para_Speed[i][1] = Vs[i].y();
        para_Speed[i][2] = Vs[i].z();

        para_Bias[i][0] = Bas[i].x();
        para_Bias[i][1] = Bas[i].y();
        para_Bias[i][2] = Bas[i].z();

        para_Bias[i][3] = Bgs[i].x();
        para_Bias[i][4] = Bgs[i].y();
        para_Bias[i][5] = Bgs[i].z();

        para_Fext[i][0] = Fexts[i].x();
        para_Fext[i][1] = Fexts[i].y();
        para_Fext[i][2] = Fexts[i].z();  
        
    }

    if (print_debug)
    {
        //ROS_DEBUG_STREAM("frame: "<< i <<" para_Fext_initial: "<<Fexts[i].transpose());
        ROS_DEBUG_STREAM(" para_position_initial: "<<Ps[WINDOW_SIZE].transpose());
        ROS_DEBUG_STREAM(" para_bias_accel_initial: "<<Bas[WINDOW_SIZE].transpose());
    }  

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Position[i][0] = tic[i].x();
        para_Ex_Position[i][1] = tic[i].y();
        para_Ex_Position[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Attitude[i][0] = q.x();
        para_Ex_Attitude[i][1] = q.y();
        para_Ex_Attitude[i][2] = q.z();
        para_Ex_Attitude[i][3] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);
    if (ESTIMATE_TD)
        para_Td[0][0] = td;
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]); // initial guess for Rs[0]' yaw pitch roll' i.e. from prev optimi
    Vector3d origin_P0 = Ps[0]; // initial guess Ps[0] i.e. from prev optimi

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Attitude[0][3], // optimized Rs[0]
                                                      para_Attitude[0][0],
                                                      para_Attitude[0][1],
                                                      para_Attitude[0][2]).toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x(); // yaw diff between initial guess/previous optimization result and current optimiresult for pose at 0
    //TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
    {
        ROS_DEBUG("euler singular point!");
        rot_diff = Rs[0] * Quaterniond(para_Attitude[0][3],
                                       para_Attitude[0][0],
                                       para_Attitude[0][1],
                                       para_Attitude[0][2]).toRotationMatrix().transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++) //updating initial guess with estimated states/optimization results
    {

        Rs[i] = rot_diff * Quaterniond(para_Attitude[i][3], para_Attitude[i][0], para_Attitude[i][1], para_Attitude[i][2]).normalized().toRotationMatrix();
        
        //ROS_INFO_STREAM_ONCE("Initial R_wb0: " << Rs[0]);
        //ROS_INFO_STREAM_ONCE("Initial R_wb1: " << Rs[1]);
        //ROS_INFO_STREAM_ONCE("Initial R_wb_window_size: " << Rs[WINDOW_SIZE]);

        Ps[i] = rot_diff * Vector3d(para_Position[i][0] - para_Position[0][0],
                                para_Position[i][1] - para_Position[0][1],
                                para_Position[i][2] - para_Position[0][2]) + origin_P0;

        Vs[i] = rot_diff * Vector3d(para_Speed[i][0],
                                    para_Speed[i][1],
                                    para_Speed[i][2]);

        Bas[i] = Vector3d(para_Bias[i][0],
                          para_Bias[i][1],
                          para_Bias[i][2]);

        Bgs[i] = Vector3d(para_Bias[i][3],
                          para_Bias[i][4],
                          para_Bias[i][5]);

        Fexts[i] = Vector3d(para_Fext[i][0],
                            para_Fext[i][1],
                            para_Fext[i][2]);
        if (i == WINDOW_SIZE)
            Fexts[WINDOW_SIZE] = Fexts[WINDOW_SIZE-1];
        
        //ROS_DEBUG_STREAM("frame: "<< i <<" para_Fext_estimated: "<<Fexts[i].transpose());
    }

    ROS_DEBUG_STREAM("position_estimate: "<<Ps[WINDOW_SIZE].transpose());
    ROS_DEBUG_STREAM("bias_accel_estimate: "<<Bas[WINDOW_SIZE].transpose());

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d(para_Ex_Position[i][0],
                          para_Ex_Position[i][1],
                          para_Ex_Position[i][2]);
        ric[i] = Quaterniond(para_Ex_Attitude[i][3],
                             para_Ex_Attitude[i][0],
                             para_Ex_Attitude[i][1],
                             para_Ex_Attitude[i][2]).toRotationMatrix();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);
    if (ESTIMATE_TD)
        td = para_Td[0][0];

    // relative info between two loop frame
    if(relocalization_info)
    { 
        Matrix3d relo_r;
        Vector3d relo_t;
        relo_r = rot_diff * Quaterniond(relo_Attitude[3], relo_Attitude[0], relo_Attitude[1], relo_Attitude[2]).normalized().toRotationMatrix();
        relo_t = rot_diff * Vector3d(relo_Position[0] - para_Position[0][0],
                                     relo_Position[1] - para_Position[0][1],
                                     relo_Position[2] - para_Position[0][2]) + origin_P0;
        double drift_correct_yaw;
        drift_correct_yaw = Utility::R2ypr(prev_relo_r).x() - Utility::R2ypr(relo_r).x();
        drift_correct_r = Utility::ypr2R(Vector3d(drift_correct_yaw, 0, 0));
        drift_correct_t = prev_relo_t - drift_correct_r * relo_t;   
        relo_relative_t = relo_r.transpose() * (Ps[relo_frame_local_index] - relo_t);
        relo_relative_q = relo_r.transpose() * Rs[relo_frame_local_index];
        relo_relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs[relo_frame_local_index]).x() - Utility::R2ypr(relo_r).x());
        //cout << "vins relo " << endl;
        //cout << "vins relative_t " << relo_relative_t.transpose() << endl;
        //cout << "vins relative_yaw " <<relo_relative_yaw << endl;
        relocalization_info = 0;    

    }
}

bool Estimator::failureDetection()
{
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        //return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true; //
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true; //
    }
    if (Fexts[WINDOW_SIZE].norm() > 25)
    {
        ROS_INFO(" big Fext estimation %f", Fexts[WINDOW_SIZE].norm());
        //return true; 
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        ROS_INFO(" big translation");
        return true; //
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        ROS_INFO(" big z translation");
        return true; //
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        //return true;
    }
    return false;
}


void Estimator::optimization()
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Position[i], SIZE_POSITION);
        problem.AddParameterBlock(para_Attitude[i], SIZE_ATTITUDE, local_parameterization);
        problem.AddParameterBlock(para_Speed[i], SIZE_SPEED);
        problem.AddParameterBlock(para_Bias[i], SIZE_BIAS);
        if (i!=WINDOW_SIZE && APPLY_MODEL_PREINTEGRATION)
            problem.AddParameterBlock(para_Fext[i], SIZE_FORCES);
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Position[i], SIZE_POSITION);
        problem.AddParameterBlock(para_Ex_Attitude[i], SIZE_ATTITUDE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC)
        {
            ROS_DEBUG("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Position[i]);
            problem.SetParameterBlockConstant(para_Ex_Attitude[i]);
        }
        else
            ROS_DEBUG("estimate extinsic param");
    }
    if (ESTIMATE_TD)
    {
        problem.AddParameterBlock(para_Td[0], 1);
        //problem.SetParameterBlockConstant(para_Td[0]);
    }

    TicToc t_whole, t_prepare;

    //Fexts[WINDOW_SIZE-1].setZero();
    //Fexts[WINDOW_SIZE].setZero();
    Fexts[WINDOW_SIZE] = Fexts[WINDOW_SIZE-1];
    vector2double(true); // set initial guess to optimization parameters
    

    if (last_marginalization_info) // if is not nullptr i.e. prior exists from previous optimization then add its prior residual
    {
        // construct new marginlization_factor
        ROS_DEBUG_STREAM_ONCE("set marginalization factor!");
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }

    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]); // starts with preinteg[1] and para_pose 0,1 and ends adding preinteg[10]
        problem.AddResidualBlock(imu_factor, NULL, para_Position[i], para_Attitude[i], para_Speed[i], para_Bias[i], para_Position[j], para_Attitude[j], para_Speed[j], para_Bias[j]);
        
        if(APPLY_MODEL_PREINTEGRATION)
        {
            ModelFactor* model_factor = new ModelFactor(pre_integrations[j]); // starts with preinteg[1] and para_pose 0,1 and ends adding preinteg[10]
            //ceres::CostFunction* model_factor = ModelFactor::Create(pre_integrations[j]);
            problem.AddResidualBlock(model_factor, NULL, para_Position[i], para_Attitude[i], para_Speed[i], para_Fext[i], para_Position[j], para_Speed[j]); //we want model_factor, NULL, para_Pose[i], para_Speed[i], para_Fext[i], para_Position[j], para_Speed[j]   
        } 
          
    }
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature) // for each landmark we have seen
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2)) //ignore estimating those feature's depth which only have one measurement in the window or are very recently added
            continue;
 
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point; // this feature's start frame's meas point

        for (auto &it_per_frame : it_per_id.feature_per_frame) // for each measurement of the accepted landmarks seen
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;
            if (ESTIMATE_TD)
            {
                    ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                     it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                     it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                    problem.AddResidualBlock(f_td, loss_function, para_Position[imu_i], para_Attitude[imu_i], para_Position[imu_j], para_Attitude[imu_j], para_Ex_Position[0], para_Ex_Attitude[0], para_Feature[feature_index], para_Td[0]);

            }
            else
            {
                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j); //start frame meas for a feature and the next frame's meas
                problem.AddResidualBlock(f, loss_function, para_Position[imu_i], para_Attitude[imu_i], para_Position[imu_j], para_Attitude[imu_j], para_Ex_Position[0], para_Ex_Attitude[0], para_Feature[feature_index]);
            }
            f_m_cnt++; // number of landmark residuals i.e. getFeatureCount * (feature_pre_frame.size - 1) i.e. total number of landmarks accepted in the window * (their total measurements - 1 ) i.e. -start frame meas)  
        }
    }

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    if(relocalization_info)
    {
        ROS_DEBUG_STREAM_ONCE("set relocalization factor!");
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(relo_Position, SIZE_POSITION);
        problem.AddParameterBlock(relo_Attitude, SIZE_ATTITUDE, local_parameterization);
        int retrive_feature_index = 0;
        int feature_index = -1;
        for (auto &it_per_id : f_manager.feature)
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;
            ++feature_index;
            int start = it_per_id.start_frame;
            if(start <= relo_frame_local_index)
            {   
                while((int)match_points[retrive_feature_index].z() < it_per_id.feature_id)
                {
                    retrive_feature_index++;
                }
                if((int)match_points[retrive_feature_index].z() == it_per_id.feature_id)
                {
                    Vector3d pts_j = Vector3d(match_points[retrive_feature_index].x(), match_points[retrive_feature_index].y(), 1.0);
                    Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                    
                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                    problem.AddResidualBlock(f, loss_function, para_Position[start], para_Attitude[start], relo_Position, relo_Attitude, para_Ex_Position[0], para_Ex_Attitude[0], para_Feature[feature_index]);
                    retrive_feature_index++;
                }     
            }
        }

    }

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //cout << summary.BriefReport() << endl;
    num_iters = static_cast<int>(summary.iterations.size());
    solve_time_ms = t_solver.toc();
    ROS_DEBUG("Iterations : %d", num_iters);
    ROS_DEBUG("solver costs: %f", solve_time_ms);


    double2vector(); //updates all states with their new estimates



    /*PREPARE marginalization info to be added as prior in next window*/

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD) // if last frame in the current window was a keyframe i.e. frame 10
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double(false);

        if (last_marginalization_info) // if we had prior in this current window then add this prior info into the prior for next frame as well
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Position[0] ||
                    last_marginalization_parameter_blocks[i] == para_Attitude[0] ||
                    last_marginalization_parameter_blocks[i] == para_Speed[0] || 
                    last_marginalization_parameter_blocks[i] == para_Bias[0]) 
                    drop_set.push_back(i);

                if(APPLY_MODEL_PREINTEGRATION)
                {
                     if (last_marginalization_parameter_blocks[i] == para_Fext[0])
                        drop_set.push_back(i);
                }
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Position[0], para_Attitude[0], para_Speed[0], para_Bias[0], para_Position[1], para_Attitude[1], para_Speed[1], para_Bias[1]},
                                                                           vector<int>{0, 1, 2, 3}); // index of parameter blocks to drop, i.e. drop para_Pose[0], para_SpeedBias[0]
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            if(APPLY_MODEL_PREINTEGRATION)
            {
                if (pre_integrations[1]->sum_dt < 10.0)
                {
                    ModelFactor* model_factor = new ModelFactor(pre_integrations[1]);
                    //ceres::CostFunction* model_factor = ModelFactor::Create(pre_integrations[1]);
     
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(model_factor, NULL,
                                                                               vector<double *>{para_Position[0], para_Attitude[0], para_Speed[0], para_Fext[0], para_Position[1], para_Speed[1]},
                                                                               vector<int>{0, 1, 2, 3}); // {0, 1, 2, 3} index of parameter blocks to drop, i.e. drop para_Pose[0], para_SpeedBias[0], para_Fext[0]
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }
            }
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0) // the feature must be visible from the start frame i.e. frame 0 that we are marginalizing
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point; // start frame landmark measurement

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    Vector3d pts_j = it_per_frame.point;
                    if (ESTIMATE_TD)
                    {
                        ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                          it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{para_Position[imu_i], para_Attitude[imu_i], para_Position[imu_j], para_Attitude[imu_j], para_Ex_Position[0], para_Ex_Attitude[0], para_Feature[feature_index], para_Td[0]},
                                                                                        vector<int>{0, 1, 6});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    else
                    {
                        ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j); // pts_i: start frame landmark meas, pts_j: other frames landmark meas
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                       vector<double *>{para_Position[imu_i], para_Attitude[imu_i], para_Position[imu_j], para_Attitude[imu_j], para_Ex_Position[0], para_Ex_Attitude[0], para_Feature[feature_index]},
                                                                                       vector<int>{0, 1, 6}); //para_Pose[imu_i] and para_Feature[feature_index] marginalized so we marg start pose of the window and all the features which are visible in frame 0
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Position[i])] = para_Position[i - 1];
            addr_shift[reinterpret_cast<long>(para_Attitude[i])] = para_Attitude[i - 1];
            addr_shift[reinterpret_cast<long>(para_Speed[i])] = para_Speed[i - 1];
            addr_shift[reinterpret_cast<long>(para_Bias[i])] = para_Bias[i - 1];
            if (i!=WINDOW_SIZE && APPLY_MODEL_PREINTEGRATION)
                addr_shift[reinterpret_cast<long>(para_Fext[i])] = para_Fext[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
        {   
            addr_shift[reinterpret_cast<long>(para_Ex_Position[i])] = para_Ex_Position[i];
            addr_shift[reinterpret_cast<long>(para_Ex_Attitude[i])] = para_Ex_Attitude[i];
        }
        if (ESTIMATE_TD)
        {
            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
        }
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        
    }
    else
    {
        if (last_marginalization_info &&
            (std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Position[WINDOW_SIZE - 1]) ||
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Attitude[WINDOW_SIZE - 1])))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double(false);
            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_Speed[WINDOW_SIZE - 1]);
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_Bias[WINDOW_SIZE - 1]);
                    //ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_Fext[WINDOW_SIZE - 2]); //why need this?
                    if (last_marginalization_parameter_blocks[i] == para_Position[WINDOW_SIZE - 1] ||
                        last_marginalization_parameter_blocks[i] == para_Attitude[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Position[i])] = para_Position[i - 1];
                    addr_shift[reinterpret_cast<long>(para_Attitude[i])] = para_Attitude[i - 1];
                    addr_shift[reinterpret_cast<long>(para_Speed[i])] = para_Speed[i - 1];
                    addr_shift[reinterpret_cast<long>(para_Bias[i])] = para_Bias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Position[i])] = para_Position[i];
                    addr_shift[reinterpret_cast<long>(para_Attitude[i])] = para_Attitude[i];
                    addr_shift[reinterpret_cast<long>(para_Speed[i])] = para_Speed[i];
                    addr_shift[reinterpret_cast<long>(para_Bias[i])] = para_Bias[i];
                    if (APPLY_MODEL_PREINTEGRATION)
                        addr_shift[reinterpret_cast<long>(para_Fext[i])] = para_Fext[i];
                }
            }
            /*for (int i = 0; APPLY_MODEL_PREINTEGRATION && i <= WINDOW_SIZE-1; i++) //either uncomment this or above
            {
                if (i == WINDOW_SIZE - 2)
                    continue;
                else if (i == WINDOW_SIZE-1)
                {
                    addr_shift[reinterpret_cast<long>(para_Fext[i])] = para_Fext[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Fext[i])] = para_Fext[i];
                }
            }*/
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                addr_shift[reinterpret_cast<long>(para_Ex_Position[i])] = para_Ex_Position[i];
                addr_shift[reinterpret_cast<long>(para_Ex_Attitude[i])] = para_Ex_Attitude[i];
            }
            if (ESTIMATE_TD)
            {
                addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
            }
            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
    ROS_DEBUG("whole marginalization costs: %f", t_whole_marginalization.toc());
    
    ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Rs[i].swap(Rs[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);
                Fz_buf[i].swap(Fz_buf[i + 1]);
                

                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
                Fexts[i].swap(Fexts[i + 1]);
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];
            Fexts[WINDOW_SIZE] = Fexts[WINDOW_SIZE - 1];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Fz_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};
            

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();
            Fz_buf[WINDOW_SIZE].clear();
            

            if (true || solver_flag == INITIAL)
            {
                double t_0 = Headers[0].stamp.toSec();
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);

            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];
                double tmp_Fz = Fz_buf[frame_count][i];
                

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity, tmp_Fz);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                Fz_buf[frame_count - 1].push_back(tmp_Fz);
                
            }

            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];
            Fexts[frame_count - 1] = Fexts[frame_count];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Fz_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();
            Fz_buf[WINDOW_SIZE].clear();
            

            slideWindowNew();
        }
    }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}
// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}

void Estimator::setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r)
{
    relo_frame_stamp = _frame_stamp;
    relo_frame_index = _frame_index;
    match_points.clear();
    match_points = _match_points;
    prev_relo_t = _relo_t;
    prev_relo_r = _relo_r;
    for(int i = 0; i < WINDOW_SIZE; i++)
    {
        if(relo_frame_stamp == Headers[i].stamp.toSec())
        {
            relo_frame_local_index = i;
            relocalization_info = 1;
            for (int j = 0; j < SIZE_POSITION; j++)
                relo_Position[j] = para_Position[i][j];
            for (int j = 0; j < SIZE_ATTITUDE; j++)
                relo_Attitude[j] = para_Attitude[i][j];
        }
    }
}

