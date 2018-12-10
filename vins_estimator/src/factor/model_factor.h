#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>

class ModelFactor : public ceres::SizedCostFunction<12, 3, 4, 3, 3, 3, 3, 3> //position i, attitude i,  speed i, fext i, position j, speed j, fext j
{
  public:
    ModelFactor() = delete;
    ModelFactor(IntegrationBase* _pre_integration):pre_integration(_pre_integration)
    {
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {

        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[1][3], parameters[1][0], parameters[1][1], parameters[1][2]);

        Eigen::Vector3d Vi(parameters[2][0], parameters[2][1], parameters[2][2]);

        Eigen::Vector3d Fexti(parameters[3][0], parameters[3][1], parameters[3][2]);

        Eigen::Vector3d Pj(parameters[4][0], parameters[4][1], parameters[4][2]);

        Eigen::Vector3d Vj(parameters[5][0], parameters[5][1], parameters[5][2]);

        Eigen::Vector3d Fextj(parameters[6][0], parameters[6][1], parameters[6][2]);


        Eigen::Map<Eigen::Matrix<double, 12, 1>> residual(residuals);
        residual = pre_integration->evaluate_model(Pi, Qi, Vi, Fexti,
                                            Pj, Vj, Fextj);
        Eigen::Matrix<double, 12, 1> residual_before = residual;


        //Eigen::Matrix<double, 12, 12> sqrt_info_full = Eigen::LLT<Eigen::Matrix<double, 12, 12>>(pre_integration->covariance_model.inverse()).matrixL().transpose();
        //Eigen::Matrix<double, 9, 9> sqrt_info_full = Eigen::LLT<Eigen::Matrix<double, 9, 9>>(pre_integration->covariance_model.inverse()).matrixL().transpose();
        
        Eigen::Matrix<double, 12, 12> sqrt_info; // for just p and v
/*        sqrt_info.setIdentity();
        sqrt_info.block<3,3>(0,0) = sqrt_info_full.block<3,3>(O_P,O_P);
        sqrt_info.block<3,3>(0,3) = sqrt_info_full.block<3,3>(O_P,O_V);
        sqrt_info.block<3,3>(3,0) = sqrt_info_full.block<3,3>(O_V,O_P);
        sqrt_info.block<3,3>(3,3) = sqrt_info_full.block<3,3>(O_V,O_V);*/

        //Eigen::Matrix<double, 6, 6> tmp_sqrt_info_pv = Eigen::LLT<Eigen::Matrix<double, 6, 6>>(pre_integration->covariance_model_pv.inverse()).matrixL().transpose();
        //sqrt_info.setIdentity();

        //sqrt_info.block<3,3>(0,0) = tmp_sqrt_info_pv.block<3,3>(0,0);
        //sqrt_info.block<3,3>(3,3) = tmp_sqrt_info_pv.block<3,3>(3,3);
        Eigen::Matrix<double, 12, 12> tmp_big_covar; //this is actually the weight matrix
        tmp_big_covar.setZero();
        tmp_big_covar.block<6, 6>(0,0) = pre_integration->covariance_model_pv.inverse();// + 1e-11 * Eigen::MatrixXd::Identity(6,6);
        //Eigen::Matrix<double, 6, 6> tmp_pv_covar = tmp_big_covar.block<6,6>(0,0);
        //tmp_big_covar.block<6, 6>(0,0) = tmp_pv_covar.inverse();
        tmp_big_covar.block<3, 3>(6,6) = 1.0 / (pre_integration->sum_dt * pre_integration->sum_dt * F_EXT_W * F_EXT_W )* Eigen::Matrix3d::Identity();
        //tmp_big_covar.block<3, 3>(6,6) = pre_integration->sum_dt * pre_integration->sum_dt * F_EXT_W * F_EXT_W * Eigen::Matrix3d::Identity();
        
        tmp_big_covar.block<3, 3>(9,9) = F_EXT_NORM_WEIGHT * Eigen::Matrix3d::Identity();

        //Eigen::Matrix<double, 6, 6> sqrt_info_old = Eigen::LLT<Eigen::Matrix<double, 6, 6>>(pre_integration->covariance_model_pv.inverse()).matrixL().transpose();
        //Eigen::Matrix<double, 6, 6> sqrt_info_new = Eigen::LLT<Eigen::Matrix<double, 6, 6>>(inv_small_covar).matrixL().transpose();
        sqrt_info = Eigen::LLT<Eigen::Matrix<double, 12, 12>>(tmp_big_covar).matrixL().transpose();

        //Eigen::Matrix<double, 6, 6> L= Eigen::LLT<Eigen::Matrix<double, 6, 6>>(pre_integration->covariance_model_pv.inverse()).matrixL();


        
        ROS_INFO_STREAM_ONCE("Model sqrt_info : ");
        ROS_INFO_STREAM_ONCE(sqrt_info);
        ROS_INFO_STREAM_ONCE("Model covariance_model inverse i.e. weight or info : ") ;
        ROS_INFO_STREAM_ONCE(tmp_big_covar);

        //ROS_INFO_STREAM_ONCE("Model covariance_model + Identity: ") ;
        //ROS_INFO_STREAM_ONCE(pre_integration->covariance_model_pv + Eigen::MatrixXd::Identity(6,6));

        //ROS_INFO_STREAM_ONCE("Model inv covariance_model :" );
        //ROS_INFO_STREAM_ONCE(pre_integration->covariance_model_pv.inverse());
        //ROS_INFO_STREAM_ONCE("Should be covar_inv :");
        //ROS_INFO_STREAM_ONCE(L * L.transpose());


        //ROS_INFO_STREAM_ONCE("Model C*inv(C) :" << pre_integration->covariance_model_pv * pre_integration->covariance_model_pv.inverse());
       // ROS_INFO_STREAM_ONCE("Model new inv covariance_model :" << inv_small_covar);
        //ROS_INFO_STREAM_ONCE("Model sqrt_info :" << sqrt_info_old);
        //ROS_INFO_STREAM_ONCE("Model new sqrt_info :" << sqrt_info);

        ROS_DEBUG_STREAM_ONCE("Model residual before:" << residual);

        residual = sqrt_info * residual;

        ROS_DEBUG_STREAM_ONCE("Model residual after:" << residual);

        if (jacobians)
        {
            double sum_dt = pre_integration->sum_dt;

            if (jacobians[0]) // derivative of residual wrt the first parameter block i.e. 3D position_i
            {
                Eigen::Map<Eigen::Matrix<double, 12, 3, Eigen::RowMajor>> jacobian_position_i(jacobians[0]);
                jacobian_position_i.setZero();

                jacobian_position_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();

                jacobian_position_i = sqrt_info * jacobian_position_i;

                if (jacobian_position_i.maxCoeff() > 1e8 || jacobian_position_i.minCoeff() < -1e8)
                {
                   // ROS_WARN("numerical unstable in jacobian of model residual wrt position_i");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }
                ROS_DEBUG_STREAM_ONCE("Model jacobian_position_i after:" << jacobian_position_i);
            }
            if (jacobians[1]) // derivative of residual wrt parameter block i.e. 4D attitude_i
            {
                Eigen::Map<Eigen::Matrix<double, 12, 4, Eigen::RowMajor>> jacobian_attitude_i(jacobians[1]);
                jacobian_attitude_i.setZero();

                //jacobian_attitude_i.block<3, 3>(O_P, O_R-O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * (G - Fexti/MASS) * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

                //jacobian_attitude_i.block<3, 3>(O_V-3, O_R-O_R) = Utility::skewSymmetric(Qi.inverse() * ((G - Fexti/MASS) * sum_dt + Vj - Vi));

                jacobian_attitude_i.block<3, 3>(O_P, O_R-O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

                jacobian_attitude_i.block<3, 3>(O_V-3, O_R-O_R) = Utility::skewSymmetric(Qi.inverse() * (G  * sum_dt + Vj - Vi));

                //jacobian_attitude_i.block<3, 3>(O_P, O_R-O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * (G - Fexti) * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

                //jacobian_attitude_i.block<3, 3>(O_V-3, O_R-O_R) = Utility::skewSymmetric(Qi.inverse() * ((G - Fexti) * sum_dt + Vj - Vi));
              
                jacobian_attitude_i = sqrt_info * jacobian_attitude_i;

                if (jacobian_attitude_i.maxCoeff() > 1e8 || jacobian_attitude_i.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in jacobian of model residual wrt attitude_i");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }
                //ROS_DEBUG_STREAM("Fexti: "<< Fexti.transpose());
                ROS_DEBUG_STREAM_ONCE("Model jacobian_attitude_i after:" << jacobian_attitude_i);
            }
            if (jacobians[2])// derivative of residual wrt parameter block i.e. 3D speed i
            {
                Eigen::Map<Eigen::Matrix<double, 12, 3, Eigen::RowMajor>> jacobian_speed_i(jacobians[2]); //buzz <double, 6, 9, Eigen::RowMajor>>
                jacobian_speed_i.setZero();
                
                jacobian_speed_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;

                jacobian_speed_i.block<3, 3>(O_V-3, O_V - O_V) = -Qi.inverse().toRotationMatrix();

                jacobian_speed_i = sqrt_info * jacobian_speed_i;
                ROS_DEBUG_STREAM_ONCE("Model jacobian_speed_i after:" << jacobian_speed_i);

                if (jacobian_speed_i.maxCoeff() > 1e8 || jacobian_speed_i.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in jacobian of model residual wrt speed_i");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }

                //ROS_ASSERT(fabs(jacobian_speed_i.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_speed_i.minCoeff()) < 1e8);
            }
            if (jacobians[3])// derivative of residual wrt  3D external force i
            {
                Eigen::Map<Eigen::Matrix<double, 12, 3, Eigen::RowMajor>> jacobian_fext_i(jacobians[3]);
                jacobian_fext_i.setZero();

                //jacobian_fext_i.block<3, 3>(O_P, 0) = - (0.5/MASS) * Qi.inverse().toRotationMatrix() * sum_dt * sum_dt;
                //jacobian_fext_i.block<3, 3>(O_V-3, 0) = - Qi.inverse().toRotationMatrix() * (sum_dt/MASS);

                jacobian_fext_i.block<3, 3>(O_P, 0) = - 0.5 * sum_dt * sum_dt * Eigen::Matrix3d::Identity();
                jacobian_fext_i.block<3, 3>(O_V-3, 0) = - sum_dt * Eigen::Matrix3d::Identity();
                jacobian_fext_i.block<3, 3>(6, 0) = - 1.0 * Eigen::Matrix3d::Identity();
                jacobian_fext_i.block<3, 3>(9, 0) = Eigen::Matrix3d::Identity();

                //jacobian_fext_i.block<3, 3>(O_P, 0) = - 0.5 * Qi.inverse().toRotationMatrix() * sum_dt * sum_dt;
                //jacobian_fext_i.block<3, 3>(O_V-3, 0) = - Qi.inverse().toRotationMatrix() * sum_dt;

                //Eigen::Matrix<double, 6, 3, Eigen::RowMajor> jacobian_fext_i_before = jacobian_fext_i;

                jacobian_fext_i = sqrt_info * jacobian_fext_i;
                ROS_DEBUG_STREAM_ONCE("Model jacobian_fext_i after:" << jacobian_fext_i);

                //Eigen::Vector3d delta_fext_i=(jacobian_fext_i.transpose()*jacobian_fext_i).inverse() * jacobian_fext_i.transpose()*residual;

                if (jacobian_fext_i.maxCoeff() > 1e8 || jacobian_fext_i.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in jacobian of model residual wrt fext_i");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }
/*                ROS_DEBUG_STREAM_ONCE("!!! big delta_fext_i: " << delta_fext_i.transpose());
                
                if (delta_fext_i.norm() > 3)
                {
                    ROS_DEBUG_STREAM("!!! big delta_fext_i: " << delta_fext_i.transpose());
                    ROS_DEBUG_STREAM("!!! big jacobian_fext_i_before: " << jacobian_fext_i_before);
                    ROS_DEBUG_STREAM("!!! big residual_before: " << residual_before);

                }*/
                //ROS_DEBUG_STREAM_THROTTLE(0.2, "delta_fext_i: "<<delta_fext_i.transpose());
                //ROS_ASSERT(fabs(jacobian_fext_i.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_fext_i.minCoeff()) < 1e8);
            }
            if (jacobians[4])// derivative of residual wrt  3D position j
            {
                Eigen::Map<Eigen::Matrix<double, 12, 3, Eigen::RowMajor>> jacobian_position_j(jacobians[4]); //<double, 6, 3, Eigen::RowMajor>>
                jacobian_position_j.setZero();

                jacobian_position_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

                jacobian_position_j = sqrt_info * jacobian_position_j;
                ROS_DEBUG_STREAM_ONCE("Model jacobian_position_j after:" << jacobian_position_j);

                if (jacobian_position_j.maxCoeff() > 1e8 || jacobian_position_j.minCoeff() < -1e8)
                {
                  //  ROS_WARN("numerical unstable in jacobian of model residual wrt position_j");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }

                //ROS_ASSERT(fabs(jacobian_position_j.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_position_j.minCoeff()) < 1e8);
            }
            if (jacobians[5])
            {
                Eigen::Map<Eigen::Matrix<double, 12, 3, Eigen::RowMajor>> jacobian_speed_j(jacobians[5]); //buzz <double, 6, 9, Eigen::RowMajor>>
                jacobian_speed_j.setZero();

                jacobian_speed_j.block<3, 3>(O_V-3, O_V - O_V) = Qi.inverse().toRotationMatrix();

                jacobian_speed_j = sqrt_info * jacobian_speed_j;
                ROS_DEBUG_STREAM_ONCE("Model jacobian_speed_j after:" << jacobian_speed_j);

                if (jacobian_speed_j.maxCoeff() > 1e8 || jacobian_speed_j.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in jacobian of model residual wrt speed_j");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }

                //ROS_ASSERT(fabs(jacobian_speed_j.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_speed_j.minCoeff()) < 1e8);
            }
            if (jacobians[6]) // derivative of residual wrt  3D external force j
            {
                Eigen::Map<Eigen::Matrix<double, 12, 3, Eigen::RowMajor>> jacobian_fext_j(jacobians[6]); //buzz <double, 6, 9, Eigen::RowMajor>>
                jacobian_fext_j.setZero();

                jacobian_fext_j.block<3, 3>(6, 0) = Eigen::Matrix3d::Identity();

                jacobian_fext_j = sqrt_info * jacobian_fext_j;
                ROS_DEBUG_STREAM_ONCE("Model jacobian_fext_j after:" << jacobian_fext_j);

                if (jacobian_fext_j.maxCoeff() > 1e8 || jacobian_fext_j.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in jacobian of model residual wrt speed_j");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }

                //ROS_ASSERT(fabs(jacobian_speed_j.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_speed_j.minCoeff()) < 1e8);
            }
        }

        return true;
    }

    //bool Evaluate_Direct(double const *const *parameters, Eigen::Matrix<double, 15, 1> &residuals, Eigen::Matrix<double, 15, 30> &jacobians);

    //void checkCorrection();
    //void checkTransition();
    //void checkJacobian(double **parameters);
    IntegrationBase* pre_integration;

};

