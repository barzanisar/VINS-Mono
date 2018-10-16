#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>

class ModelFactor : public ceres::SizedCostFunction<6, 3, 4, 3, 3, 3, 3> //position i, attitude i,  speed i, fext i, position j, speed j, 
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


        Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
        residual = pre_integration->evaluate_model(Pi, Qi, Vi, Fexti,
                                            Pj, Vj);


        Eigen::Matrix<double, 12, 12> sqrt_info_full = Eigen::LLT<Eigen::Matrix<double, 12, 12>>(pre_integration->covariance_model.inverse()).matrixL().transpose();
        //sqrt_info.setIdentity();
        Eigen::Matrix<double, 6, 6> sqrt_info; // for just p and v
        sqrt_info.block<3,3>(0,0) = sqrt_info_full.block<3,3>(O_P,O_P);
        sqrt_info.block<3,3>(0,3) = sqrt_info_full.block<3,3>(O_P,O_V);
        sqrt_info.block<3,3>(3,0) = sqrt_info_full.block<3,3>(O_V,O_P);
        sqrt_info.block<3,3>(3,3) = sqrt_info_full.block<3,3>(O_V,O_V);

        ROS_DEBUG_STREAM_ONCE("Model residual before:" << residual);
        ROS_DEBUG_STREAM_ONCE("Model covariance_model before:" << pre_integration->covariance_model);
        ROS_DEBUG_STREAM_ONCE("Model sqrt_info after:" << sqrt_info);
        residual = sqrt_info * residual;

        ROS_DEBUG_STREAM_ONCE("Model residual after:" << residual);

        if (jacobians)
        {
            double sum_dt = pre_integration->sum_dt;

            if (jacobians[0]) // derivative of residual wrt the first parameter block i.e. 3D position_i
            {
                Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jacobian_position_i(jacobians[0]);
                jacobian_position_i.setZero();

                jacobian_position_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();

                jacobian_position_i = sqrt_info * jacobian_position_i;

                if (jacobian_position_i.maxCoeff() > 1e8 || jacobian_position_i.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in preintegration");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }
                ROS_DEBUG_STREAM_ONCE("Model jacobian_position_i after:" << jacobian_position_i);
            }
            if (jacobians[1]) // derivative of residual wrt parameter block i.e. 4D attitude_i
            {
                Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> jacobian_attitude_i(jacobians[1]);
                jacobian_attitude_i.setZero();

                jacobian_attitude_i.block<3, 3>(O_P, O_R-O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * (G - Fexti/MASS) * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

                jacobian_attitude_i.block<3, 3>(O_V-3, O_R-O_R) = Utility::skewSymmetric(Qi.inverse() * ((G - Fexti/MASS) * sum_dt + Vj - Vi));

                jacobian_attitude_i = sqrt_info * jacobian_attitude_i;

                if (jacobian_attitude_i.maxCoeff() > 1e8 || jacobian_attitude_i.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in preintegration");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }
                ROS_DEBUG_STREAM_ONCE("Model jacobian_attitude_i after:" << jacobian_attitude_i);
            }
            if (jacobians[2])// derivative of residual wrt parameter block i.e. 3D speed i
            {
                Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jacobian_speed_i(jacobians[2]); //buzz <double, 6, 9, Eigen::RowMajor>>
                jacobian_speed_i.setZero();
                
                jacobian_speed_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;

                jacobian_speed_i.block<3, 3>(O_V-3, O_V - O_V) = -Qi.inverse().toRotationMatrix();

                jacobian_speed_i = sqrt_info * jacobian_speed_i;
                ROS_DEBUG_STREAM_ONCE("Model jacobian_speed_i after:" << jacobian_speed_i);

                //ROS_ASSERT(fabs(jacobian_speed_i.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_speed_i.minCoeff()) < 1e8);
            }
            if (jacobians[3])// derivative of residual wrt  3D external force
            {
                Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jacobian_fext_i(jacobians[3]);
                jacobian_fext_i.setZero();

                jacobian_fext_i.block<3, 3>(O_P, 0) = - (0.5/MASS) * Qi.inverse().toRotationMatrix() * sum_dt * sum_dt;
                jacobian_fext_i.block<3, 3>(O_V-3, 0) = - Qi.inverse().toRotationMatrix() * (sum_dt/MASS);
                jacobian_fext_i = sqrt_info * jacobian_fext_i;
                ROS_DEBUG_STREAM_ONCE("Model jacobian_fext_i after:" << jacobian_fext_i);

                //ROS_ASSERT(fabs(jacobian_fext_i.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_fext_i.minCoeff()) < 1e8);
            }
            if (jacobians[4])// derivative of residual wrt  3D position j
            {
                Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jacobian_position_j(jacobians[4]); //<double, 6, 3, Eigen::RowMajor>>
                jacobian_position_j.setZero();

                jacobian_position_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

                jacobian_position_j = sqrt_info * jacobian_position_j;
                ROS_DEBUG_STREAM_ONCE("Model jacobian_position_j after:" << jacobian_position_j);

                //ROS_ASSERT(fabs(jacobian_position_j.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_position_j.minCoeff()) < 1e8);
            }
            if (jacobians[5])
            {
                Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jacobian_speed_j(jacobians[5]); //buzz <double, 6, 9, Eigen::RowMajor>>
                jacobian_speed_j.setZero();

                jacobian_speed_j.block<3, 3>(O_V-3, O_V - O_V) = Qi.inverse().toRotationMatrix();

                jacobian_speed_j = sqrt_info * jacobian_speed_j;
                ROS_DEBUG_STREAM_ONCE("Model jacobian_speed_j after:" << jacobian_speed_j);

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

