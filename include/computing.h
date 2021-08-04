/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   computing.h
 * Author: jaiem
 *
 * Created on 19 juillet 2021, 16:20
 */

#ifndef COMPUTING_H
#define COMPUTING_H

#include "gazebo_msgs/GetModelState.h"

void disp_param(float& cap,std::vector<float>& P0,std::vector<float>& P1, std::vector<float>& P2, std::vector<float>& P3,std::vector<float>& M0, std::vector<float>& M1, std::vector<float>& M2, float& alpha, float& beta, std::vector<float>& omega, float& R, float& C, float& teta, std::vector<float>& A,std::vector<float>& B);
void getRobotPoseCallback(const geometry_msgs::Pose& robot_pose_topic);
float compute_alpha(std::vector<float> vr, std::vector<float> vt);
//std_msgs::Float32 compute_velocity(std_msgs::Float32 teta, std_msgs::Float32 d, char motor);
//float compute_velocity(float teta, float d, char motor);
double quaternionToAngle(geometry_msgs::Quaternion );
//float compute_teta(float Ax, float Ay);
//float compute_d(float Ax, float Ay);
std::vector<float> compute_bezier(float t, std::vector<float> P0 ,std::vector<float> P1, std::vector<float> P2,std::vector<float> P3);
void __init_param__ (std::vector<float>& P0,std::vector<float>& P1, std::vector<float>& P2, std::vector<float>& P3, float& vr, float& vl, float& vt);
float compute_alpha_beta (std::vector<float> P0,std::vector<float> P1);
void update_parameters(float& d, float& cap,geometry_msgs::Pose robot_pose, std::vector<float>& P0,std::vector<float>& P1, std::vector<float>& P2, std::vector<float>& P3,std::vector<float>& M0, std::vector<float>& M1, std::vector<float>& M2, float& alpha, float& beta, std::vector<float>& omega, float& R, float& C, float& teta, std::vector<float>& A, std::vector<float>& B);
float compute_velocity(float R, float Vt , float d, char motor);
float compute_angle_RP(float Ax, float Ay);
#endif /* COMPUTING_H */
