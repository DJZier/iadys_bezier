/*
 * Copyright (C) 2020 IADYS  - All Rights Reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 *
 * This file contains confidential information. All rights (including copyright,
 * confidential information, trade secrets and design rights) are owned by IADYS.
 * No use or disclosure is to be made without the written permission of IADYS.
 *
 * \author Florian CONVERT
 * test
 */

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <unistd.h>
#include <vector>

#include "gazebo_msgs/GetModelState.h"
#include "computing.h"
#include "constants.h"

geometry_msgs::Pose robot_pose;


void getRobotPoseCallback(const geometry_msgs::Pose& robot_pose_topic) {
    robot_pose = robot_pose_topic;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "alignment_first_control"); // initializing the "alignment_first_control" node
    ros::NodeHandle n;
    ros::NodeHandle privateNodeHandle("~");

    geometry_msgs::Vector3 thrusters_command;

    std_msgs::Float32 cmd_thruster_r; // initial command on the first right thruster
    std_msgs::Float32 cmd_thruster_l; // initial command on the first left thruster
    std_msgs::Float32 cmd_thruster_t; // initial command on the lateral thruster
    std_msgs::Float32 angle; // angle between cap and P2 (bezier point of control)
    std_msgs::Float32 teta_robot;
    std_msgs::Float32 Vtx; // trajectory vector --> x position
    std_msgs::Float32 Vty;// trajectory vector --> x position

    float vr, vl;//vitesses des moteurs droite gauche 
    float vt = 2; // vitesse moyenne
    std::vector<float> P0(2),P1(2),P2(2),P3(2); //Position of the fourth point of control
    
    float teta,alpha,beta, R, C; // alpha et beta servent a calculer omega le centre du cercle de l'arc, teta l'angle formé parles extrémités de l'arc et du centre omega, R rayon de courbe, C corde de l'arc
    std::vector<float> Mt(2),M0(2), M1(2), M2(2); //Coordonnées des trois points formant l'arc de cercle pour la trajectoire du robot
    std::vector<float> omega(2); // Centre du cercle passant par M0,M1,M2
    std::vector<float> A(2), B(2); // points milieux des cordes M0M1 et M1M2
    float d= 0.26; // distance entre les moteurs
    float cap; // Cap du robot
    
    
    // Initialisation de paramtètres
     __init_param__(P0, P1, P2, P3, vr, vl ,vt);    
    cmd_thruster_t.data = 0.0;
    double loop_frequency = 10; // hz
    
    // Souscription au topic robot_pose qui détermine la position du root dans le world
    ros::Subscriber nearest_point_rf_s3_sub = n.subscribe("/robot_pose", 1, getRobotPoseCallback); // subscribing to the robot position "/robot_pose" topic


    ros::Rate loop_rate((ros::Rate)loop_frequency);

    // Déclaration des publisher
    ros::Publisher thrust_l_pub = n.advertise<std_msgs::Float32>("thrust_l", 1); //publishing RC channel 1 on "thrust_l" topic
    ros::Publisher thrust_r_pub = n.advertise<std_msgs::Float32>("thrust_r", 1); //publishing RC channel 1 on "thrust_r" topic
    ros::Publisher thrust_t_pub = n.advertise<std_msgs::Float32>("thrust_t", 1); //publishing RC channel 1 on "thrust_t" topic
    ros::Publisher angle_pub = n.advertise<std_msgs::Float32>("angle", 1); //publishing RC channel 1 on "angle" topic"
    ros::Publisher teta_pub = n.advertise<std_msgs::Float32>("teta", 1); //publishing RC channel 1 on "angle" topic"

    
    // Angle du robot par rapport à la plateforme
     float teta_RP = compute_angle_RP(robot_pose.position.x,robot_pose.position.y);
     
     // Balayage du robot sur place pour trouver la plateforme --> simulation comportement ArUco
        while ( (cap+teta_RP)>5 || (cap+teta_RP)<-5) {
            ros::spinOnce();
            
            update_parameters(d, cap, robot_pose, P0,P1,P2,P3,M0,M1,M2, alpha, beta, omega, R, C, teta, A, B);
            if ((cap+teta_RP)>0){
                cmd_thruster_l.data = -5;
                cmd_thruster_r.data = 5;                                
            }
            else {
                cmd_thruster_r.data = -5;
                cmd_thruster_l.data = 5;
            } 
            thrust_l_pub.publish(cmd_thruster_l);
            thrust_r_pub.publish(cmd_thruster_r);
            teta_RP = compute_angle_RP(robot_pose.position.x,robot_pose.position.y);
        }
    
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();

        update_parameters(d, cap, robot_pose, P0,P1,P2,P3,M0,M1,M2, alpha, beta, omega, R, C, teta, A, B);
        
        // Arrêt du robot lorqu'il est dans la plateforme, sinon on calcule la vitesse des moteurs
        if (robot_pose.position.x-P3[0] < 0.5 && robot_pose.position.y-P3[1] < 0.5 && robot_pose.position.x-P3[0] > -0.5 && robot_pose.position.y-P3[1] > -0.5){
            vr=0;
            vl=0;
        }else{
            vl = compute_velocity(R, vt , d, 'l');
            vr = compute_velocity(R, vt , d, 'r');
        }
        
        std::cout << "vr = " << vr << std::endl;        
        std::cout << "vl = " << vl << std::endl;

        cmd_thruster_l.data = vl;
        cmd_thruster_r.data = vr;
        angle.data = cap;
        teta_robot.data = compute_angle_RP(robot_pose.position.x,robot_pose.position.y);
        
        thrust_l_pub.publish(cmd_thruster_l);
        thrust_r_pub.publish(cmd_thruster_r);
        angle_pub.publish(angle);
        teta_pub.publish(teta_robot);
        

    }
     return 0;
}




/**Problème avec ces ptn d'angles encore la
 
 calcul courbe de bézier --> 4 points exemple 1m devant le robot et plateforme
 * calculer distane trajectoire (courbe de bézier)
 * calcul de la vitesse a appliquer aux motuer grace à l'applatissement de la courbe de tangente
 * calculer le temps de récation du robot (expérimentalement)
 * trouver le prochain point t+1
 * dérivée de t+1 --> cap du point
 * différence de cap
 * 
 * 
 * départ temps de réaction 500ms --> converti en distance grace à vistesse
 * 
 * attention aux inversions de projections 
 */