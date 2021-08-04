#include "constants.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include "computing.h"
#include "gazebo_msgs/GetModelState.h"
#include <vector>

#define square(a) (a)*(a)


//float compute_alpha(std::vector<float> vr, std::vector<float> vt){
//    float nvr = sqrt(square(vr[1])+square(vr[2]));
//    std::cout << "nvr = " << nvr <<  std::endl;
//    float nvt = sqrt(square(vt[1])+square(vt[2]));
//    return acos((((vr[1]*vt[1]+vr[2]*vt[2])/(nvr*nvt)))*3.14/180);
//}

int coef_dist_motor = 5;
void disp_param(float& cap, std::vector<float>& P0,std::vector<float>& P1, std::vector<float>& P2, std::vector<float>& P3,std::vector<float>& M0, std::vector<float>& M1, std::vector<float>& M2, float& alpha, float& beta, std::vector<float>& omega, float& R, float& C, float& teta, std::vector<float>& A,std::vector<float>& B)
{
    std::cout << "affichage des paramètres: "  << std::endl;
    std::cout << "cap " << cap << std::endl;
    std::cout << "cos : " << cos(cap*PI/180)  << std::endl;
        std::cout << "sin : " << sin(cap*PI/180)  << std::endl;
    std::cout << "P0 x: " << P0[0] << " y: " << P0[1] << std::endl;
    std::cout << "P1 x: " << P1[0] << " y: " << P1[1] << std::endl;
    std::cout << "P2 x: " << P2[0] << " y: " << P2[1] << std::endl;
    std::cout << "P3 x: " << P3[0] << " y: " << P3[1] << std::endl;
    std::cout << "M0 x: " << M0[0] << " y: " << M0[1] << std::endl;
    std::cout << "M1 x: " << M1[0] << " y: " << M1[1] << std::endl;
    std::cout << "M2 x: " << M2[0] << " y: " << M2[1] << std::endl;
    std::cout << "alpha " << alpha << std::endl;
    std::cout << "beta " << beta << std::endl;
    std::cout << "tan(alpha) " << tan(alpha*PI/180) << std::endl;
    std::cout << "tan(beta) " << tan(beta*PI/180) << std::endl;
    std::cout << "omega x: " << omega[0] << " y: " << omega[1] << std::endl;
    std::cout << "R: " << R << std::endl;
    std::cout << "C: " << C << std::endl;
    std::cout << "teta: " << teta << std::endl;
    std::cout << "A x: " << A[0] << " y: " << A[1] << std::endl;
    std::cout << "B x: " << B[0] << " y: " << B[1] << std::endl;
}
std::vector<float> compute_bezier(float t, std::vector<float> P0 ,std::vector<float> P1, std::vector<float> P2,std::vector<float> P3){
    std::vector<float> r(2);
    float a = pow((1-t), 3) * P0[0];
    float b = 3*t*pow((1-t),2) * P1[0];
    float c = 3*t*t*(1-t) * P2[0];
    float d = t*t*t * P3[0];
    r[0] = a+b+c+d;

    float e = pow((1-t), 3) * P0[1];
    float f = 3*t*pow((1-t),2) * P1[1];
    float g = 3*t*t*(1-t) * P2[1];
    float h = t*t*t * P3[1];
    r[1] = e+f+g+h;

    return r;
}

float compute_alpha_beta (std::vector<float> P0,std::vector<float> P1){
    return -atan(((P1[1]-P0[1])/(P1[0]-P0[0])))*180/PI;
}

void update_parameters(float& d, float& cap, geometry_msgs::Pose robot_pose, std::vector<float>& P0,std::vector<float>& P1, std::vector<float>& P2, std::vector<float>& P3,std::vector<float>& M0, std::vector<float>& M1, std::vector<float>& M2, float& alpha, float& beta, std::vector<float>& omega, float& R, float& C, float& teta, std::vector<float>& A,std::vector<float>& B){
        // Définition de P0
        P0[0]= robot_pose.position.x; // --> essayer de déclarer les Pi du meme type que .position
        P0[1]= robot_pose.position.y;
        cap = quaternionToAngle(robot_pose.orientation)*180/PI;

        // Définition de P3
        P3[0] = 8.76;
        P3[1] = -3;               

        // Définition de P2
        P2[0] = P3[0] - sqrt(square(robot_pose.position.x-P3[0])+square(robot_pose.position.y-P3[1]))/2;
        P2[1] = P3[1];
        // Calcul de M0
        M0 = P0;
        
        if (robot_pose.position.x-P3[0] > 1 || robot_pose.position.y-P3[1] > 1 || robot_pose.position.x-P3[0] < -2|| robot_pose.position.y-P3[1] < -1){
            // Définition de P1
            P1[0] = P0[0] + 2*cos(cap*PI/180)*((P3[0]-P1[0])/3);
            P1[1] = P0[1] + 2*sin(cap*PI/180)*((P3[0]-P1[0])/3);

        

        // Calcul de M1
        M1 = compute_bezier(0.2, P0, P1, P2, P3);

        // Calcul de M2
        M2 = compute_bezier(0.4, P0, P1, P2, P3);

        }else {
            P1=P2;
            M1 = compute_bezier(0.2, P0, P1, P2, P3);
            M2=P3;
        }


        // Calculs d'alpha et beta
        alpha = compute_alpha_beta(M0,M1);
        beta = compute_alpha_beta(M1,M2);

        // Calculs de A et B
        A[0] = (M0[0]+M1[0])/2;
        A[1] = (M0[1]+M1[1])/2;

        B[0] = (M2[0]+M1[0])/2;
        B[1] = (M2[1]+M1[1])/2;
        
        // Calcul de Omega
        omega[1] = A[1] + ((A[0]-B[0])-(A[1]-B[1])*tan(beta*PI/180))/(tan(beta*PI/180)-tan(alpha*PI/180));        
        omega[0] = A[0] + (omega[1]-A[1]) * tan(alpha*PI/180);
        
        disp_param(cap, P0,P1,P2,P3,M0,M1,M2, alpha, beta, omega, R, C, teta, A, B);

        // Calcul du rayon de courbure R
        float teta_robot = compute_angle_RP(robot_pose.position.x,robot_pose.position.y);
        
        if ((cap+teta_robot)>0){
            R = sqrt((square(omega[0]-M0[0])+square(omega[1]-M0[1])));
        }else {
            R = -sqrt((square(omega[0]-M0[0])+square(omega[1]-M0[1])));
        }
        
        

        // Calcul de la corde M0M2 --> C
        C = sqrt((square(M2[0]-M0[0])+square(M2[1]-M0[1])));

        // Calcul de teta 
        teta = 2* asin(C/(2*R))*180/PI;
}

void __init_param__ (std::vector<float>& P0,std::vector<float>& P1, std::vector<float>& P2, std::vector<float>& P3, float& vr, float& vl, float& vt){
        // Initialisation des points de contrôle
    P0[0] = 0;
    P0[1] = 0;
    P3[0] = 8.76;
    P3[1] = -3;
    
    // Initialisation des vitesses
    vr = 0;
    vl = 0;
    vt = 2;
}

// Compute velocity of right and left motor to follow the trajectory
float compute_velocity(float R, float Vt , float d, char motor){
    
    float velocity;
    //compute the circle radius
    

    if (motor == 'l'){
        velocity = ((R-coef_dist_motor*d)/R)*Vt;
    }
    else if (motor == 'r') {
        velocity =  ((R+coef_dist_motor*d)/R)*Vt;
    }

    return velocity;
}

// Compute angle between perpendicular axis of the platform and the robot
float compute_angle_RP(float Ax, float Ay){
    float Axp = 8.76;
    float Ayp = -3;
    if (Ayp<Ay){return (atan((Axp-Ax)/(Ayp-Ay))*180/PI)+90;}
    else{return (atan((Axp-Ax)/(Ayp-Ay))*180/PI)-90;}
    
}



double quaternionToAngle(geometry_msgs::Quaternion quaternion) {
    double angle;

    double x = quaternion.x;
    double y = quaternion.y;
    double z = quaternion.z;
    double w = quaternion.w;

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (pow(y, 2) + pow(z, 2));
    angle = std::atan2(t3, t4); // computing the robot cap in the world frame

//    std::cout << "robot heading  " << angle << std::endl;

    return angle;
}

