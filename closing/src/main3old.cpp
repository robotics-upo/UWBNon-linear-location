
#include "ros/ros.h"
#include <math.h>

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <range_msgs/P2PRangeWithPose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <stdio.h>
#include <stdlib.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <cstdlib>      
#include "glog/logging.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include "eigen3/Eigen/LU"
#include "eigen3/Eigen/Eigenvalues"
#include <vector>
#include <algorithm>
#include <iostream>


#include <tf/tf.h>

// Eigen::Matrix3Xd Distance;
// Eigen::VectorXcd Id;
// Eigen::VectorXcd Contador;
// Eigen::VectorXcd Registro_id;

//----------------------------------------------------------------------------------
//----------------------Datos previos y variables ----------------------------------
//----------------------------------------------------------------------------------

using namespace std;

using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::LossFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver; 

using ceres::Covariance;




//Identificador del Anchor 
double Identificador = 1815290605989855728;
//Distancias proporcionadas por el Anchor Identificador
double Distance[1000];
//Coordenadas de la posición del robot
double xRobot[1000];
double yRobot[1000];
double zRobot[1000];
//Contadores de registro de distancia y posiciones 
int i = 0;
int k = 0;
int cont_loam = 1;
int const n_Distancias = 20;
//Alturas a la que estan dispuestos todos lo Anchor considerada conocida y del robot
float AlturaRobotMedia;
float Sumatorio;
const float z_Anchor = 0.8 ; 
float Z = 0;
double d_bef;
//Valores iniciales para la optimización
double initial_x = 5.0;
double x = initial_x;

double initial_y = 5.0;
double y = initial_y;

double initial_z = 0.8;
double z = initial_y;
double rob_i[3];


vector<double> pos;
double pseudorange;	


class MyCostFunctor{

    public:

        MyCostFunctor(vector<double> rob_i_, double t_i_)
               : rob_i(rob_i_), t_i(t_i_) {}
            
        template <typename T>
        bool operator()(const T* const pos, const T* const pos2, T* residual) const {
                T square_sum = T(0);
                    for (int i = 0; i < rob_i.size(); ++i) {
                        square_sum += pow(pos[i] - T(rob_i[i]) , 2);
                }
                T distance = (square_sum != T(0)) ? sqrt(square_sum) : T(0) ;
                    residual[0] = sqrt((distance) - (t_i)*(distance) - (t_i));
                    ROS_INFO("RESIDUAL");
                return true;
            }


    private:
        const vector<double> rob_i;
        const double t_i;
    
};


//----------------------------------------------------------------------------------
//--------------------Callbacks de los topics subscritos----------------------------
//----------------------------------------------------------------------------------

//Callback para la toma de distancias de los uwb
void uwbCallback(const range_msgs::P2PRangeWithPose &msg)
{
    // int i=0;
    // //1º Primero se comprueba que el Id no esta registrado, en caso negativo se almacena dentro del sistema
    // for (i=0;i=11;i++)
    // {
    //     if ((Id[i] == 0) && (Registro_id[i] == 0))
    //     {
    //         Id[i] = msg.destination_id;
    //         Registro_id[i] = 1;
    //         break;
    //     }

    // }
    // //2º Se pasa a realizar el almacenamiento de las distancia de Rango al menos 3 por Id
    // for(j=0;j=11;j++)
    // {
    //     if (Id[i] == msg.destination_id;)
    //     {
            
    //     }
        
    // }
   // ROS_INFO("Identificador: [%d].\n", msg.destination_id);
    if (Identificador == msg.destination_id)
    {
        Distance[i] = msg.range;
       // ROS_INFO("Distancia nodo 0: [%f].\n", Distance[i]);
        
    }
    
    i++;  
     

}

//Callback para recibir las coordenadas del robot
void LOAMCallback(const nav_msgs::Odometry tsg)
{
    d_bef = sqrt(( xRobot[cont_loam]-tsg.pose.pose.position.x)*( xRobot[cont_loam]-tsg.pose.pose.position.x)+( yRobot[cont_loam]-tsg.pose.pose.position.y)*( yRobot[cont_loam]-tsg.pose.pose.position.y)+( zRobot[cont_loam]-tsg.pose.pose.position.z)*( zRobot[cont_loam]-tsg.pose.pose.position.z));
    ROS_INFO("Distancia en loam [%f]", d_bef);
    ROS_INFO("Distancia en loam [%f]", xRobot[cont_loam]);
    ROS_INFO("Distancia en loam [%f]", tsg.pose.pose.position.x);
    ROS_INFO("Distancia en loam [%f]", yRobot[cont_loam]);
    ROS_INFO("Distancia en loam [%f]", tsg.pose.pose.position.y);
    ROS_INFO("Distancia en loam [%f]", zRobot[cont_loam]);
    ROS_INFO("Distancia en loam [%f]", tsg.pose.pose.position.z);
    ROS_INFO("Distancia en loam [%d]", cont_loam);
    if (d_bef > 0.2)
    {
        cont_loam++;
        xRobot[cont_loam] = tsg.pose.pose.position.x;
        yRobot[cont_loam] = tsg.pose.pose.position.y;
        zRobot[cont_loam] = tsg.pose.pose.position.z;
        
        d_bef = 0;

    }
 


    
   // ROS_INFO("Contadorrrrrrrr: [%d].\n", cont_loam);
}




//----------------------------------------------------------------------------------
// ---------------------------Main principal---------------------------------------
//----------------------------------------------------------------------------------


int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    range_msgs::P2PRangeWithPose msg;
    ros::NodeHandle nh;
    // n_msg = 0;
    // //dreal = 0;
    // int destino;
    // destino=msg.destination_id;
    //  // Se comienza la recepcion de datos
    
    ros::Subscriber sub = nh.subscribe("/localino_node/anchor/range", 1, uwbCallback);
    //ros::Subscriber cub = nh.subscribe("/corrected_height", 1, alturaCallback);
    //ros::Subscriber cub = nh.subscribe("/mavros/global_position/local", 1, alturaCallback);
    ros::Subscriber nub = nh.subscribe("/aft_mapped_to_init", 1, LOAMCallback);
    // ros::Subscriber Subscriber = nh.subscribe("/laser_cloud_corner_last", 1, pclCallback);
    // ros::Subscriber Subscriber1 = nh.subscribe("/velodyne_cloud_registered", 1, MapCallback);
    // ros::Subscriber Sub = nh.subscribe("/cloud_pcd", 1, pcl2Callback);

    // pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/vel_to_loc", 1);
    // pub=nh.advertise<nav_msgs::Odometry>("/posicion_uwb",1);
    // tub=nh.advertise<nav_msgs::Odometry>("/posicion_loamcierre",1);
    // yub=nh.advertise<nav_msgs::Odometry>("/posicion_uwbdetect",1);

    // P<<  0.001, 0.000, 0,
    //  0.000, 0.001, 0,
    //  0, 0, 0.001;
    double xi, yi, zi;
   
    
    pos.push_back(x);

    pos.push_back(y);

    pos.push_back(z);
    


    while (ros::ok())
    {
        ros::spinOnce();
        //Se define la función de coste del sistema
        if (cont_loam == 100)
        {
            ROS_INFO("Se procede a calcular la posición del nodo");
            //Se asume que las alturas del robot son del rango entre 10 y 1 cm con lo que se calcula la media 
            
            double estCoords[3];
            double estb;
           
            // Build the problem.
            Problem problem;

            for (int j = 0; j < cont_loam; ++j) {

                vector<double> rob_i;

                double t_i;
                xi = xRobot[i];
                yi = yRobot[i];
                zi = zRobot[i]; 

                rob_i.push_back(xi);

                rob_i.push_back(yi);

                rob_i.push_back(zi);

                t_i = Distance[i];


                CostFunction* cost_f = new AutoDiffCostFunction<MyCostFunctor, 1, 3, 1>(
                            new MyCostFunctor(rob_i, t_i));


                problem.AddResidualBlock(cost_f, NULL, estCoords, &estb);
                ROS_INFO("contador j [%d]", j);
            }
            cont_loam = 0;

            ROS_INFO("RESIDUAL4");


            Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.minimizer_progress_to_stdout = true;

			
			options.check_gradients = true;
			options.gradient_check_relative_precision = 1e-4;
            

            Solver::Summary summary;
            Solve(options, &problem, &summary);
            cout << summary.BriefReport() << "\n\n";
            double ext_x;
            ext_x = estCoords[0];
            double ext_y;
            ext_y = estCoords[1];
            double ext_z;
            ext_z = estCoords[2];
            cout << "coordenadas:\t  x = " << ext_x << endl;
            cout << "coordenadas:\t  y = " << ext_y << endl;
            cout << "coordenadas:\t  z = " << ext_z << endl;
            

        }
    }


} 
 
 
