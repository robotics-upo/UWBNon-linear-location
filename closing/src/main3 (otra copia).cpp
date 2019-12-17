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
#include <set>
#include <utility>


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

long Identificador[12];

double estCoords[12][3];

double distancia1 = 0;
double distancia2 = 0;
double distancia3 = 0;
double distancia4 = 0;
double distancia5 = 0;
double distancia6 = 0;
double distancia7 = 0;
double distancia8 = 0;
double distancia9 = 0;
double distancia10 = 0;
double distancia11 = 0;
double distancia12 = 0;

int New_ID1 = 1;
int New_ID2 = 2;
int New_ID3 = 3;
int New_ID4 = 4;
int New_ID5 = 5;
int New_ID6 = 6;
int New_ID7 = 7;
int New_ID8 = 8;
int New_ID9 = 9;
int New_ID10 = 10;
int New_ID11 = 11;
int New_ID12 = 12;

int flag1 = 0;
int flag2 = 0;
int flag3 = 0;
int flag4 = 0;
int flag5 = 0;
int flag6 = 0;
int flag7 = 0;
int flag8 = 0;
int flag9 = 0;
int flag10 = 0;
int flag11 = 0;
int flag12 = 0;

int g;
int h;
int q = 0;




//Distancias proporcionadas por el Anchor Identificador
double Distance[12][1000];
//Coordenadas de la posición del robot
double xRobot[1000];
double yRobot[1000];
double zRobot[1000];
//Contadores de registro de distancia y posiciones 
int i = 1;
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

int flag_loam = 0;
double dist_actual;

set<long> Anchors;
set<long>::iterator i = Anchors.begin();


vector<double> pos;
double pseudorange;	


class MyCostFunctor{

    public:

        MyCostFunctor(vector<double> rob_i_, double t_i_)
               : rob_i(rob_i_), t_i(t_i_) {}
            
        template <typename T>
        bool operator()(const T* const pos, const T* const pos2, T* residual) const {
                    //residual[0] = sqrt((sqrt((pos[0]-T(rob_i[0]))*(pos[0]-T(rob_i[0]))+(pos[1]-T(rob_i[1]))*(pos[1]-T(rob_i[1])+(pos[2]-T(rob_i[2]))*(pos[2]-T(rob_i[2]))))-T(t_i))*(sqrt((pos[0]-T(rob_i[0]))*(pos[0]-T(rob_i[0]))+(pos[1]-T(rob_i[1]))*(pos[1]-T(rob_i[1])+(pos[2]-T(rob_i[2]))*(pos[2]-T(rob_i[2]))))-T(t_i)));
                    residual[0] = sqrt((pos[0]-T(rob_i[0]))*(pos[0]-T(rob_i[0]))+(pos[1]-T(rob_i[1]))*(pos[1]-T(rob_i[1])+(pos[2]-T(rob_i[2]))*(pos[2]-T(rob_i[2]))))-T(t_i);
                    ROS_INFO("RESIDUAL");
                return true;
            }


    private:
        const vector<double> rob_i;
        const double t_i;
    
};
//----------------------------------------------------------------------------------
//--------------------  Funciones del sistema---------------------------------------
//----------------------------------------------------------------------------------


//----------------------------------------------------------------------------------
//--------------------Callbacks de los topics subscritos----------------------------
//----------------------------------------------------------------------------------

//Callback para la toma de distancias de los uwb
void uwbCallback(const range_msgs::P2PRangeWithPose &msg)
{
   
        // Identificador[q] = msg.destination_id;
    Anchors.insert(msg.destination_id);
    i = Anchors.begin();
    cout << "Numero de colores: " << Anchors.size() << endl;
    while(i != Anchors.end() ) cout << "\t" << *i++ << endl;
    
    

    if (Identificador[0] == msg.destination_id)
    {
        estCoords[0][0] = msg.position.point.x;
        estCoords[0][1] = msg.position.point.y;
        estCoords[0][2] = msg.position.point.z;
        ROS_INFO("pOSICION X nodo1 [%f]", estCoords[0][0]);


        New_ID1 = 1;
        distancia1 = msg.range;
        flag1 = 1;

    }

    // if (Identificador[1] == msg.destination_id)
    // {
    //     estCoords[1][0] = msg.position.point.x;
    //     estCoords[1][1] = msg.position.point.y;
    //     estCoords[1][2] = msg.position.point.z;
    //     ROS_INFO("pOSICION X nodo2 [%f]", estCoords[1][0]);

    //     New_ID2 = 2;
    //     distancia2 = msg.range;
    //     flag2 = 1;
    
    // }

    // if (Identificador[2] == msg.destination_id)
    // {
    //     estCoords[2][0] = msg.position.point.x;
    //     estCoords[2][1] = msg.position.point.y;
    //     estCoords[2][2] = msg.position.point.z;
    //     ROS_INFO("pOSICION X nodo2 [%f]", estCoords[2][0]);

    //     New_ID3 = 3;
    //     distancia3 = msg.range;
    //     flag3 = 1;

    // }

    // if (Identificador[3] == msg.destination_id)
    // {
    //     estCoords[3][0] = msg.position.point.x;
    //     estCoords[3][1] = msg.position.point.y;
    //     estCoords[3][2] = msg.position.point.z;
    //     ROS_INFO("pOSICION X nodo3 [%f]", estCoords[3][0]);

    //     New_ID4 = 4;
    //     distancia4 = msg.range;
    //     flag4 = 1;
    
    // }

    // if (Identificador[4] == msg.destination_id)
    // {
    //     estCoords[4][0] = msg.position.point.x;
    //     estCoords[4][1] = msg.position.point.y;
    //     estCoords[4][2] = msg.position.point.z;
    //     ROS_INFO("pOSICION X nodo5 [%f]", estCoords[4][0]);

    //     New_ID5 = 5;
    //     distancia5 = msg.range;
    //     flag5 = 1;
            
    // }

    // if (Identificador[5] == msg.destination_id)
    // {
    //     estCoords[5][0] = msg.position.point.x;
    //     estCoords[5][1] = msg.position.point.y;
    //     estCoords[5][2] = msg.position.point.z;

    //     New_ID6 = 6;
    //     distancia6 = msg.range;
    //     flag6 = 1;
        
    // }

    // if (Identificador[6] == msg.destination_id)
    // {
    //     estCoords[6][0] = msg.position.point.x;
    //     estCoords[6][1] = msg.position.point.y;
    //     estCoords[6][2] = msg.position.point.z;

    //     New_ID7 = 7;
    //     distancia7 = 0;
    //     flag7 = 1;
   
    // }

    // if (Identificador[7] == msg.destination_id)
    // {
    //     estCoords[7][0] = msg.position.point.x;
    //     estCoords[7][1] = msg.position.point.y;
    //     estCoords[7][2] = msg.position.point.z;

    //     New_ID8 = 8;
    //     distancia8 = msg.range;
    //     flag8 = 1;
          
    // }

    // if (Identificador[8] == msg.destination_id)
    // {
    //     estCoords[8][0] = msg.position.point.x;
    //     estCoords[8][1] = msg.position.point.y;
    //     estCoords[8][2] = msg.position.point.z;

    //     New_ID9 = 9;
    //     distancia9 = msg.range;
    //     flag9 = 1;
  
    // }

    // if (Identificador[9] == msg.destination_id)
    // {
    //     estCoords[9][0] = msg.position.point.x;
    //     estCoords[9][1] = msg.position.point.y;
    //     estCoords[9][2] = msg.position.point.z;

    //     New_ID10 = 10;
    //     distancia10 = msg.range;
    //     flag10 = 1;
     
    // }

    // if (Identificador[10] == msg.destination_id)
    // {
    //     estCoords[10][0] = msg.position.point.x;
    //     estCoords[10][1] = msg.position.point.y;
    //     estCoords[10][2] = msg.position.point.z;

    //     New_ID11 = 11;
    //     distancia11 = msg.range;
    //     flag11 = 1;
   
    // }

    // if (Identificador[11] == msg.destination_id)
    // {
    //     estCoords[11][0] = msg.position.point.x;
    //     estCoords[11][1] = msg.position.point.y;
    //     estCoords[11][2] = msg.position.point.z;

    //     New_ID12 = 12;
    //     distancia12 = msg.range;
    //     flag12 = 1;
 
    // }

}

//Callback para recibir las coordenadas del robot
void LOAMCallback(const nav_msgs::Odometry tsg)
{
    d_bef = sqrt(( xRobot[cont_loam]-tsg.pose.pose.position.x)*( xRobot[cont_loam]-tsg.pose.pose.position.x)+( yRobot[cont_loam]-tsg.pose.pose.position.y)*( yRobot[cont_loam]-tsg.pose.pose.position.y)+( zRobot[cont_loam]-tsg.pose.pose.position.z)*( zRobot[cont_loam]-tsg.pose.pose.position.z));

    if (d_bef > 0.4)
    {
        cont_loam++;
        xRobot[cont_loam] = tsg.pose.pose.position.x;
        yRobot[cont_loam] = tsg.pose.pose.position.y;
        zRobot[cont_loam] = tsg.pose.pose.position.z;

        if (flag1 == 1)
        {
            Distance[0][cont_loam] = distancia1;
            flag1 = 0;

        }

        if (flag2 == 1)
        {
            Distance[1][cont_loam] = distancia2;
            flag2 = 0;

        }

        if (flag3 == 1)
        {
            Distance[2][cont_loam] = distancia3;
            flag3 = 0;

        }

        if (flag4 == 1)
        {
            Distance[3][cont_loam] = distancia4;
            flag4 = 0;

        }

        if (flag5 == 1)
        {
            Distance[4][cont_loam] = distancia5;
            flag5 = 0;

        }

        if (flag6 == 1)
        {
            Distance[5][cont_loam] = distancia6;
            flag6 = 0;

        }
        if (flag7 == 1)
        {
            Distance[6][cont_loam] = distancia7;
            flag7 = 0;

        }

        if (flag8 == 1)
        {
            Distance[7][cont_loam] = distancia8;
            flag8 = 0;

        }

        if (flag9 == 1)
        {
            Distance[8][cont_loam] = distancia9;
            flag9 = 0;

        }

        if (flag10 == 1)
        {
            Distance[9][cont_loam] = distancia10;
            flag10 = 0;

        }

        if (flag11 == 1)
        {
            Distance[10][cont_loam] = distancia11;
            flag11 = 0;

        }

        if (flag12 == 1)
        {
            Distance[11][cont_loam] = distancia12;
            flag12 = 0;

        }
        
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

            for (g = 0; g < 12; g++)
            {
                
                double estb;
                ROS_INFO("pOSICION X nodo [%d] [%f]", g, estCoords[3][0]);
            
                // Build the problem.
                Problem problem;

                for (int j = 0; j < cont_loam; ++j) {

                    vector<double> rob_i;

                    double t_i;
                    xi = xRobot[j];
                    yi = yRobot[j];
                    zi = zRobot[j]; 

                    rob_i.push_back(xi);

                    rob_i.push_back(yi);

                    rob_i.push_back(zi);

                    t_i = Distance[g][j];


                    CostFunction* cost_f = new AutoDiffCostFunction<MyCostFunctor, 1, 2, 2>(
                                new MyCostFunctor(rob_i, t_i));


                    problem.AddResidualBlock(cost_f, NULL, estCoords[g], &estb);
                    //ROS_INFO("contador j [%f]", cost_f);
                }
                cont_loam = 0;

                ROS_INFO("RESIDUAL4");


                Solver::Options options;
                //options.linear_solver_type = ceres::DENSE_QR;
                options.minimizer_progress_to_stdout = false;

                
                options.check_gradients = false;
                options.gradient_check_relative_precision = 1e5;
                options.function_tolerance = 1e-12;
                options.parameter_tolerance = 1e-12;

                

                Solver::Summary summary;
                Solve(options, &problem, &summary);
                cout << summary.BriefReport() << "\n\n";
                double ext_x;
                ext_x = estCoords[g][0];
                double ext_y;
                ext_y = estCoords[g][1];
                double ext_z;
                ext_z = estCoords[g][2];
                ROS_INFO("Coordenadas para el nodo [%d]", g);
                cout << "coordenadas:\t  x = " << ext_x << endl;
                cout << "coordenadas:\t  y = " << ext_y << endl;
                cout << "coordenadas:\t  z = " << ext_z << endl;

            }
            
            
            

        }
    }


} 
 
