
#include "ros/ros.h"
#include <math.h>

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <range_msgs/P2PRange.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <stdio.h>
#include <stdlib.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include "eigen3/Eigen/LU"
#include "eigen3/Eigen/Eigenvalues"
#include <vector>
#include <algorithm>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/tf.h>



#define TIEMPORECEPCION 30	// Tiempo de recepción en segundos
#define NMAXDATOS 300	        // Tiempo de recepción en segundos
using namespace std;

int n_msg;  // number of messages received
double R[3]={0};//Vector de distancias tomados con los nodos
double almn[1000][3];//Position Matrix
//Flags de toma de datos de los sensores
double s1=0;
double s2=0;
double s3=0;

// // //Position of nodes
/* double x2=7.47;//x position node 0
double y2=14.7;//y position node 0
double z2=-1.1;//z position node 0
double x4=5.31;//x position node 1
double y4=-0.49;//y position node 1
double z4=-0.95;//z position node 1
double x3=17.41;//x position node 2
double y3=-11.89;//y position node 2
double z3=-1.8;//z position node 2 */


double x3=-7.308;//x position node 0
double y3=-0.78;//y position node 0
double z3=-0.88;//z position node 0
double x4=-4.1625;//x position node 1
double y4=6.5565;//y position node 1
double z4=-1.88;//z position node 1
double x2=0.38;//x position node 2
double y2=0;//y position node 2
double z2=-1.755;//z position node 2

//Posicion que se quiere calcular
double x;//x position a calcular
double y;//y position a calcular
double z;//z position a calcular

int Loampos = 0;

Eigen::MatrixXd P(3,3);

Eigen::MatrixXd P_loam(3,3);
Eigen::MatrixXd P_loamerror(3,3);
Eigen::MatrixXd S_correct(3,3);
Eigen::MatrixXd K_correct(3,3);

Eigen::MatrixXd P_corregida(3,3);
Eigen::Matrix4f transformation2;
// Eigen::Vector3d X(-0.5,-1.5,0.88);
Eigen::Vector3d X(0,0,0);
Eigen::Vector3d X_ayud(0,0,0);
Eigen::Vector3d X_loam(0,0,0);
Eigen::Vector3d X_loamdetecciondecierre(0,0,0);
Eigen::Vector3d XT(0,0,0);
Eigen::Vector3d X2(0,0,0);
Eigen::Vector3d Y_correct(0,0,0);
Eigen::Vector3d X_corregida(0,0,0);

double dist1;
double dist2;
double dist3;
double sal1;
double sal2;
double sal3;
int mp=0;
int mpaux;
int mpcont;
double DB;
//Terminos del jacobiano H de la posicion actual
double a;
double b;
double c;
double d;
double e;
double f;
double g;
double h;
double i;
//Terminos del jacobiano H de la posicion a comprobar el cierre
double j=X2(0)-x3;
double k=X2(1)-y3;
double l=X2(2)-z3;
double m=X2(0)-x4;
double n=X2(1)-y4;
double o=X2(2)-z4;
double p=X2(0)-x2;
double q=X2(1)-y2;
double r=X2(2)-z2;
//Valores de los angulos de euler de la matriz de rotacion
double yaw;
double pitch;
double roll;

double r11;
double r12;
double r13;
double r21;
double r22;
double r23;
double r31;
double r32;
double r33;



int flag_detect;
int n_detect;
float altura=0;

 
double D0;//distance f rom node 1 to node 0
double D1;//distance from node 2 to node 0
double D2;//aux. distance d10
double disB;
double Einv;
double Edet;

double Daux0;//distance from node 1 to node 0
double Daux1;//distance from node 2 to node 0
double Daux2;//aux. distance d10

double matrizpos[10000][3];
double matrizposloam[10000][3];
double matrizposloamdeteccion[10000][3];
double matrizdis[10000][9];
float t1_x=0;
float t1_y=0;
float t1_z=0;
float q1_x=0.500;
float q1_y=0.500;
float q1_z=0.500;
float q1_w=-0.500;
int m3=0;

float pend_yaw;
float pend_pitch;
float pend_roll;
float pend_x;
float pend_y;
float pend_z;

float e_yaw;
float e_pitch;
float e_roll;
float e_x;
float e_y;
float e_z;


float errorminpos;
float errormin;
int cont_angle;
int n_cierre;



//Variables de correccion
float m2;
float error_coo;
int n_pos_to_correct;
float E;
int cont_pos;
float error_min;
Eigen::Vector3d X_error(0,0,0);

float error;
float error1;
float error2;
int cierre=0;

float erroryawcierre;
float errorpitchcierre;
float errorrollcierre;
float errorxcierre;
float errorycierre;
float errorzcierre;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ant(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudsave1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudsave2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudsave3(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudsave4(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudsave5(new pcl::PointCloud<pcl::PointXYZ>);
float cont_map=0;
float cont_loam=0;
//Variables de ros
ros::Publisher pub1;
ros::Publisher  pub;
ros::Publisher  cub;
ros::Publisher  aub;
ros::Publisher  bub;
ros::Publisher  tub;
ros::Time timegps;
ros::Publisher yub;
ros::Publisher rub;
ros::Publisher vub;
ros::Publisher zub;
range_msgs::P2PRange msg;
nav_msgs::Odometry tsg;
nav_msgs::Odometry alt;

void MapCallback(const sensor_msgs::PointCloud2ConstPtr& map_msg)
{
    cont_map++;
    ROS_INFO("Contador nubes de puntos[%f]",cont_map);
	if (cont_map==20)
    {
        pcl::fromROSMsg(*map_msg, *cloudsave1);
    }
    if (cont_map==50)
    {
        pcl::fromROSMsg(*map_msg, *cloudsave2);
    }
    if (cont_map==80)
    {
        pcl::fromROSMsg(*map_msg, *cloudsave3);
    }
    if (cont_map==110)
    {
        pcl::fromROSMsg(*map_msg, *cloudsave4);
    }
    if (cont_map==140)
    {
        pcl::fromROSMsg(*map_msg, *cloudsave5);
    }	
}
void uwbCallback(const range_msgs::P2PRange& msg)
{
   // ROS_INFO("Se ha detectado una medida de rango");

    if ((msg.destination_id==1) && (msg.range!=0) && (s1==0) && (msg.range<30))
    {
        R[0]=msg.range;
        s1=1;
        //ROS_INFO("Distancia nodo 0: [%f].\n", R[0]);

    }
    else if ((msg.destination_id==2) && (msg.range!=0) && (s2==0) && (msg.range<30))
    {
        R[1]=msg.range;
        s2=1;
       // ROS_INFO("Distancia nodo 1: [%f].\n", R[1]);

    }
    else if ((msg.destination_id==3) && (msg.range!=0) && (s3==0) && (msg.range<30))
    {
        R[2]=msg.range;
        s3=1;
      //  ROS_INFO("Distancia nodo 2: [%f].\n", R[2]);

    }
   
    

    else if ((R[0]!=0)&&(R[1]!=0)&&(R[2]!=0))
    {
        


        //Una vez se han recibido las tres distancias correctamente, se dispone a realizar el EKF para la obtención de la posición
        //ROS_INFO("Se han recibido tres medidas consecutivas correctas");
        //El Filtro de Kalman Extendido(EKF, "Extended Kalman Filter"), se compone de dos partes fundamentales: Predicción y Corrección
        //------------------------------------------------------------------------------------------------------------------------------
        //-------------------------------------------   Fase de Predicción  ------------------------------------------------------------
        //------------------------------------------------------------------------------------------------------------------------------
        //ROS_INFO("Se inicia el proceso de prediccion del EKF");
        //ROS_INFO("Posicion de prediccion");
        //std::cout<<X<< "\n\n";
        
        //Consta de dos partes,
        //--> Estimación de la posición
        //--> Estimación de la matriz de covarianza 
        //La estimación de la posición se tomara la posición actual X(x,y,z)
        //Definir matriz P (Covarianza)
        Eigen::MatrixXd Q(3,3);
        Q<<  0.001, 0.000, 0,
              0.000, 0.001, 0,
              0, 0, 0.001;
        
        P=P+Q;
        //ROS_INFO("Matriz de covarianza de prediccion");
        std::cout << P << "\n\n";
        //------------------------------------------------------------------------------------------------------------------------------
        //-------------------------------------------  Operaciones auxiliares  ---------------------------------------------------------
        //------------------------------------------------------------------------------------------------------------------------------
        //Calculo de la matriz H
    	D0=sqrt((X(0)-x3)*(X(0)-x3)+(X(1)-y3)*(X(1)-y3)+(X(2)-z3)*(X(2)-z3));
    	D1=sqrt((X(0)-x4)*(X(0)-x4)+(X(1)-y4)*(X(1)-y4)+(X(2)-z4)*(X(2)-z4));
    	D2=sqrt((X(0)-x2)*(X(0)-x2)+(X(1)-y2)*(X(1)-y2)+(X(2)-z2)*(X(2)-z2));
    	//ROS_INFO("Se calcula el jacobiano del proceso");
        a=X(0)-x3;
        b=X(1)-y3;
        c=X(2)-z3;
        d=X(0)-x4;
        e=X(1)-y4;
        f=X(2)-z4;
        g=X(0)-x2;
        h=X(1)-y2;
        i=X(2)-z2;
        Eigen::MatrixXd H(3,3);
    	H<< a/D0, b/D0, c/D0,
            d/D1, e/D1, f/D1,
            g/D2, h/D2, i/D2;
        //std::cout << H << "\n\n";
        
        //------------------------------------------------------------------------------------------------------------------------------
        //------------------------------------------      Fase de correción     --------------------------------------------------------
        //------------------------------------------------------------------------------------------------------------------------------ 
        //ROS_INFO("Se inicia el proceso de correccion");
        //Calculo de la Y (diferencia entre medidas predichas y las obtenidas de los sensores)
        Eigen::Vector3d Y(sal1,sal2,sal3), Z(R[0], R[1], R[2]), h(D0, D1, D2);
        // std::cout << Y << "\n\n";
        // std::cout << Z << "\n\n";
        // std::cout << h << "\n\n";

        Y=Z-h;
        /* std::cout << Z << "\n\n";
        std::cout << h << "\n\n";
        std::cout << Y << "\n\n"; */
        //Calculo de la matriz S (Matriz de covarianza residual)
        Eigen::MatrixXd S(3,3);
        Eigen::MatrixXd L(3,3);
        
        L<< 0.02, 0.0035, 0, 0.0035, 0.02, 0.003, 0.003, 0, 0.02;        //L<< 0.15, 0.0, 0, 0, 0.45, 0, 0, 0, 0.000000000015;
        //L<< 0.01, 0.0035, 0, 0.0035, 0.02, 0.003, 0.003, 0, 0.01;
        S=H*P*H.transpose()+L;
        //std::cout << S << "\n\n";
        //Calculo de la matriz K (Matriz de ganancias)
        Eigen::MatrixXd K(3,3);
        K=P*H.transpose()*S.inverse();
        //std::cout << K << "\n\n";
        //Calculo de la posición actual corregida
        
        X=X+K*Y;
        X(2)=altura;
        X(1)=X(1)-0.02;
        X(0)=X(0)-0.2;
        
        //ROS_INFO("Errores de posicion inicial del loam y el cierre");
        error=X(0)-X_loam(0);
        error1=X(2)-X_loam(1);
        error2=(-X(1))-X_loam(2);
        //std::cout << error << "\n\n";
        //std::cout << error1 << "\n\n";
        //std::cout << error2 << "\n\n";


        matrizposloam[mp][0]=X_loam(0);
        matrizposloam[mp][1]=X_loam(1);
        matrizposloam[mp][2]=X_loam(2);

        matrizpos[mp][0]=X(0);
        matrizpos[mp][1]=X(1);
        matrizpos[mp][2]=X(2);
        // matrizposloam[mp][0]=X_loam(0);
        // matrizposloam[mp][1]=X_loam(1);
        // matrizposloam[mp][2]=X_loam(2);
        //std::cout << matrizpos[mp][0] << "\n\n";
        //std::cout << matrizpos[mp][1] << "\n\n";
        //std::cout << matrizpos[mp][2] << "\n\n";
        //ROS_INFO("Posicion del drone");
        //ROS_INFO("Posicion: [%f] [%f] [%f] .\n", X(0), X(1), X(2));
        //Calculo de la matriz de covarianza corregida
        P=(Eigen::MatrixXd::Identity(3,3)-K*H)*P;
        matrizdis[mp][0]=P(0,0);
        matrizdis[mp][1]=P(0,1);
        matrizdis[mp][2]=P(0,2);
        matrizdis[mp][3]=P(1,0);
        matrizdis[mp][4]=P(1,1);
        matrizdis[mp][5]=P(1,2);
        matrizdis[mp][6]=P(2,0);
        matrizdis[mp][7]=P(2,1);
        matrizdis[mp][8]=P(2,2);
        // std::cout << P << "\n\n";
        // std::cout << matrizdis[mp][0] << "\n\n";
        
        
        if  (mp>40)
        {
        //    ROS_INFO("Entrando en proceso de busqueda de deteccion del cierre de bucle");
            for (mpaux=mp-30; mpaux>=0; mpaux--)
            {
                //ROS_INFO("Distancias de Bhattacharyya");
                X2(0)=matrizpos[mpaux][0];
                X2(1)=matrizpos[mpaux][1];
                X2(2)=matrizpos[mpaux][2];
                //std::cout << X2 << "\n\n";
                disB=sqrt((X2(0)-X(0))*(X2(0)-X(0))+(X2(1)-X(1))*(X2(1)-X(1))+(X2(2)-X(2))*(X2(2)-X(2)));
                //std::cout << disB << "\n\n";   
                
                Eigen::MatrixXd P2(3,3);
                P2(0,0)=matrizdis[mpaux][0];
                P2(0,1)=matrizdis[mpaux][1];
                P2(0,2)=matrizdis[mpaux][2];
                P2(1,0)=matrizdis[mpaux][3];
                P2(1,1)=matrizdis[mpaux][4];
                P2(1,2)=matrizdis[mpaux][5];
                P2(2,0)=matrizdis[mpaux][6];
                P2(2,1)=matrizdis[mpaux][7];
                P2(2,2)=matrizdis[mpaux][8];
                //std::cout << P2 << "\n\n";
                Eigen::MatrixXd PT(3,3);
                XT=X-X2;
                PT=(P+P2)/2;
                //std::cout << PT << "\n\n";
                //std::cout << XT << "\n\n";
                Einv=XT.transpose()*PT.inverse()*XT;
                //std::cout << Einv << "\n\n";
                Edet=log(PT.determinant()/(P2.determinant()+P.determinant()));
                //std::cout << Edet << "\n\n";
                DB=Einv/8+Edet/2;
        
                
                if (DB<2.4)
                {
                    
                    flag_detect=0;
                    flag_detect=1;//Se activa el flag de deteccion del cierre para realizar la correcion de la deriva generada hasta ese punto
                    ROS_INFO("Distancia de Bhattacharaya: [%f].\n", DB);
                    if (disB>0.4)
                    {
                        // pcd_to_map.header.frame_id = "/loc_init"; // /camera_init
                        //Comienza el proceso ICP para obtener la orientación del velodyne
                        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                        icp.setInputSource(cloud_in);
                        icp.setInputTarget(cloud_map);
                        pcl::PointCloud<pcl::PointXYZ> Final;
                        icp.align(Final);
                        std::cout << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore() << std::endl;
                        Eigen::Matrix4f transformation = icp.getFinalTransformation ();
                        Eigen::Matrix3f m5;
                        m5 = transformation.topLeftCorner(3,3);
                        Eigen::AngleAxisf aa(m5);
                        Eigen::Quaternionf quaternion(aa);
                        std::cout << "\nw:" << quaternion.w() << " x:" << quaternion.x() << " y:" << quaternion.y() << " z:" << quaternion.z() << "\n";
                        std::cout << " The axis : " << aa.axis() << std::endl; 
                        std::cout << " The angle : " << aa.angle() << std::endl; 
                        //std::cout << transformation << std::endl;
                        //transformation2 = icp.getFinalTransformation ();
                        //pub1.publish(pcd_to_map);
                        transformation(0,3)=matrizposloam[mpaux][0];
                        transformation(1,3)=matrizposloam[mpaux][1];
                        transformation(2,3)=matrizposloam[mpaux][2];
                        //std::cout << transformation << std::endl;

                        //Calculo del error incremental proporcionalmente
                        nav_msgs::Odometry posicionescierre;
                        posicionescierre.pose.pose.orientation.x=quaternion.x();
                        posicionescierre.pose.pose.orientation.y=quaternion.y();
                        posicionescierre.pose.pose.orientation.z=quaternion.z();
                        posicionescierre.pose.pose.orientation.w=quaternion.w();
                        posicionescierre.pose.pose.position.x=matrizposloam[mp][0];
                        posicionescierre.pose.pose.position.y=matrizposloam[mp][1];
                        posicionescierre.pose.pose.position.z=matrizposloam[mp][2];
                        posicionescierre.header.frame_id="/loc_init";
                        ROS_INFO("Posición Loam actual donde se detecta el cierre");
                        std::cout << matrizposloam[mp][0] << "\n\n";

                        std::cout << matrizposloam[mp][1] << "\n\n";

                        std::cout << matrizposloam[mp][2] << "\n\n";

                        std::cout << X_loam << "\n\n";
                        //posiciones.header.stamp=timegps;
                        tub.publish(posicionescierre);
                        nav_msgs::Odometry posicionesdetect;
                        posicionesdetect.pose.pose.orientation.x=quaternion.x();
                        posicionesdetect.pose.pose.orientation.y=quaternion.y();
                        posicionesdetect.pose.pose.orientation.z=quaternion.z();
                        posicionesdetect.pose.pose.orientation.w=quaternion.w();
                        posicionesdetect.pose.pose.position.x=X(0);
                        posicionesdetect.pose.pose.position.y=X(2);
                        posicionesdetect.pose.pose.position.z=-X(1);
                        posicionesdetect.header.frame_id="/loc_init";
                        ROS_INFO("Posición del Loam anterior zona visitada con anterioridad");
                        std::cout << matrizposloam[mpaux][0] << "\n\n";

                        std::cout << matrizposloam[mpaux][1] << "\n\n";

                        std::cout << matrizposloam[mpaux][2] << "\n\n";
                        //posiciones.header.stamp=timegps;
                        yub.publish(posicionesdetect);


                        //Para la realización de la transformación de la nube de puntos se calcula la evolucion del crecimiento del error
                        

                    }
            
                    

                    //Calculo de la matriz de rotacion al aplicar el error de crecimiento proporcional

                    
                    cierre=0;
                    
                    //Proceso de correción del mapa
                        
                } 
                       

                
                

            
                
            } 
            cierre=1;   
        }
        mp++;
        //std::cout << mpcont << "\n\n";
        std::cout << mp << "\n\n";
        s1=0;
        s2=0;
        s3=0;
        R[1]=0;
        R[0]=0;
        R[2]=0;
    }
    else
    {
        if ((s2==1) && (s3==1) && (s1==0))
        {
            R[0]=0;
            R[1]=0; 
            R[2]=0;
            s1=0;
            s2=0;
            s3=0; 
 
        }
            
        else if ((s1==1) && (s3==1) && (s2==0))
        {
            R[0]=0;
            R[1]=0;
            R[2]=0;
            s1=0;
            s2=0;
            s3=0;
        }
            
        else if ((s1==1) && (s2==1) && (s3==0))
        {
            R[1]=0;
            R[0]=0;
            R[2]=0;
            s1=0;
            s2=0;
            s3=0;

        }
    }
                 
//Cuando se detecta el cierre de bucle se realiza el proceso de correcion del mapa 
//Se realiza mediante un filtro de kalman con entradas de prediccion las posiciones con deriva obtenidas de la trayectoria
//Y como medida de los sensores las posiciones de los nodos que se tomaran como absolutas

    //Se almacenan las posiciones obtenidas para el posterior analisis de similitud de posiciones
    //Se publiucan la posicion actual del drone
    X_ayud(0) = X(0);
    X_ayud(1) = X(1)-1;
    X_ayud(2) = X(2);
    nav_msgs::Odometry posiciones;
    posiciones.pose.pose.position.x=X_ayud(0);
    posiciones.pose.pose.position.y=X_ayud(2);
    posiciones.pose.pose.position.z=-X_ayud(1);
    posiciones.header.frame_id="/loc_init";
    //posiciones.header.stamp=timegps;
    pub.publish(posiciones);
    // Se muestra la medida por pantalla
    //ROS_INFO("Posicion por trilateracion: [%f],[%f],[%f]",X.x, X.y, X.z/*almn[i][0],almn[i][1],almn[i][2]*/ );
    i++;
    
    
}


void LOAMCallback(const nav_msgs::Odometry tsg)
{
    X_loam(0)=tsg.pose.pose.position.x;
    X_loam(1)=tsg.pose.pose.position.y;
    X_loam(2)=tsg.pose.pose.position.z;
    cont_loam++;
    Loampos = 1;
    ROS_INFO("Contador LOAM [%f]", cont_loam);
    if (m=0)
    {
        X=X_loam;
        m3=1;
    }
    
}

// void alturaCallback(const nav_msgs::Odometry alt)
// {
//     altura=alt.pose.pose.position.z;
// }

   


void pclCallback(const sensor_msgs::PointCloud2ConstPtr& velo_msg)
{
	
	pcl::fromROSMsg(*velo_msg, *cloud_in );
}

void pcl2Callback(const sensor_msgs::PointCloud2ConstPtr& pcd_msg)
{
	
	pcl::fromROSMsg(*pcd_msg, *cloud_map );
}






int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    range_msgs::P2PRange msg;
    ros::NodeHandle nh;
    n_msg = 0;
    //dreal = 0;
    int destino;
    destino=msg.destination_id;
    //  // Se comienza la recepcion de datos
    
    ros::Subscriber sub = nh.subscribe("/ranges_uwb", 1, uwbCallback);
    //ros::Subscriber cub = nh.subscribe("/corrected_height", 1, alturaCallback);
    //ros::Subscriber cub = nh.subscribe("/mavros/global_position/local", 1, alturaCallback);
    ros::Subscriber nub = nh.subscribe("/integrated_to_init", 1, LOAMCallback);
    ros::Subscriber Subscriber = nh.subscribe("/laser_cloud_corner_last", 1, pclCallback);
    ros::Subscriber Subscriber1 = nh.subscribe("/velodyne_cloud_registered", 1, MapCallback);
    ros::Subscriber Sub = nh.subscribe("/cloud_pcd", 1, pcl2Callback);

    // pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/vel_to_loc", 1);
    pub=nh.advertise<nav_msgs::Odometry>("/posicion_uwb",1);
    tub=nh.advertise<nav_msgs::Odometry>("/posicion_loamcierre",1);
    yub=nh.advertise<nav_msgs::Odometry>("/posicion_uwbdetect",1);

    P<<  0.001, 0.000, 0,
     0.000, 0.001, 0,
     0, 0, 0.001;

    // Bucle
    ros::spin();
    return 0;
} 
 
 