
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include <range_msgs/P2PRange.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <algorithm>


#define TIEMPORECEPCION 30	// Tiempo de recepción en segundos
#define NMAXDATOS 300	        // Tiempo de recepción en segundos
using namespace std;

int n_msg;  // number of messages received
int i=0;
float dreal;  // Distancia a la que han sido colocados los sensores de distancia
float r[3]={0};//vector of nodes' distances
float posicionf[3]={0};//vector of position
float almn[1000][3];//Position Matrix
float s1=0;
float s2=0;
float s3=0;

//Position of nodes
float x0=0;//x position node 0
float y0=0;//x position node 0
float z0=0;//x position node 0

float x1=0;//x position node 1
float y1=0;//x position node 1
float z1=0;//x position node 1

float x2=0;//x position node 2
float y2=0;//x position node 2
float z2=0;//x position node 2

float div;

float d10;//distance from node 1 to node 0
float d20;//distance from node 2 to node 0
float b10;//aux. distance d10
float b20;//aux. distance d20
float matrizpos[1000][3];
float matrizdis[1000][3];
int j=0;
float a=0,b=0,c=0,d=0,n=0,m=0;
float A=0,B=0,C=0,D=0;
float div=0;
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

void uwbCallback(const range_msgs::P2PRange msg)
{

    if ((msg.source_id==0) && (msg.range!=0) && (s1==0))
    {
        r[0]=msg.range;
        s1=1;
        ROS_INFO("Distancia nodo 0: [%f].\n", r[0]);

    }
    else if ((msg.source_id==2) && (msg.range!=0) && (s2==0))
    {
        r[1]=msg.range;
        s2=1;
        ROS_INFO("Distancia nodo 1: [%f].\n", r[1]);

    }
    else if ((msg.source_id==1) && (msg.range!=0) && (s3==0))
    {
        r[2]=msg.range;
        s3=1;
        ROS_INFO("Distancia nodo 2: [%f].\n", r[2]);

    }
    else
    {
        if ((s2==1) && (s3==1))
            
            r[0]=0;
        
        else if ((s1==1) && (s3==1))
            r[1]=0;
        else if ((s1==1) && (s3==1))
            r[2]=0;


        r[0]=0;
        r[1]=0;
        r[2]=0;
        s1=0;
        s2=0;
        s3=0;
       
    }

    if ((r[0]!=0)&&(r[1]!=0)&&(r[2]!=0))
    {
        a=x1-x0;
        b=x2-x0;
        c=y1-y0;
        d=y2-y0;
        n=z1-z0;
        m=z2-z0;
        div=a*a*d*d+b*b*c*c-2*a*b*c*d;
        A=(d*(a*d-c*b))/(div);
        B=(b*(b*d-d*a))/(div);
        C=(c*(b*c-d*a))/(div);
        D=(a*(a*d-c*b))/(div);
        d10=sqrt(a*a+c*c+n*n);
        d10=sqrt(b*b+d*d+m*m);
        b10=(r[0]*r[0]-r[1]*r[1]+d10)/2;
        b20=(r[0]*r[0]-r[2]*r[2]+d20)/2;
        posicionf[0]=(A*b10+B*b20)+x0;
        posicionf[1]=((C*b10+C*b20)+y0);
        nav_msgs::Odometry posiciones;
        posiciones.pose.pose.position.x=posicionf[0];
        posiciones.pose.pose.position.y=posicionf[1];
        posiciones.pose.pose.position.z=posicionf[2];
        posiciones.header.frame_id="/map";
        //posiciones.header.stamp=timegps;
        pub.publish(posiciones);
        // Se muestra la medida por pantalla
        ROS_INFO("Posicion por trilateracion: [%f],[%f],[%f]",posicionf[0],posicionf[1],posicionf[2]/*almn[i][0],almn[i][1],almn[i][2]*/ );
        i++;
        r[0]=0;
        r[1]=0;
        r[2]=0;
        s1=0;
        s2=0;
        s3=0;
    }


}
void lidarrangeCallback(const std_msgs::Float32 msg)
{
    posicionf[2]=msg.data+0.454;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    range_msgs::P2PRange msg;
    ros::NodeHandle nh;
    n_msg = 0;
    dreal = 0;
    int destino;
    destino=msg.destination_id;

    //ROS_INFO("Caracterizacion de sensores de rango.\n [%d]", destino);

    //  ROS_INFO("Introduzca la distancia a la que se encuentra el sensor de la base:");

    //  scanf("%f", &dreal);

    //  getchar();
    //  ROS_INFO("Pulse una tecla para comenzar la recepcion de datos del sensor uwb");
    //  getchar();

    //  // Se inicia el temporizador que parará la recepcion de datos pasados TIEMPORECEPCION segundos
    //  ros::Timer timer = nh.createTimer(ros::Duration(TIEMPORECEPCION), timerCallback, true);
    //  // Se comienza la recepcion de datos
    ros::Subscriber nub = nh.subscribe("/corrected_height", 1, lidarrangeCallback);
    ros::Subscriber sub = nh.subscribe("/ranges_uwb", 1, uwbCallback);

    pub=nh.advertise<nav_msgs::Odometry>("/posicion_uwb",1);
    // Bucle
    ros::spin();




    return 0;
}

