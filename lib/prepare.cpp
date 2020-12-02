//
// Created by uav on 2020/10/30.
//
#include "core.h"

/**
 *  prepare.cpp
 */
//悬停漂移总量
Eigen::Vector2d hoverDriftSum = Eigen::Vector2d::Zero();
//悬停时相对于起飞点的漂移量
Eigen::Vector2d hover2homeDrift = Eigen::Vector2d::Zero();

Eigen::Vector3d drift = Eigen::Vector3d::Zero();
int hoverFunCount=300;
void setHoverPva();

void setBeforeOffbPva()
{
    
    pvaTargetPointMsg.positions.clear();
    pvaTargetPointMsg.velocities.clear();
    pvaTargetPointMsg.accelerations.clear();
    pvaTargetPointMsg.effort.clear();

    pvaTargetPointMsg.positions.push_back(dronePoseLp.pose.position.x);
    pvaTargetPointMsg.positions.push_back(dronePoseLp.pose.position.y);
    pvaTargetPointMsg.positions.push_back(0.001);
    pvaTargetPointMsg.positions.push_back(0);

    pvaTargetPointMsg.velocities.push_back(0);
    pvaTargetPointMsg.velocities.push_back(0);
    pvaTargetPointMsg.velocities.push_back(0);

    pvaTargetPointMsg.accelerations.push_back(0);
    pvaTargetPointMsg.accelerations.push_back(0);
    pvaTargetPointMsg.accelerations.push_back(0);

    pvaTargetPointMsg.effort.push_back(0);

}

/**
 * get yawdegree before taking off
 */
bool get_yaw_fun(){

    getYawFuncCount--;
   // ROS_INFO("YAW FUNCTION:%d",getYawFuncCount);

    if(getYawFuncCount==0)
    {

        droneHome.x() +=dronePoseCurrent.pose.position.x;
        droneHome.y() +=dronePoseCurrent.pose.position.y;

        //在盲飞点的基础上，加上x和y方向的漂移量
        for (int i = 0; i < 3; i++)
        {
            
            blindPoints[i][0]+=droneHome.x();
            blindPoints[i][1]+=droneHome.y();

         }

        //在圆环初始点的基础上，加上x和y方向的漂移量
        for (int i = 0; i < 6; i++)
        {
             frontPoints[i][0]+=droneHome.x();
             frontPoints[i][1]+=droneHome.y();
             centerPoints[i][0]+=droneHome.x();
             centerPoints[i][1]+=droneHome.y();
             loop_pose[i][0]+=droneHome.x();
             loop_pose[i][1]+=droneHome.y();
        }

        //在降落点的基础上，加上x和y方向的漂移量
        for (int i = 0; i < 2; i++)
        {
            landOffPoints[i][0]+=droneHome.x();
            landOffPoints[i][1]+=droneHome.y();
        }

        return true;
    }
    else
    {
         return false;
    }
}


/**
 * take off precess
 */
bool take_off_func(mavros_msgs::State state)
{

    if(currentStateMsg.mode != "OFFBOARD" )
    {
        return false;
    }

    ///checking Mode before taking off
    ROS_INFO_ONCE("TAKE_OFF MODE!!!");
    
    if(abs(planeCurrHeight-homeHoverHeight) < 0.1){
         hoverFunCount=100;
         return true;
    }

    ///set takeoff point
    takeOffPoint[0] = 0;
    takeOffPoint[1] = 0;
    takeOffPoint[2] = homeHoverHeight;

    setTakeOffPva();
    pubPvaTargetPoint.publish(pvaTargetPointMsg);
    ROS_INFO("----------target height:%f",pvaTargetPointMsg.positions[2]);

    return false;
}


/**
 * hovering and adjusting process
 */
bool hover_and_adjust_func()
{
    hoverFunCount--;
    ROS_INFO_ONCE(" HOVER MODE!!!!!");

    if(hoverFunCount==0)
    {
        return true;
    }
    else if(hoverFunCount==-500)
    {
        return true;
    }
    else
    {
        setHoverPva();
        pubPvaTargetPoint.publish(pvaTargetPointMsg);
        return false;
    }

}

/**
 * set frontPoint value in front of loop
 */
void setTakeOffPva(){
    pvaTargetPointMsg.positions.clear();
    pvaTargetPointMsg.velocities.clear();
    pvaTargetPointMsg.accelerations.clear();
    pvaTargetPointMsg.effort.clear();

    pvaTargetPointMsg.positions.push_back(takeOffPoint[0]);
    pvaTargetPointMsg.positions.push_back(takeOffPoint[1]);
    pvaTargetPointMsg.positions.push_back(takeOffPoint[2]);
    pvaTargetPointMsg.positions.push_back(takeOffPoint[3]);

    pvaTargetPointMsg.velocities.push_back(takeOffPoint[4]);
    pvaTargetPointMsg.velocities.push_back(takeOffPoint[5]);
    pvaTargetPointMsg.velocities.push_back(takeOffPoint[6]);

    pvaTargetPointMsg.accelerations.push_back(takeOffPoint[7]);
    pvaTargetPointMsg.accelerations.push_back(takeOffPoint[8]);
    pvaTargetPointMsg.accelerations.push_back(takeOffPoint[9]);

    pvaTargetPointMsg.effort.push_back(-1);
}

/**
 * set hoverPoint value
 */
 void setHoverPva()
 {
     pvaTargetPointMsg.positions.clear();
     pvaTargetPointMsg.velocities.clear();
     pvaTargetPointMsg.accelerations.clear();
     pvaTargetPointMsg.effort.clear();

     pvaTargetPointMsg.positions.push_back(1.1);
     pvaTargetPointMsg.positions.push_back(1.2);
     pvaTargetPointMsg.positions.push_back(1.3);
     pvaTargetPointMsg.positions.push_back(0);

     pvaTargetPointMsg.velocities.push_back(0);
     pvaTargetPointMsg.velocities.push_back(0);
     pvaTargetPointMsg.velocities.push_back(0);

     pvaTargetPointMsg.accelerations.push_back(0);
     pvaTargetPointMsg.accelerations.push_back(0);
     pvaTargetPointMsg.accelerations.push_back(0);

     pvaTargetPointMsg.effort.push_back(-3);

 }

 /**
 * set takeOffPoint value
 */
 double takeOffPoint[10] =
 {
     //x, y, z, yaw, vx, vy, vz, ax, ay, az
     0 , 0 , 0 , 0, 0, 0, 0, 0, 0, 0
 };
