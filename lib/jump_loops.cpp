//
// Created by uav on 2020/10/30.
//

#include "core.h"


int loopStep = 0;
double frontLoopDistance = 1.1;
double behindLoopDistance = 0.5;
double height = 1.0;
double velocityX = 0.5;

trajectory_msgs::JointTrajectoryPoint pvaTargetPointMsg;

void setBoardPva(int numberLoop);
bool isArrivedBoard(int numberLoop);

double loop_height[6]={1.35-0.1,1.45,1.55,1.60,1.60,1.75+0.3};
double board_height[6]={0.5,0.7-0.2,0.9-0.2,0.7,0.9,1.0};


double loop_pose[6][2]={
    {2.3,-1.5},
    {4.6,-2.7},
    {6.67,-1.0},
    {9.34,2.89},
    {12.75,2.0},
    {18.4,-2.15}
};


double frontPoints[6][10] =
{
     //x,   y,   z,yaw,vx,vy,vz,ax,ay,az
    {2.3 - frontLoopDistance, -1.5 +0.1 , loop_height[0], 0, velocityX, 0, 0, 0, 0, 0},//dot_1
    {4.6 - 1.5 -0.1 , -2.7 + 0.2-0.1, loop_height[1], 0, velocityX, 0, 0, 0, 0, 0},  //dot_3
    {6.67-0.2 - frontLoopDistance-0.2-0.1, -1.0-0.15, loop_height[2], 0, velocityX, 0, 0, 0, 0, 0},//dot_5
    {9.34-0.3- frontLoopDistance, 2.89 -0.1, loop_height[3], 0, velocityX, 0, 0, 0, 0, 0},//dot_7
    {12.75- frontLoopDistance-0.15-0.2  , 2.0+0.1   , loop_height[4], 0, velocityX, 0, 0, 0, 0, 0},//dot_9
    {18.4 - 1.4 -0.4, -1.8 ,loop_height[5] , 0, velocityX, 0, 0, 0, 0, 0},  //dot_13
};


double centerPoints[7][10] =
{
    //x, y, z, yaw, vx, vy, vz, ax, ay, az
    {2.3+ 0.5, -1.5 + 0.1  , loop_height[0] , 0, velocityX, 0, 0, 0, 0, 0}, //dot_2
    {4.6 + 0.1+ behindLoopDistance, -2.7 +0.2 -0.1,loop_height[1]  , 0, velocityX, 0, 0, 0, 0, 0}, //dot_4
    {6.67 + behindLoopDistance, -1.0  ,loop_height[2]  , 0, velocityX, 0, 0, 0, 0, 0},//dot_6
    {9.34 + behindLoopDistance, 2.89-0.1  , loop_height[3]  , 0, velocityX, 0, 0, 0, 0, 0},//dot_8
    {12.75 + 0.3, 2 + 0.1  ,loop_height[4] , 0, velocityX, 0, 0, 0, 0, 0},//dot_10
    {18.4 + 0.5, -1.8 + 0.2 ,loop_height[5] , 0, velocityX, 0, 0, 0, 0, 0},  //dot_14
};


bool go_to_loop(int numberLoop)
{

    if(numberLoop==0)
    {
        ROS_INFO_ONCE("----start loop %d",numberLoop);
    }
    if(numberLoop==1)
    {
        ROS_INFO_ONCE("----start loop %d",numberLoop);
    }
    if(numberLoop==2)
    {
        ROS_INFO_ONCE("----start loop %d",numberLoop);
    }
    if(numberLoop==3)
    {
        ROS_INFO_ONCE("----start loop %d",numberLoop);
    }
    if(numberLoop==4)
    {
        ROS_INFO_ONCE("----start loop %d",numberLoop);
    }
    if(numberLoop==5)
    {
        ROS_INFO_ONCE("----start loop %d",numberLoop);
    }
    if(numberLoop==6)
    {
        ROS_INFO_ONCE("----start loop %d",numberLoop);
    }

    if(loopStep==0)
    {
        if(numberLoop==0)
        {
            loopStep=1;
            ROS_INFO("go to loop %d front center",numberLoop);
            return false;
        }

        setBoardPva(numberLoop);
        pubPvaTargetPoint.publish(pvaTargetPointMsg);
        if(update_drift(numberLoop))
        {
            loopStep=1;
            ROS_INFO("go to loop %d front center",numberLoop);
        }

        if(isArrivedBoard(numberLoop))
        {
            loopStep=1;
            update_drift(numberLoop);
            ROS_INFO("go to loop %d front center",numberLoop);
         
        }
        return false;

    }

    if(loopStep==1) //飞到圆心正前方
    {
       // update_drift(numberLoop);

        setFrontPva(numberLoop);
        pubPvaTargetPoint.publish(pvaTargetPointMsg);
        if(isArrivedFront(numberLoop))
        {
            ROS_INFO("go to loop %d back",numberLoop);
            loopStep=2;
        }
        return false;
    }

    if(loopStep==2)  //飞到圆心后面
    {
 
        setCenterPva(numberLoop);
        pubPvaTargetPoint.publish(pvaTargetPointMsg);
        if(isArrivedCenter(numberLoop))
        {
            ROS_INFO("ARRIVED loop %d back",numberLoop);
             loopStep=0;
            return true;
        }
        return false;

    }

}

//飞到号码牌子前面
void setBoardPva(int numberLoop)  
{
    pvaTargetPointMsg.positions.clear();
    pvaTargetPointMsg.velocities.clear();
    pvaTargetPointMsg.accelerations.clear();
    pvaTargetPointMsg.effort.clear();
    pvaTargetPointMsg.positions.push_back(frontPoints[numberLoop][0]-drift.x());
     pvaTargetPointMsg.positions.push_back(frontPoints[numberLoop][1]-drift.y());
    pvaTargetPointMsg.positions.push_back(board_height[numberLoop]-drift.z());
    pvaTargetPointMsg.positions.push_back(frontPoints[numberLoop][3]);

    pvaTargetPointMsg.velocities.push_back(frontPoints[numberLoop][4]);
    pvaTargetPointMsg.velocities.push_back(frontPoints[numberLoop][5]);
    pvaTargetPointMsg.velocities.push_back(frontPoints[numberLoop][6]);

    pvaTargetPointMsg.accelerations.push_back(frontPoints[numberLoop][7]);
    pvaTargetPointMsg.accelerations.push_back(frontPoints[numberLoop][8]);
    pvaTargetPointMsg.accelerations.push_back(frontPoints[numberLoop][9]);

    pvaTargetPointMsg.effort.push_back(numberLoop);

}

//飞到环的圆心的正前方
void setFrontPva(int numberLoop){
    pvaTargetPointMsg.positions.clear();
    pvaTargetPointMsg.velocities.clear();
    pvaTargetPointMsg.accelerations.clear();
    pvaTargetPointMsg.effort.clear();

    pvaTargetPointMsg.positions.push_back(frontPoints[numberLoop][0]-drift.x());
    pvaTargetPointMsg.positions.push_back(frontPoints[numberLoop][1]-drift.y());
    pvaTargetPointMsg.positions.push_back(frontPoints[numberLoop][2]-drift.z());
    pvaTargetPointMsg.positions.push_back(frontPoints[numberLoop][3]);

    pvaTargetPointMsg.velocities.push_back(frontPoints[numberLoop][4]);
    pvaTargetPointMsg.velocities.push_back(frontPoints[numberLoop][5]);
    pvaTargetPointMsg.velocities.push_back(frontPoints[numberLoop][6]);

    pvaTargetPointMsg.accelerations.push_back(frontPoints[numberLoop][7]);
    pvaTargetPointMsg.accelerations.push_back(frontPoints[numberLoop][8]);
    pvaTargetPointMsg.accelerations.push_back(frontPoints[numberLoop][9]);

    pvaTargetPointMsg.effort.push_back(numberLoop);
}

//飞到环的圆心的后面
void setCenterPva(int numberLoop){
    pvaTargetPointMsg.positions.clear();
    pvaTargetPointMsg.velocities.clear();
    pvaTargetPointMsg.accelerations.clear();
    pvaTargetPointMsg.effort.clear();

    pvaTargetPointMsg.positions.push_back(centerPoints[numberLoop][0]-drift.x());
    pvaTargetPointMsg.positions.push_back(centerPoints[numberLoop][1]-drift.y());
    pvaTargetPointMsg.positions.push_back(centerPoints[numberLoop][2]-drift.z());
    pvaTargetPointMsg.positions.push_back(centerPoints[numberLoop][3]);

    pvaTargetPointMsg.velocities.push_back(centerPoints[numberLoop][4]);
    pvaTargetPointMsg.velocities.push_back(centerPoints[numberLoop][5]);
    pvaTargetPointMsg.velocities.push_back(centerPoints[numberLoop][6]);

    pvaTargetPointMsg.accelerations.push_back(centerPoints[numberLoop][7]);
    pvaTargetPointMsg.accelerations.push_back(centerPoints[numberLoop][8]);
    pvaTargetPointMsg.accelerations.push_back(centerPoints[numberLoop][9]);

    pvaTargetPointMsg.effort.push_back(numberLoop);

}

//判断是否到达号码牌的正前方
bool isArrivedBoard(int numberLoop)
{
    if(abs(dronePoseCurrent.pose.position.x-pvaTargetPointMsg.positions[0])<0.1 &&
       abs(dronePoseCurrent.pose.position.y-pvaTargetPointMsg.positions[1])<0.1 &&
       abs(planeCurrHeight-pvaTargetPointMsg.positions[2])<0.1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

//判断是否到达环圆心点前的位置
bool isArrivedFront(int numberLoop){
    if(abs(dronePoseCurrent.pose.position.x-(frontPoints[numberLoop][0]-drift.x()))<0.1&&
       abs(dronePoseCurrent.pose.position.y-(frontPoints[numberLoop][1]-drift.y()))<0.1 &&
       abs(planeCurrHeight-(frontPoints[numberLoop][2]-drift.z()))<0.1)
        return true;
    else
        return false;
}

//判断是否到达环圆心点后的点位置
bool isArrivedCenter(int numberLoop){
    if(abs(dronePoseCurrent.pose.position.x-(centerPoints[numberLoop][0]-drift.x()))<0.1 &&
       abs(dronePoseCurrent.pose.position.y-(centerPoints[numberLoop][1]-drift.y()))<0.1 &&
       abs(planeCurrHeight-(centerPoints[numberLoop][2]-drift.z()))<0.1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

//利用视觉YOLO更新飞机的漂移量
bool update_drift(int numberLoop)
{
    if(visionPose.pose.orientation.w==1&&visionPose.pose.orientation.x==numberLoop+1)
    {
        drift.x() =  loop_pose[numberLoop][0] - visionPose.pose.position.x - dronePoseCurrent.pose.position.x;
        drift.y() =  loop_pose[numberLoop][1] - visionPose.pose.position.y - dronePoseCurrent.pose.position.y;
        drift.z() =  board_height[numberLoop] - visionPose.pose.position.z - planeCurrHeight;

        ROS_INFO("VISIONPOSE.X:  %f     dronePoseCurrent.x:  %f", visionPose.pose.position.x,dronePoseCurrent.pose.position.x);
        ROS_INFO("VISIONPOSE.y:  %f     dronePoseCurrent.y:  %f", visionPose.pose.position.y,dronePoseCurrent.pose.position.y);
        ROS_INFO("VISIONPOSE.z:  %f     dronePoseCurrent.z:  %f", visionPose.pose.position.z,planeCurrHeight);

        ROS_INFO("DRIFT  %f   %f  %f",drift.x(),drift.y(),drift.z());

        return true;
    }
    else
    {
        return false;
    }
}



