#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf/transform_datatypes.h"
#include <sstream>
#include "sensor_msgs/LaserScan.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

struct ball{
  int x;
  int y;
};

struct robot{
  int x;
  int y;
  double rot;
};

struct blue_goal{
  int x;
  int y;
};

struct yellow_goal{
  int x;
  int y;
};

struct environment{
  ball m_ball;
  robot m_bot;
  blue_goal m_bgoal;
  yellow_goal m_ygoal;
};
struct node{
    float angle_avg;
    float angle_start;
    float angle_end;
    float distance;
    struct node *left;
    struct node *right;
};
typedef struct node NodeStruct;
typedef NodeStruct *NodePtr;

NodePtr creat_node(float ang_start,float ang_end,float dis,int line_num);
NodePtr cal_dis_angle(NodePtr p);
NodePtr append_node(NodePtr p);
NodePtr head=NULL;
NodePtr tmp=NULL;
NodePtr tail=NULL;
environment env;

NodePtr creat_node(float ang_start,float ang_end,float dis){
    NodePtr newnode = (NodePtr)malloc(sizeof(node));
    //newnode->angle_avg = (ang_start+ang_end)/2;
    newnode->angle_start=ang_start;
    newnode->angle_end=ang_end;
    newnode->distance = dis;
    newnode->right = NULL;
    newnode->left = NULL;
    //printf("creat");
    return newnode;
}

NodePtr append_node(NodePtr p)
{
    //printf("append");
    if(head==NULL)
    {
        head=p;
        tail=p;
        p->right=p;
        p->left=p;
    }
    else
    {
        tail->right=p;
        p->left=tail;
        p->right=NULL;
        tail=p;
    }
    return head;
}

void display_list(NodePtr p){
    p=head;
    while (p!=NULL)
    {
        printf("ang_start:%f\tang_end:%f\tdis_avg:%f\n ",p->angle_start,p->angle_end,p->distance);
        //printf("ang_avg:%f\tdis_avg:%f\n ",p->angle_avg,p->distance);
        p=p->right;
    }
}
int counter_test;
float data_condition_dis=0;
float data_condition_range=0;
float total_dis_avg=0;
float total_dis_avg_sq=0;
float data_test[3000][2];
void apf_test(const sensor_msgs::LaserScan &model)
{
   //float angle=0;
   counter_test=(model.angle_max-model.angle_min)/model.angle_increment;
  // printf("%d",counter_test);
   total_dis_avg=0;
   total_dis_avg_sq=0;
   for(int i=0;i<=counter_test;i++)
   {
       data_test[i][0]=model.angle_min+i*model.angle_increment;
       if(model.ranges[i]==model.ranges[i]){
            data_test[i][1]=model.ranges[i];
       }
       else
       {
            data_test[i][1]=0;
       }
      total_dis_avg=total_dis_avg+data_test[i][1];
      total_dis_avg_sq=total_dis_avg_sq+data_test[i][1]*data_test[i][1];
     // printf("data_test[i][0]:%f\n",data_test[i][1]);
   }
   data_condition_dis=total_dis_avg/counter_test;
   data_condition_range=sqrt(total_dis_avg_sq-data_condition_dis*data_condition_dis);
   printf("%f\t%f\n",data_condition_dis,data_condition_range);

}

void GazeboCallback(const gazebo_msgs::ModelStates::ConstPtr &model)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  //ROS_INFO("model name: %s",model->name[0]);
  tf::Quaternion q;
  double roll, pitch, yaw;
  tf::quaternionMsgToTF(model->pose[3].orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  env.m_bot.x = model->pose[3].position.x*100;
  env.m_bot.y = model->pose[3].position.y*100;
  env.m_bot.rot = yaw;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fira6_strategy");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Subscriber sub = n.subscribe("gazebo/model_states", 1000, GazeboCallback);
  ros::Subscriber sub1 = n.subscribe("/merger_scan", 1000, apf_test);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {

      int cal_count=0;
      NodePtr p=NULL;
      float data[3];
      float dis_avg;
      int num=0;
      for(int i=0;i<=counter_test-1;i++)
      {

             if(abs(data_test[i][1]-data_test[i+1][1])<=1000&&data_test[i][1]!=0){
                 if(cal_count==0)
                 {
                     data[0]=data_test[i][0];
                     dis_avg=dis_avg+data_test[i][1];
                     cal_count++;
                 }
                 else
                 {
                     dis_avg=dis_avg+data_test[i][1];
                     cal_count++;
                 }

             }
             else
             {
                 if(cal_count!=0)
                 {
                     data[1]=data_test[i][0];
                     cal_count++;
                     //printf("%d\n",cal_count);
                     data[2]=(dis_avg+data_test[i][1])/cal_count;
                     //printf("%f\n",data[2]);
                     //printf("angle_start:%f\t angle_end:%f\t dis:%f\n",data[0],data[1],data[2]);
                     if(data[2]<=2.072){
                        p=creat_node(data[0],data[1],data[2]);
                        append_node(p);
                        num++;
                     //printf("%d\n",num);
                     }
                     dis_avg=0;
                     cal_count=0;
                 }
                 dis_avg=0;
                 cal_count=0;
             }

    }
   // display_list(p);
    std_msgs::String msg;
    std::stringstream ss;
   // ss << "hello world " << count;
    msg.data = ss.str();
    //ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    //ROS_INFO("mbot_x: %d\tmbot_y: %d\tmbot_yaw: %f\n",env.m_bot.x,env.m_bot.y,env.m_bot.rot);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}

