#ifndef TF_LISTERNER_H
#define TF_LISTERNER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <pthread.h>
#include <signal.h>

class Tf_Listerner
{
public:
    Tf_Listerner(std::string parent_frame,std::string child_frame);
    ~Tf_Listerner();
    float x();
    float y();
    float z();
    float ox();
    float oy();
    float oz();
    float ow();
    void stop_tf_listerner();

private:
    ros::NodeHandle n_;
    std::string parent_frame_;
    std::string child_frame_;
    float x_;
    float y_;
    float z_;
    float ox_;
    float oy_;
    float oz_;
    float ow_;
    tf::TransformListener listener_;

    int stop_tf_listerner_thread_flag_;
    void PthreadTfListerner();
    static void *ThreadTfListerner(void * arg);
    void ThreadRun();
    pthread_t m_tid;
};
#endif