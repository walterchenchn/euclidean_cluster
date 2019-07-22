#include "tf_listerner.h"

Tf_Listerner::Tf_Listerner(std::string parent_frame,std::string child_frame)
: x_(0),y_(0),z_(0),ox_(0),oy_(0),oz_(0),ow_(1),stop_tf_listerner_thread_flag_(0)
{
    parent_frame_ = parent_frame;
    child_frame_ = child_frame;

    static int run_number = 0;
    //ROS_INFO("Whether to enter tf listerner.");
    //if(run_number = 0)
    {
        ROS_INFO("Whether to enter tf listerner.");
        PthreadTfListerner();
        run_number = 1;
    }
}

Tf_Listerner::~Tf_Listerner()
{
    if(stop_tf_listerner_thread_flag_)
    {
        pthread_kill(m_tid,0);
    }
}

float Tf_Listerner::x()
{
    return x_;
}

float Tf_Listerner::y()
{
    return y_;
}

float Tf_Listerner::z()
{
    return z_;
}

float Tf_Listerner::ox()
{
    return ox_;
}

float Tf_Listerner::oy()
{
    return oy_;
}

float Tf_Listerner::oz()
{
    return oz_;
}

float Tf_Listerner::ow()
{
    return ow_;
}

void Tf_Listerner::stop_tf_listerner()
{
    stop_tf_listerner_thread_flag_ = 1;
}

void Tf_Listerner::PthreadTfListerner()
{
    if(pthread_create(&m_tid,NULL,ThreadTfListerner,(void*)this) != 0)
    {
        ROS_INFO("Start tf listerner thread failed!");
        return; 
    }
}

void * Tf_Listerner::ThreadTfListerner(void * arg)
{
    Tf_Listerner *ptr =(Tf_Listerner*) arg;
    ptr->ThreadRun();
    return NULL;
}

void Tf_Listerner::ThreadRun()
{
    ROS_INFO("Start tf listerner.");
    if(stop_tf_listerner_thread_flag_ == 1)
    {
        //pthread_kill(m_tid,0);
        return;
    }

    listener_.waitForTransform(parent_frame_,child_frame_,ros::Time(0),ros::Duration(4.0));
    while(ros::ok())
    {
        tf::StampedTransform transform;
        try
        {
            //2. 监听对应的tf,返回平移和旋转
            //std::cout<<"2. 监听对应的tf,返回平移和旋转"<<std::endl;
            listener_.lookupTransform(parent_frame_, child_frame_,
                               ros::Time(0), transform);
                               //ros::Time(0)表示最近的一帧坐标变换，不能写成ros::Time::now()
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        x_ = transform.getOrigin().x();
        y_ = transform.getOrigin().y();
        z_ = transform.getOrigin().z();
        ox_ = transform.getRotation().getX();
        oy_ = transform.getRotation().getY();
        oz_ = transform.getRotation().getZ();
        ow_ = transform.getRotation().getW();
    }
}

