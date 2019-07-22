#include "euclidean_cluster_core.h"

EuClusterCore::EuClusterCore(ros::NodeHandle &nh)
{
    //
    //生成检测图
    seg_distance_ = {15, 30, 45, 60};
    cluster_distance_ = {0.5, 1.0, 1.5, 2.0, 2.5};

    //接收路径信息
    sub_path_ = nh.subscribe("/path", 5, &EuClusterCore::path_point, this);
    sub_point_cloud_ = nh.subscribe("/filtered_points_no_ground", 5, &EuClusterCore::point_cb, this);

    pub_bounding_boxs_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detected_bounding_boxs", 5);
    pub_path_length_ = nh.advertise<std_msgs::Int8>("/pub_path_length", 5);
    ros::spin();
}

EuClusterCore::~EuClusterCore() {}

void EuClusterCore::publish_cloud(const ros::Publisher &in_publisher,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                                  const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}

void EuClusterCore::voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(in);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(*out);
}

void EuClusterCore::cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                                    double in_max_cluster_distance, std::vector<Detected_Obj> &obj_list)
{

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    //create 2d pc
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_pc, *cloud_2d);
    //make it flat
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
    {
        cloud_2d->points[i].z = 0;
    }

    if (cloud_2d->points.size() > 0)
        tree->setInputCloud(cloud_2d);

    std::vector<pcl::PointIndices> local_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
    euclid.setInputCloud(cloud_2d);
    euclid.setClusterTolerance(in_max_cluster_distance);
    euclid.setMinClusterSize(MIN_CLUSTER_SIZE);
    euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
    euclid.setSearchMethod(tree);
    euclid.extract(local_indices);

    int block_num = 0;

    for (size_t i = 0; i < local_indices.size(); i++)
    {
        // the structure to save one detected object
        Detected_Obj obj_info;

        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();

        for (auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit)
        {
            //fill new colored cluster point by point
            pcl::PointXYZ p;
            p.x = in_pc->points[*pit].x;
            p.y = in_pc->points[*pit].y;
            p.z = in_pc->points[*pit].z;

            obj_info.centroid_.x += p.x;
            obj_info.centroid_.y += p.y;
            obj_info.centroid_.z += p.z;

            if (p.x < min_x)
                min_x = p.x;
            if (p.y < min_y)
                min_y = p.y;
            if (p.z < min_z)
                min_z = p.z;
            if (p.x > max_x)
                max_x = p.x;
            if (p.y > max_y)
                max_y = p.y;
            if (p.z > max_z)
                max_z = p.z;
        }

        //min, max points
        obj_info.min_point_.x = min_x;
        obj_info.min_point_.y = min_y;
        obj_info.min_point_.z = min_z;

        obj_info.max_point_.x = max_x;
        obj_info.max_point_.y = max_y;
        obj_info.max_point_.z = max_z;

        //calculate centroid, average
        if (local_indices[i].indices.size() > 0)
        {
            obj_info.centroid_.x /= local_indices[i].indices.size();
            obj_info.centroid_.y /= local_indices[i].indices.size();
            obj_info.centroid_.z /= local_indices[i].indices.size();
        }

        //calculate bounding box
        double length_ = obj_info.max_point_.x - obj_info.min_point_.x;
        double width_ = obj_info.max_point_.y - obj_info.min_point_.y;
        double height_ = obj_info.max_point_.z - obj_info.min_point_.z;

        obj_info.bounding_box_.header = point_cloud_header_;

        obj_info.bounding_box_.pose.position.x = obj_info.min_point_.x + length_ / 2;
        obj_info.bounding_box_.pose.position.y = obj_info.min_point_.y + width_ / 2;
        obj_info.bounding_box_.pose.position.z = obj_info.min_point_.z + height_ / 2;

        obj_info.bounding_box_.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);
        obj_info.bounding_box_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
        obj_info.bounding_box_.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);

        obj_list.push_back(obj_info);
    }
}

void EuClusterCore::cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<Detected_Obj> &obj_list)
{
    //cluster the pointcloud according to the distance of the points using different thresholds (not only one for the entire pc)
    //in this way, the points farther in the pc will also be clustered

    //0 => 0-15m d=0.5
    //1 => 15-30 d=1
    //2 => 30-45 d=1.5
    //3 => 45-60 d=2.0
    //4 => >60   d=2.5

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_pc_array(5);

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        segment_pc_array[i] = tmp;
    }

    for (size_t i = 0; i < in_pc->points.size(); i++)
    {
        pcl::PointXYZ current_point;
        current_point.x = in_pc->points[i].x;
        current_point.y = in_pc->points[i].y;
        current_point.z = in_pc->points[i].z;

        float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

        // 如果点的距离大于60m, 忽略该点
        //if (origin_distance >= 120)
        if (origin_distance >= 60)
        {
            continue;
        }

        if (origin_distance < seg_distance_[0])
        {
            segment_pc_array[0]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[1])
        {
            segment_pc_array[1]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[2])
        {
            segment_pc_array[2]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[3])
        {
            segment_pc_array[3]->points.push_back(current_point);
        }
        else
        {
            segment_pc_array[4]->points.push_back(current_point);
        }
    }

    std::vector<pcl::PointIndices> final_indices;
    std::vector<pcl::PointIndices> tmp_indices;

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    // for (size_t i = 0; i < 1; i++)  //仅输出最近处点云
    {
        cluster_segment(segment_pc_array[i], cluster_distance_[i], obj_list);
    }
}

//接收路径信息
/*
void EuClusterCore::path_point(nav_msgs::Path path)
{
    tf::TransformListener listener(ros::Duration(0.01));
    geometry_msgs::PointStamped path_point_temp1;
    geometry_msgs::PointStamped path_point_temp2;
	path_point_temp1.header.frame_id = path.header.frame_id;
	path_point_temp2.header.frame_id = path.header.frame_id;
	path_point_temp1.header.stamp = path.header.stamp;
    path_point_temp2.header.stamp = path.header.stamp;

    for(int i = 0; i < path.poses.size(); i++)
    {
        path_point_temp1.point.x = path.poses[i].pose.position.x;
        path_point_temp1.point.y = path.poses[i].pose.position.y;
        path_point_temp1.point.z = path.poses[i].pose.position.z;

        listener.transformPoint("/velodyne", path_point_temp1, path_point_temp2);

        path_.poses[i].pose.position.x = path_point_temp2.point.x;
        path_.poses[i].pose.position.y = path_point_temp2.point.y;
        path_.poses[i].pose.position.z = path_point_temp2.point.z;
    }
}
*/
void EuClusterCore::path_point(nav_msgs::Path path)
{
    Tf_Listerner tf("/velodyne", "/world");
    auto ox = tf.ox();
    auto oy = tf.oy();
    auto oz = tf.oz();
    auto ow = tf.ow();
    tf::Quaternion q2(ox, oy, oz, ow);
    tf::Matrix3x3 Matrix;
    tf::Vector3 v3, v4, v5;
    Matrix.setRotation(q2);
    v3[0] = tf.x();
    v3[1] = tf.y();
    v3[2] = tf.z();

    for(int i = 0; i < path.poses.size(); i++)
    {
        v4[0] = path.poses[i].pose.position.x;
        v4[1] = path.poses[i].pose.position.y;
        v4[2] = path.poses[i].pose.position.z;
        v5 = Matrix * v4 + v3;
        path_.poses[i].pose.position.x = v5[0];
        path_.poses[i].pose.position.y = v5[1];
        path_.poses[i].pose.position.z = v5[2];
    }
}

//计算点云中的点与路径点距离
float EuClusterCore::dis_to_path(float barr_x, float barr_y)
{
    //确认path是否接收到 
    if(!path_.poses.size())
    {
        ROS_INFO("NO Path!");
        return 1000;
    }

    std::vector<float> path_length;
    for (int i = 0; i < path_.poses.size(); i++)
    {
        if(i == 0)
            path_length[i] += pow((pow(path_.poses[i].pose.position.x, 2) + pow(path_.poses[i].pose.position.y, 2)), 0.5);
        else
            path_length[i] += pow((pow(path_.poses[i].pose.position.x - path_.poses[i - 1].pose.position.x, 2) + pow(path_.poses[i].pose.position.y - path_.poses[i - 1].pose.position.y, 2)), 0.5);
    }   

    float min_point_dis = 10000;
    float dis_temp = 0;
 
    for (int i = 0; i < path_.poses.size(); i++)
    {
        dis_temp = pow(barr_x - path_.poses[i].pose.position.x, 2) + pow(barr_y - path_.poses[i].pose.position.y, 2);
        if(dis_temp > 4)
            continue;
        if (min_point_dis > dis_temp + path_length[i])
        {
            min_point_dis = dis_temp;
        }
    }
    return min_point_dis;
}

//路径上的障碍物
float EuClusterCore::path_barrier(std::vector<Detected_Obj> &obj_list)
{
    float dis_path = 1000;
    for (size_t i = 0; i < obj_list.size(); i++)
    {
        float mid_x = obj_list[i].bounding_box_.pose.position.x;
        float mid_y = obj_list[i].bounding_box_.pose.position.y;

        float cloud_x_dis = obj_list[i].bounding_box_.dimensions.x;
        float cloud_y_dis = obj_list[i].bounding_box_.dimensions.y;

        float min_x = mid_x - cloud_x_dis / 2;
        float max_x = mid_x + cloud_x_dis / 2;
        float min_y = mid_y - cloud_y_dis / 2;
        float max_y = mid_y + cloud_y_dis / 2;

        if (dis_path > dis_to_path(mid_x, min_y))
            dis_path = dis_to_path(mid_x, min_y);
        if (dis_path > dis_to_path(min_x, min_y))
            dis_path = dis_to_path(min_x, min_y);
        if (dis_path > dis_to_path(min_x, mid_y))
            dis_path = dis_to_path(min_x, mid_y);
        if (dis_path > dis_to_path(min_x, max_y))
            dis_path = dis_to_path(min_x, max_y);
        if (dis_path > dis_to_path(mid_x, max_y))
            dis_path = dis_to_path(mid_x, max_y);
    }
    return dis_path;
}

void EuClusterCore::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    point_cloud_header_ = in_cloud_ptr->header;

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    // down sampling the point cloud before cluster
    voxel_grid_filer(current_pc_ptr, filtered_pc_ptr, LEAF_SIZE);

    std::vector<Detected_Obj> global_obj_list;
    cluster_by_distance(filtered_pc_ptr, global_obj_list);

    jsk_recognition_msgs::BoundingBoxArray bbox_array;

    std_msgs::Int8 dis;
    int path_dis = 0;
    path_dis = path_barrier(global_obj_list);
    dis.data = path_dis;
    ROS_INFO("The min dis is %d", path_dis);
    pub_path_length_.publish(dis);

    for (size_t i = 0; i < global_obj_list.size(); i++)
    {
        bbox_array.boxes.push_back(global_obj_list[i].bounding_box_);
    }
    bbox_array.header = point_cloud_header_;

    pub_bounding_boxs_.publish(bbox_array);
}