/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include "System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    void rgbd_Vo(cv::Mat Tcw);

    ORB_SLAM2::System* mpSLAM;
    ros::Publisher* pPosPub;
    ros::Publisher* cameraPath;
    ros::Publisher* odomVo;
    nav_msgs::Path camera_path;
    tf::TransformBroadcaster odomBr;
    tf::Transform transform;
};

void ImageGrabber::rgbd_Vo(cv::Mat Tcw)
{
  geometry_msgs::PoseStamped poseMSG;
      if(!Tcw.empty())
      {

          cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
          cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

          vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

          camera_path.header.frame_id = "camera_link";
          poseMSG.pose.position.x = twc.at<float>(0);
          poseMSG.pose.position.y = twc.at<float>(2);
          poseMSG.pose.position.z = twc.at<float>(1);
          poseMSG.pose.orientation.x = q[0];
          poseMSG.pose.orientation.y = q[1];
          poseMSG.pose.orientation.z = q[2];
          poseMSG.pose.orientation.w = q[3];
          poseMSG.header.frame_id = "camera_pose";
          poseMSG.header.stamp = ros::Time::now();
          cout << "PublishPose position.x = " << poseMSG.pose.position.x << poseMSG.pose.position.y << poseMSG.pose.position.z << endl;
          camera_path.poses.push_back(poseMSG);


          ros::Time currentTime = ros::Time::now();

          //publisher
          geometry_msgs::TransformStamped odomTrans;
          odomTrans.header.stamp = currentTime;
          odomTrans.header.frame_id = "odom";
          odomTrans.child_frame_id = "camera_link";

          odomTrans.transform.translation.x = twc.at<float>(0);
          odomTrans.transform.translation.y = twc.at<float>(2);
          odomTrans.transform.translation.z = twc.at<float>(1);
          //set quaternion
          odomTrans.transform.rotation.w = q[3];
          odomTrans.transform.rotation.x = q[0];
          odomTrans.transform.rotation.y = q[1];
          odomTrans.transform.rotation.z = q[2];
          //send transform
          odomBr.sendTransform(odomTrans);

          nav_msgs::Odometry odom;
          odom.header.stamp = currentTime;
          odom.header.frame_id = "odom";
          odom.child_frame_id = "camera_link";

          //set position
          odom.pose.pose.position.x = twc.at<float>(0);
          odom.pose.pose.position.y = twc.at<float>(2);
          odom.pose.pose.position.z = twc.at<float>(1);
          //set velocity
          odom.twist.twist.linear.x = 0;
          odom.twist.twist.linear.y = 0;
          odom.twist.twist.linear.z = 0;

          (odomVo)->publish(odom);
          (cameraPath)->publish(camera_path);
          (pPosPub)->publish(poseMSG);
      }
}
void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose =  mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    ROS_INFO("Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" );

    std::string path_to_vocabulary = ros::package::getPath("orb_slam2_ros")+"/Vocabulary/ORBvoc.bin";
    std::cout<<  path_to_vocabulary << std::endl;
    std::string path_to_settings = ros::package::getPath("orb_slam2_ros")+"/config/astra.yaml";
    bool PureLocalization = true;

    ros::param::get("path_to_vocabulary",path_to_vocabulary);
    ros::param::get("path_to_settings",path_to_settings);
    ros::param::get("PureLocalization",PureLocalization);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(path_to_vocabulary,path_to_settings,ORB_SLAM2::System::RGBD,true,PureLocalization);

    ImageGrabber igb(&SLAM);
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth/image_rect_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::Publisher PosPub = nh.advertise<geometry_msgs::PoseStamped>("camera/pose", 5);
    ros::Publisher camerapath = nh.advertise<nav_msgs::Path>("camera_path",1);
    ros::Publisher odom_vo = nh.advertise<nav_msgs::Odometry>("odom",50);

    igb.pPosPub = &(PosPub);
    igb.cameraPath = &(camerapath);
    igb.odomVo = &(odom_vo);

    ros::spin();
    // Stop all threads
    SLAM.Shutdown();
    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    std::string map_path = ros::package::getPath("orb_slam2_ros")+"/map/MapPointandKeyFrame.bin";

    if(PureLocalization == false)
    {
      SLAM.SaveMap(map_path);
    }

    ros::shutdown();

    return 0;
}




