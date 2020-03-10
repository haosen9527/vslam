#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>


using namespace cv;
using namespace std;

Mat img1, img2;
void ORB_demo(int, void*);
int main(int argc, char** argv)
{


   img1 = imread("/home/haosen/下载/912525731.jpg",1);
   img2 = imread("/home/haosen/下载/1383125660.jpg",1);
  if (!img1.data|| !img2.data)
  {
    cout << "图片未找到！" << endl;
    return -1;
  }
  namedWindow("ORB_demo",CV_WND_PROP_FULLSCREEN);

  ORB_demo(0,0);

//  imshow("input image of box",img1);
//  imshow("input image of box_in_scene", img2);


  waitKey(0);
  return 0;


}

/*---------------检测与匹配--------------*/
void ORB_demo(int, void *)
{
  int Hession = 1000;
  double t1 = getTickCount();
  //特征点提取
  Ptr<ORB> detector = ORB::create(Hession);
  vector<KeyPoint> keypoints_obj;
  vector<KeyPoint> keypoints_scene;
  //定义描述子
  Mat descriptor_obj, descriptor_scene;
  //检测并计算成描述子
  detector->detectAndCompute(img1, Mat(), keypoints_obj, descriptor_obj);
  detector->detectAndCompute(img2, Mat(), keypoints_scene, descriptor_scene);

  double t2 = getTickCount();
  double t = (t2 - t1) * 1000 / getTickFrequency();
  //特征匹配
  FlannBasedMatcher fbmatcher(new flann::LshIndexParams(20, 10, 2));
  vector<DMatch> matches;
  //将找到的描述子进行匹配并存入matches中
  fbmatcher.match(descriptor_obj, descriptor_scene, matches);

  double minDist = 1000;
  double maxDist = 0;
  //找出最优描述子
  vector<DMatch> goodmatches;
  for (int i = 0; i < descriptor_obj.rows; i++)
  {
    double dist = matches[i].distance;
    if (dist < minDist)
    {
      minDist=dist ;
    }
    if (dist > maxDist)
    {
      maxDist=dist;
    }

  }
  for (int i = 0; i < descriptor_obj.rows; i++)
  {
    double dist = matches[i].distance;
    if (dist < max(2 * minDist, 0.02))
    {
      goodmatches.push_back(matches[i]);
    }
  }
  Mat orbImg;

  drawMatches(img1, keypoints_obj, img2, keypoints_scene, goodmatches, orbImg,
    Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  //----------目标物体用矩形标识出来------------
  vector<Point2f> obj;
  vector<Point2f>scene;
  for (size_t i = 0; i < goodmatches.size(); i++)
  {
    obj.push_back(keypoints_obj[goodmatches[i].queryIdx].pt);
    scene.push_back(keypoints_scene[goodmatches[i].trainIdx].pt);
  }
  vector<Point2f> obj_corner(4);
  vector<Point2f> scene_corner(4);
  //生成透视矩阵
  Mat H = findHomography(obj, scene, RANSAC);

  obj_corner[0] = Point(0, 0);
  obj_corner[1] = Point(img1.cols, 0);
  obj_corner[2] = Point(img1.cols, img1.rows);
  obj_corner[3] = Point(0, img1.rows);
  //透视变换
  perspectiveTransform(obj_corner, scene_corner, H);
  Mat resultImg=orbImg.clone();


  for (int i = 0; i < 4; i++)
  {
    line(resultImg, scene_corner[i]+ Point2f(img1.cols, 0), scene_corner[(i + 1) % 4]+ Point2f(img1.cols, 0), Scalar(0, 0, 255), 2, 8, 0);
  }

  //imshow("result image",resultImg);

  cout << "ORB执行时间为:" << t << "ms" << endl;
  cout << "最小距离为：" <<minDist<< endl;
  cout << "最大距离为：" << maxDist << endl;
  imshow("ORB_demo", orbImg);
}
