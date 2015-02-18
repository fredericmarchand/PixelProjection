#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>

std::vector<cv::Point3d> Generate3DPoints();

int main( int argc, char* argv[])
{
  // Read points
  std::vector<cv::Point3d> objectPoints = Generate3DPoints();
  
  cv::Mat K(3,3,cv::DataType<double>::type); // intrinsic parameter matrix
  K.at<double>(0,0) = -500;
  K.at<double>(1,0) = 0;
  K.at<double>(2,0) = 0;

  K.at<double>(0,1) = 0;
  K.at<double>(1,1) = -500;
  K.at<double>(2,1) = 0;

  K.at<double>(0,2) = 320;
  K.at<double>(1,2) = 240;
  K.at<double>(2,2) = 1;

  cv::Mat R(3,3,cv::DataType<double>::type); // rotation matrix
  R.at<double>(0,0) = 0;
  R.at<double>(1,0) = 1;
  R.at<double>(2,0) = 0;

  R.at<double>(0,1) = 1;
  R.at<double>(1,1) = 0;
  R.at<double>(2,1) = 0;

  R.at<double>(0,2) = 0;
  R.at<double>(1,2) = 0;
  R.at<double>(2,2) = 1;

  cv::Mat T(3,1,cv::DataType<double>::type); // translation vector
  T.at<double>(0,0) = 70;
  T.at<double>(0,1) = 95;
  T.at<double>(0,2) = 120;

  std::cout << "K: " << K << std::endl;
  std::cout << "R: " << R << std::endl;
  std::cout << "T: " << T << std::endl;
  
  // Create zero distortion
  cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
  distCoeffs.at<double>(0) = 0;
  distCoeffs.at<double>(1) = 0;
  distCoeffs.at<double>(2) = 0;
  distCoeffs.at<double>(3) = 0;
  
  std::vector<cv::Point2d> projectedPoints;

  cv::Mat rvecR(3,1,cv::DataType<double>::type);//rodrigues rotation matrix
  cv::Rodrigues(R,rvecR);

  cv::projectPoints(objectPoints, rvecR, T, K, distCoeffs, projectedPoints);

  for(unsigned int i = 0; i < projectedPoints.size(); ++i)
    {
    std::cout << "World point: " << objectPoints[i] << " Projected to " << projectedPoints[i] << std::endl;
    }
  std::getchar();
  return 0;
}

std::vector<cv::Point3d> Generate3DPoints()
{
  std::vector<cv::Point3d> points;

  double x,y,z;

  x=150;y=200;z=350;
  points.push_back(cv::Point3d(x,y,z));


  return points;
}
