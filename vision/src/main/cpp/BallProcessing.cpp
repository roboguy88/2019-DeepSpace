/*
#include "Display.h"
#include "Capture.h"
#include "BallProcessing.h"

#include <opencv2/opencv.hpp>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <stdio.h>
#include <iostream>

#include <cameraserver/CameraServer.h>
#include <cscore.h>

#include "networktables/NetworkTableInstance.h"

#include "devices/kinect.h"

cv::RNG rngBall(12345);
cv::Rect ball_bounding_rect;
int ball_thresh = 100;
float ball_height_offset;
float ball_width_offset;
float ball_width_goal = 320;
float ball_height_goal = 240;
std::string Ball_Distance = "Sumthin";


void BallProcessing::Init() {
  Process::Init();
  processType = "BallProcessing";

  auto inst = nt::NetworkTableInstance::GetDefault();
  auto visionTable = inst.GetTable("VisionTable");
  auto table = visionTable->GetSubTable("TapeTable");
  BallDistanceEntry = table->GetEntry("Hatch Distance");
  BallXoffsetEntry = table->GetEntry("Hatch X Offset");
  BallYoffsetEntry = table->GetEntry("Hatch Y Offset");
}

void BallProcessing::Periodic() {
  Process::Periodic();
  if (_capture.IsValidFrame()) {

    _capture.CopyCaptureMat(_imgProcessing);
    cv::cvtColor(_imgProcessing, _imgProcessing, cv::COLOR_BGR2HSV);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> filteredContoursBall;
    std::vector<std::vector<cv::Point>> filteredHullsBall;
    std::vector<cv::Rect> ir_rects;
    int active_contour;
    bool show_window;

    double largestArea = 0.0;
    active_contour = -1;
    // Filters Colour for Reflective Ball
    cv::inRange(_imgProcessing, cv::Scalar(0, 100, 100), cv::Scalar(100, 255, 255), _imgProcessing);
    cv::findContours(_imgProcessing, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);
    // Filters Size For Ball
    for (int i = 0; i < contours.size(); i++) {
      std::vector<cv::Point> contour = contours[i];
      cv::Rect r = cv::boundingRect(contour);
      
      double area = cv::contourArea(contour);
      if (area > 300.0) {
        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        double solidity = 100 * area / contourArea(hull);

        if (solidity < 60.0) {
          if (area > largestArea) {
            largestArea = area;
            active_contour = filteredContoursBall.size();
          }
          filteredContoursBall.push_back(contour);
          filteredHullsBall.push_back(hull);
          ir_rects.push_back(r);
        }
      }
    }
    /// Detect edges using Canny
    cv::Canny(_imgProcessing, _imgProcessing, ball_thresh, ball_thresh * 2);

    /// Find contours
    std::vector<cv::Vec4i> hierarchy;

    /// Find the convex hull object for each contour
    std::vector<std::vector<cv::Point>> hullBall(filteredContoursBall.size());
    for (size_t i = 0; i < filteredContoursBall.size(); i++) {
      cv::convexHull(filteredContoursBall[i], hullBall[i]);
    }

    /// Draw filteredContours + hull results
    _imgProcessing = cv::Mat::zeros(_imgProcessing.size(), CV_8UC3);
    std::vector<cv::Rect> boundRectBall( filteredContoursBall.size() );

    _imgProcessedTrack = cv::Mat::zeros(_videoMode.height, _videoMode.width, CV_8UC3);
    for (size_t i = 0; i < filteredContoursBall.size(); i++) {
      cv::Scalar color = cv::Scalar(rngBall.uniform(0, 256), rngBall.uniform(0, 256), rngBall.uniform(0, 256));
      cv::drawContours(_imgProcessedTrack, filteredContoursBall, (int)i, color);
      cv::drawContours(_imgProcessedTrack, hullBall, (int)i, color);
    }

    for (size_t i = 0; i < filteredContoursBall.size(); i++) {
      cv::Scalar color = cv::Scalar(rngBall.uniform(0, 256), rngBall.uniform(0, 256), rngBall.uniform(0, 256));
      cv::drawContours(_imgProcessedTrack, filteredContoursBall, (int)i, color);
      cv::drawContours(_imgProcessedTrack, hullBall, (int)i, color);
    }

    /// Find contoursBox
    /// Approximate contoursBox to polygons + get bounding rects and circles
    std::vector<std::vector<cv::Point>> hullBall_poly(hullBall.size());
    std::vector<cv::Point2f> centerBall(hullBall.size());
    std::vector<float> radiusBall(hullBall.size());

    for(int i = 0; i < hullBall.size(); i++) {
      approxPolyDP(cv::Mat(hullBall[i]), hullBall_poly[i], 3, true);
      boundRectBall[i] = cv::boundingRect(cv::Mat(hullBall_poly[i]));
      cv::minEnclosingCircle((cv::Mat)hullBall_poly[i], centerBall[i], radiusBall[i]);
    }


    /// Draw polygonal contour + bonding rects + circles
    for(int i = 0; i < hullBall.size(); i++) {
      cv::Scalar color = cv::Scalar(rngBall.uniform(0, 255), rngBall.uniform(0,255), rngBall.uniform(0,255));
      cv::drawContours(_imgProcessedTrack, hullBall_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
      ball_bounding_rect = cv::boundingRect(filteredContoursBall[i]); // Find the bounding rectangle for biggest contour
      cv::rectangle(_imgProcessedTrack, boundRectBall[i].tl(), boundRectBall[i].br(), color, 2, 8, 0);
      cv::circle(_imgProcessedTrack, centerBall[i], (int)radiusBall[i], color, 2, 8, 0);
    }

    //_____________________Center Calculations__________________

    std::vector<cv::Moments> muBall(hullBall_poly.size()); // do we need this if we have mutex ? *
    for(int i = 0; i < hullBall_poly.size(); i++) {
      muBall[i] = moments(hullBall_poly[i], false);
    }

    // get the centroid of figures.
    std::vector<cv::Point2f> mcBall(hullBall_poly.size());
    for(int i = 0; i < hullBall_poly.size(); i++) {
      mcBall[i] = cv::Point2f(muBall[i].m10/muBall[i].m00 , muBall[i].m01/muBall[i].m00);
    }

    for(int i = 0; i < hullBall_poly.size(); i++) {
      cv::Scalar color = cv::Scalar(167,151,0); // B G R values
      cv::circle(_imgProcessedTrack, mcBall[i], 4, color, -1, 8, 0);

      // offsets from centerBall
      cv::Point centerBall = cv::Point((mcBall[i].x), (mcBall[i].y));
      ball_width_offset = ball_width_goal - centerBall.x;
      ball_height_offset = ball_height_goal - centerBall.y;
      std::cout << "Offset From CenterBall x,y =" << ball_width_offset << "," << ball_height_offset << std::endl; // height is x ?
      BallDistanceEntry.SetString(Ball_Distance);
      BallXoffsetEntry.SetDouble(ball_width_offset);
      BallYoffsetEntry.SetDouble(ball_height_offset);
    }
  }
}
*/