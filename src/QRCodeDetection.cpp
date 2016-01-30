/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2015, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "QRCodeDetection.h"
#include <qrcode_detection/QRCodeArray.h>

#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <zbar.h>

#include <tf/LinearMath/Vector3.h>


namespace qrcode_detection
{

QRCodeDetection::QRCodeDetection(ros::NodeHandle nh, ros::NodeHandle priv_nh) :
    it_(nh)
{
  scanner_ = new zbar::ImageScanner;
  scanner_->set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

  ros::SubscriberStatusCallback connect_cb_ros = boost::bind(&QRCodeDetection::connectCb, this);
  image_transport::SubscriberStatusCallback connect_cb_it = boost::bind(&QRCodeDetection::connectCb, this);
  pub_qrcodes_ = nh.advertise<qrcode_detection::QRCodeArray>("qrcodes", 10, connect_cb_ros, connect_cb_ros);
  pub_qrimage_ = it_.advertise("qrcode_img", 2, connect_cb_it, connect_cb_it);

  unsigned major, minor;
  zbar::zbar_version(&major, &minor);
  ROS_INFO("Detecting QR Codes with zbar %d.%d", major, minor);
}

QRCodeDetection::~QRCodeDetection()
{
}

void QRCodeDetection::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (!pub_qrcodes_.getNumSubscribers() && !pub_qrimage_.getNumSubscribers()) {
    sub_camera_.shutdown();
  } else if (!sub_camera_) {
    sub_camera_ = it_.subscribeCamera("image_raw", 2, &QRCodeDetection::imageCallback, this);
  }
}

void QRCodeDetection::imageCallback(const sensor_msgs::ImageConstPtr& image,
                                    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  ros::WallTime tic = ros::WallTime::now();

  cv_bridge::CvImageConstPtr cv_image;
  if (image->encoding == "mono8") {
	  cv_image = cv_bridge::toCvShare(image, "mono8");
  } else {
	  cv_image = cv_bridge::toCvCopy(image, "mono8");
  }

  // Wrap image data
  zbar::Image zbar(cv_image->image.cols, cv_image->image.rows, "Y800", cv_image->image.data,
                   cv_image->image.cols * cv_image->image.rows);

  // Scan the image for QR Codes
  scanner_->scan(zbar);

  ros::WallTime toc = ros::WallTime::now();
  ROS_DEBUG("zbar completed in %03ums", (unsigned int)((toc - tic).toNSec() / 1000000));

  // Extract results
  qrcode_detection::QRCodeArray msg;
  msg.header = image->header;
  msg.camera_info = *camera_info;

  for (zbar::Image::SymbolIterator symbol = zbar.symbol_begin(); symbol != zbar.symbol_end(); ++symbol) {

    if (symbol->get_location_size() != 4) {
      ROS_WARN("Could not get symbol locations (location_size != 4)");
      continue;
    }

    float center_x = (float)(symbol->get_location_x(0) + symbol->get_location_x(1) + symbol->get_location_x(2)
        + symbol->get_location_x(3)) / 4.0;
    float center_y = (float)(symbol->get_location_y(0) + symbol->get_location_y(1) + symbol->get_location_y(2)
        + symbol->get_location_y(3)) / 4.0;
    tf::Vector3 v0(symbol->get_location_y(1) - symbol->get_location_y(0),
                   symbol->get_location_x(1) - symbol->get_location_x(0), 0.0);
    tf::Vector3 v1(symbol->get_location_x(2) - symbol->get_location_x(1),
                   symbol->get_location_y(1) - symbol->get_location_y(2), 0.0);
    tf::Vector3 v2(symbol->get_location_y(2) - symbol->get_location_y(3),
                   symbol->get_location_x(2) - symbol->get_location_x(3), 0.0);
    tf::Vector3 v3(symbol->get_location_x(3) - symbol->get_location_x(0),
                   symbol->get_location_y(0) - symbol->get_location_y(3), 0.0);
    float width = (v1.length() + v3.length()) / 2;
    float height = (v0.length() + v2.length()) / 2;
    tf::Vector3 vsum = v0.normalize() + v1.normalize() + v2.normalize() + v3.normalize();
    float angle = atan2(vsum.y(), vsum.x());


    ROS_INFO("(%0.1f, %0.1f, %0.2f), (%d,%d) (%d,%d) (%d,%d) (%d,%d), '%s'",
             center_x, center_y, angle,
             symbol->get_location_x(0), symbol->get_location_y(0),
             symbol->get_location_x(1), symbol->get_location_y(1),
             symbol->get_location_x(2), symbol->get_location_y(2),
             symbol->get_location_x(3), symbol->get_location_y(3),
             symbol->get_data().c_str());

    qrcode_detection::QRCode code;
    code.data = symbol->get_data();
    code.center_x = center_x;
    code.center_y = center_y;
    code.angle = angle;
    code.width = width;
    code.height = height;
    code.ptx[0] = symbol->get_location_x(0);
    code.ptx[1] = symbol->get_location_x(1);
    code.ptx[2] = symbol->get_location_x(2);
    code.ptx[3] = symbol->get_location_x(3);
    code.pty[0] = symbol->get_location_y(0);
    code.pty[1] = symbol->get_location_y(1);
    code.pty[2] = symbol->get_location_y(2);
    code.pty[3] = symbol->get_location_y(3);
    msg.codes.push_back(code);

    if (pub_qrimage_.getNumSubscribers() > 0) {
      // point order is left/top, left/bottom, right/bottom, right/top
      int min_x = INT32_MAX, min_y = INT32_MAX, max_x = INT32_MIN, max_y = INT32_MIN;
      for (int i = 0; i < 4; ++i) {
        if (symbol->get_location_x(i) > max_x)
          max_x = symbol->get_location_x(i);
        if (symbol->get_location_x(i) < min_x)
          min_x = symbol->get_location_x(i);
        if (symbol->get_location_y(i) > max_y)
          max_y = symbol->get_location_y(i);
        if (symbol->get_location_y(i) < min_y)
          min_y = symbol->get_location_y(i);
      }
      try {
        cv::Rect rect(cv::Point2i(std::max(min_x, 0), std::max(min_y, 0)),
                      cv::Point2i(std::min(max_x, cv_image->image.cols), std::min(max_y, cv_image->image.rows)));
        cv_bridge::CvImagePtr qrcode_cv(new cv_bridge::CvImage(*cv_image));
        qrcode_cv->image = cv_image->image(rect);

        sensor_msgs::Image msg;
        qrcode_cv->toImageMsg(msg);
        pub_qrimage_.publish(msg);
      } catch (cv::Exception& e) {
        ROS_ERROR("cv::Exception: %s", e.what());
      }
    }
  }
  pub_qrcodes_.publish(msg);

  // clean up
  zbar.set_data(NULL, 0);
}

} // namespace qrcode_detection
