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

#ifndef QRCODE_DETECTION_H
#define QRCODE_DETECTION_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

namespace zbar
{
class ImageScanner;
}

namespace qrcode_detection
{

class QRCodeDetection
{
public:
  QRCodeDetection(ros::NodeHandle nh, ros::NodeHandle priv_nh);
  ~QRCodeDetection();

protected:
  void connectCb();
  void imageCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info);

private:
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_camera_;
  image_transport::Publisher pub_qrimage_;
  ros::Publisher pub_qrcodes_;

  boost::mutex connect_mutex_;

  zbar::ImageScanner *scanner_;
};

} // namespace qrcode_detection

#endif // QRCODE_DETECTION_H

