// Copyright 2024 Stuart Johnson
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Stuart Johnson nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef IPS_CAM__IMAGE_PROCESSOR_HPP_
#define IPS_CAM__IMAGE_PROCESSOR_HPP_

#include <limits.h>
#include <unistd.h>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace ips_cam
{

struct CameraIntrinsics
{
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  // todo: fix this - these need to come from the file
  int image_height = 720;    // or height
  int image_width = 1280;    // or width
};

struct IcsParams
{
  std::string intrinsics_file;
  std::string checkerboard_image_file;
  std::string origin_image_file;
  int cbExtentX;
  int cbExtentY;
  double cbBlockSize;
  std::map<int, double> originTag;
};

struct TrackingParams
{
  std::vector<int> tag;
  std::vector<double> tag_z;
  std::map<int, double> tag_lookup;
};

struct TagPose
{
  // The pose of an aruco tag in the world coordinate system.
  int tag;
  double theta;    // radians
  double x;    // world coordinate units (e.g., mm)
  double y;    // world coordinate units (e.g., mm)
  double z;    // world coordinate units - this value is known via settings
  friend std::ostream & operator<<(std::ostream & os, const TagPose & tagPose)
  {
    double angleDeg = tagPose.theta * 180.0 / 3.14159265358979323846;
    os << "tag: " << tagPose.tag << " angle (deg): " << angleDeg <<
      " loc: (" << tagPose.x << "," << tagPose.y << "," << tagPose.z << ")";
    return os;
  }
};

struct MarkerDetections
{
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
};

// utilities for load confugurations. I have kept these seperate
// from ros2 node params for now. Each of the 3 path names here
// should be present in the ros2 node parameters - I think.

void check_file_existence(std::string filename);

std::string expand_and_check(std::string filename);

CameraIntrinsics load_camera_intrinsics(std::string filename);

IcsParams load_ics_params(std::string filename);

TrackingParams load_tracking_params(std::string filename);

void load_thingy(std::string filename);

// basic camera coord system tools

// convert to a column-vector matrix
cv::Mat toHomoMat(std::vector<cv::Point2f>);

cv::Mat toHomo(cv::Mat columnVectors);

cv::Mat fromHomo(cv::Mat homoColumnVectors);

void homoDivide(cv::Mat homoColumnVectors);

cv::Mat extractColumn(cv::Mat inputMatrix, int col);

cv::Mat extractRow(cv::Mat inputMatrix, int row);

cv::Mat removeColumn(cv::Mat inputMatrix, int col);

// more advanced camera tools

cv::Mat composeCameraExtrinsicMatrix(cv::Mat rvec, cv::Mat tvec);

cv::Mat composeCameraExtrinsicMatrixFull(cv::Mat rvec, cv::Mat tvec);

cv::Mat composeExtrinsicFlip(int cbExtentX, int cbExtentY, double cbBlockMeters);

struct IndoorCoordSystem
{
  // The intrinsics of the camera are useful for many downstream
  // applications, so including it here
  CameraIntrinsics cameraIntrinsics;

  // an indoor coordinate system is defined by two images:
  // 1) a chessboard image of known dimensions.
  // 2) an origin image containing an aruco marker near the position
  //     corresponding to (X,Y,Z) = (0,0,0).
  //
  cv::Mat rvec;
  cv::Mat tvec;
  cv::Mat extMat;
  cv::Mat extMatFull;
  cv::Mat extFlip;
  // the indoor coordinate system:
  // 1) is right handed with Z pointing up. Z=0 is in the
  //     plane of the chessboard.
  // 2) is in units of meters.
  // 3) X points along the long dimension of the chessboard.
  // 4) (X,Y,Z) = 0 is near where the aruco marker in the the origin image.

  IndoorCoordSystem()
  {
    rvec = cv::Mat(3, 1, cv::DataType<double>::type);
    tvec = cv::Mat(3, 1, cv::DataType<double>::type);
  }

  void SetExtrinsics()
  {
    extMat = composeCameraExtrinsicMatrix(rvec, tvec);
    extMatFull = composeCameraExtrinsicMatrixFull(rvec, tvec);
  }

  void SetFlippedExtrinsics(int cbExtentX, int cbExtentY, double cbBlockMeters)
  {
    extMat = composeCameraExtrinsicMatrix(rvec, tvec);
    extMatFull = composeCameraExtrinsicMatrixFull(rvec, tvec);
    extFlip = composeExtrinsicFlip(cbExtentX, cbExtentY, cbBlockMeters);
    extMat = extMat * extFlip;
    extMatFull = extMatFull * extFlip;
  }
};

IndoorCoordSystem EstablishIndoorCoordinateSystem(
  int cbExtentX, int cbExtentY, double cbBlockMeters,
  CameraIntrinsics camIntrinsics,
  cv::Mat cbPatternImage, cv::Mat cbOriginImage, std::map<int, double> originTag);

IndoorCoordSystem EstablishIndoorCoordinateSystem(
  IcsParams icsParams
);

class ImagePointsToWorldPoints
{
public:
  // constructor
  // prepare to repeatedly transform numpoints points at ICS coord z into
  // world points
  ImagePointsToWorldPoints(IndoorCoordSystem ics, double z, int numPoints);

  ImagePointsToWorldPoints(IndoorCoordSystem ics, cv::Mat z, int numPoints);

  void Init(IndoorCoordSystem ics, cv::Mat z, int numPoints);

  cv::Mat ToWorld(cv::Mat imagePoints);

  // properties
  cv::Mat pxy_inverse;
  cv::Mat onesCol;
  cv::Mat scale_numerator;
  cv::Mat adj_xy_z;
};

class TagPoseEstimator
{
public:
  // constructor
  TagPoseEstimator();

  // properties
  cv::Mat rhs;
  cv::Mat estimatorMatrix;

  // estimate tag pose indicated by ordered world coords.
  TagPose estimate(cv::Mat worldCoords);
};

class ObjectTracker
{
public:
  // constructors
  ObjectTracker(IndoorCoordSystem ics, std::map<int, double> tags);

  // properties

  // the world coordinate system for all reporting
  IndoorCoordSystem ics;

  // z-coordinate of detection - there will eventually be one of these
  // for each tracked object.
  double z;

  // configuration
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  TagPoseEstimator tagPoseEstimator;

  std::map<int, ImagePointsToWorldPoints> imgToWorld;

  std::map<int, double> trackedTags;

  // methods
  MarkerDetections FindMarkers(cv::Mat inputImage);

  std::vector<TagPose> FindMarkerPoses(
    MarkerDetections detections,
    std::vector<cv::Mat> & worldCorners);

  void ImageToWorld(MarkerDetections detections, std::vector<cv::Mat> & worldCorners);

  std::vector<TagPose> Track(cv::Mat inputImage);
};

}  // namespace ips_cam

#endif  // IPS_CAM__IMAGE_PROCESSOR_HPP_
