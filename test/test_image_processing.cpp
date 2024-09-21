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


#include <limits.h>
#include <unistd.h>
#include <gtest/gtest.h>

#include <string>
#include <fstream>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <ips_cam/image_processor.hpp>

std::string getexepath()
{
  char result[PATH_MAX];
  ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
  return std::string(result, (count > 0) ? count : 0);
}

cv::Mat MakeWorldRectangleXY(cv::Mat location, cv::Mat extent)
{
  // location: 3d location of XY rectangle
  // extent: 2d extent of rectangle
  cv::Mat out(3, 4, cv::DataType<double>::type);
  cv::Mat zeroMat(1, 1, cv::DataType<double>::type, cv::Scalar(0.0));
  cv::Mat extentOver2 = extent / 2.0;
  double data[9] = {0, 1, 0, -1, 0, 0, 0, 0, 1};
  cv::Mat rotmat90(3, 3, cv::DataType<double>::type, data);
  cv::Mat current_vec;
  cv::vconcat(extentOver2, zeroMat, current_vec);
  for (int i = 0; i < 4; i++) {
    out.col(i) = current_vec + location;
    current_vec = rotmat90 * current_vec;
  }

  return out;
}

void functionThatThrows()
{
  throw std::runtime_error("This is an error");
}

ips_cam::IndoorCoordSystem SetupICS(int origin = 2)
{
  cv::Mat cbPatternImage = cv::imread("../../../im_ref2.png", cv::IMREAD_COLOR);
  cv::Mat cbOriginImage;
  if (origin == 1) {
    cbOriginImage = cv::imread("../../../im_ref2_aruco1.png", cv::IMREAD_COLOR);
  } else {
    cbOriginImage = cv::imread("../../../im_ref2_aruco2.png", cv::IMREAD_COLOR);
  }
  ips_cam::CameraIntrinsics camIntrinsics =
    ips_cam::load_camera_intrinsics("../../../camera_intrinsics.yml");

  std::map<int, double> originTag;
  originTag[1] = 0.0;

  ips_cam::IndoorCoordSystem ics = ips_cam::EstablishIndoorCoordinateSystem(
    6, 4, 198.0,
    camIntrinsics,
    cbPatternImage, cbOriginImage, originTag);

  return ics;
}

TEST(test_image_processing, test_ics_config)
{
  auto aruco1 = cv::imread("../../../im_ref2_aruco1.png", cv::IMREAD_COLOR);
  auto aruco2 = cv::imread("../../../im_ref2_aruco2.png", cv::IMREAD_COLOR);

  // use the configuration file to bring up the ics
  ips_cam::IcsParams ip =
    ips_cam::load_ics_params("~/IndoorPositioningSystem/ips_config/ics_params.yml");
  // TrackingParams tp =
  //    load_tracking_params("~/IndoorPositioningSystem/ips_config/tracking.yml");

  ips_cam::IndoorCoordSystem ics = ips_cam::EstablishIndoorCoordinateSystem(ip);

  // look for tag 1 at z=0
  std::map<int, double> tags;
  tags[1] = 0.0;

  // the expected results are somewhat subtle...

  std::cout << "ics1:" << std::endl;
  ips_cam::ObjectTracker tagFinder1 = ips_cam::ObjectTracker(ics, tags);
  auto poses11 = tagFinder1.Track(aruco1);
  std::cout << "aruco1: " << poses11[0] << std::endl;
  auto poses12 = tagFinder1.Track(aruco2);
  std::cout << "aruco2: " << poses12[0] << std::endl;
}


TEST(test_image_processing, test_ics)
{
  auto aruco1 = cv::imread("../../../im_ref2_aruco1.png", cv::IMREAD_COLOR);
  auto aruco2 = cv::imread("../../../im_ref2_aruco2.png", cv::IMREAD_COLOR);

  ips_cam::IndoorCoordSystem ics1 = SetupICS(1);

  // look for tag 1 at z=0
  std::map<int, double> tags;
  tags[1] = 0.0;

  // the expected results are somewhat subtle...

  std::cout << "ics1:" << std::endl;
  ips_cam::ObjectTracker tagFinder1 = ips_cam::ObjectTracker(ics1, tags);
  auto poses11 = tagFinder1.Track(aruco1);
  std::cout << "aruco1: " << poses11[0] << std::endl;
  auto poses12 = tagFinder1.Track(aruco2);
  std::cout << "aruco2: " << poses12[0] << std::endl;

  ips_cam::IndoorCoordSystem ics2 = SetupICS(2);

  std::cout << "ics2:" << std::endl;
  ips_cam::ObjectTracker tagFinder2 = ips_cam::ObjectTracker(ics2, tags);
  auto poses21 = tagFinder2.Track(aruco1);
  std::cout << "aruco1: " << poses21[0] << std::endl;
  auto poses22 = tagFinder2.Track(aruco2);
  std::cout << "aruco2: " << poses22[0] << std::endl;
}

TEST(test_image_processing, test_find_pattern)
{
  cv::Size pattern_size(6, 4);    // Chessboard pattern size
  // world coord choice 1
  std::vector<cv::Point3d> object_points_1;
  std::vector<cv::Point3d> object_points_2;
  for (int i = 0; i < pattern_size.height; ++i) {
    for (int j = 0; j < pattern_size.width; ++j) {
      object_points_1.push_back(cv::Point3d(j, pattern_size.height - i - 1, 0));
      object_points_2.push_back(cv::Point3d(pattern_size.width - j - 1, i, 0));
    }
  }
  // std::cout << object_points[0] << std::endl;
  // std::cout << object_points[1] << std::endl;

  cv::Mat frame, gray;

  std::cout << get_current_dir_name() << std::endl;

  frame = cv::imread("../../../im_ref2.png", cv::IMREAD_COLOR);

  cv::imshow("hi", frame);
  cv::waitKey(1000);

  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

  std::cout << frame.type() << std::endl;
  std::cout << gray.type() << std::endl;

  cv::Mat outputImage_0 = frame.clone();
  cv::Mat outputImage_1 = frame.clone();
  cv::Mat outputImage_2 = frame.clone();

  cv::imshow("mono", gray);
  cv::waitKey(1000);

  std::vector<cv::Point2f> corners;
  bool found = cv::findChessboardCorners(gray, pattern_size, corners);
  if (found) {
    cv::cornerSubPix(
      gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
      cv::TermCriteria(
        cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1)
    );
    cv::drawChessboardCorners(outputImage_0, pattern_size, corners, found);
  }

  // OK, so the chessboard finder returns coordinate systems with Z reversed, and
  // x along the the long axis of the chessboard. To fix Z, I can either
  // reverse x or y in world coords.

  std::vector<cv::Point2d> image_points;

  cv::Mat(corners).convertTo(image_points, cv::Mat(image_points).type());

  cv::imshow("points", outputImage_0);
  cv::waitKey(1000);

  // now obtain the camera extrinsics for this image
  // we will need some intrinsics...
  ips_cam::CameraIntrinsics camIntrinsics =
    ips_cam::load_camera_intrinsics("../../../camera_intrinsics.yml");

  // std::vector<cv::Vec3d> rvec, tvec;

  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);

  // std::cout << object_points << std::endl;
  // std::cout << image_points << std::endl;
  // std::cout << camIntrinsics.camera_matrix << std::endl;
  // std::cout << camIntrinsics.dist_coeffs << std::endl;

  // cv::solvePnP(object_points,
  // corners,
  // camIntrinsics.camera_matrix,
  // camIntrinsics.dist_coeffs,
  // rvec, tvec, false, cv::SOLVEPNP_IPPE);
  cv::solvePnP(
    object_points_1,
    image_points,
    camIntrinsics.camera_matrix,
    camIntrinsics.dist_coeffs,
    rvec,
    tvec,
    false,
    cv::SOLVEPNP_IPPE);

  cv::drawFrameAxes(
    outputImage_1,
    camIntrinsics.camera_matrix,
    camIntrinsics.dist_coeffs,
    rvec, tvec, 2.0);

  // std::cout << rvec << std::endl;
  // std::cout << tvec << std::endl;

  cv::imshow("pattern axes 1", outputImage_1);
  cv::waitKey(5000);

  cv::solvePnP(
    object_points_2,
    image_points,
    camIntrinsics.camera_matrix,
    camIntrinsics.dist_coeffs,
    rvec, tvec, false, cv::SOLVEPNP_IPPE);

  cv::drawFrameAxes(
    outputImage_2,
    camIntrinsics.camera_matrix,
    camIntrinsics.dist_coeffs, rvec, tvec, 2.0);

  std::cout << rvec << std::endl;
  std::cout << tvec << std::endl;

  cv::imshow("pattern axes 2", outputImage_2);
  cv::waitKey(5000);

  // cv::Mat rmat;
  // cv::Rodrigues(rvec, rmat);
  // std::cout << rmat << std::endl;

  // compose with tvec to get extrinsics
  // cv::Mat extMat(3, 4, CV_64F, cv::Scalar::all( 0.0 ));
  // cv::hconcat(rmat,tvec, extMat);
  // std::cout << extMat << std::endl;

  cv::Mat extMat = ips_cam::composeCameraExtrinsicMatrix(rvec, tvec);
  cv::Mat extMat2 = ips_cam::composeCameraExtrinsicMatrixFull(rvec, tvec);
  cv::Mat einv;
  cv::invert(extMat2, einv);
  std::cout << "new stuff!" << std::endl;
  std::cout << extMat2 << std::endl;
  std::cout << einv << std::endl;

  // std::cout << frame.channels() << std::endl;
  // std::cout << frame.size() << std::endl;

  // Paint a red square on this image at (-1,-1) in the world coordinate system.
  // This is a square in world coords, a polygon in pixel coords.
  // let's ignore distortion for this exercise.
  // loc and ext are in units of the chessboard.
  double loc[3] = {-1, -1, 0};
  cv::Mat location = cv::Mat(3, 1, CV_64F, loc);
  double ext[2] = {0.1, 0.1};
  cv::Mat extent = cv::Mat(2, 1, CV_64F, ext);
  cv::Mat rect = MakeWorldRectangleXY(location, extent);

  // promote to homo coords
  cv::Mat rectHomo = ips_cam::toHomo(rect);

  std::cout << extMat.type() << std::endl;
  std::cout << camIntrinsics.camera_matrix.type() << std::endl;

  // build the full camera matrix
  cv::Mat camMat = camIntrinsics.camera_matrix * extMat;

  // convert to camera coords
  cv::Mat camPolyHomo = camMat * rectHomo;

  // homo divide
  ips_cam::homoDivide(camPolyHomo);

  // strip the homo ones
  cv::Mat camPoly = ips_cam::fromHomo(camPolyHomo);

  // round to nearest integers
  std::cout << camPoly << std::endl;

  cv::Mat camPolyInt;
  camPoly.convertTo(camPolyInt, CV_32S);

  std::cout << camPolyInt << std::endl;

  cv::fillPoly(outputImage_2, camPolyInt.t(), cv::Scalar(0, 0, 255));

  cv::imshow("red box", outputImage_2);
  cv::waitKey(5000);

  // cv::fillPoly(frame, camPolyInt.t(), cv::Scalar(0, 0, 255));
  // cv::imwrite("../../../im_ref_tagged.png", frame);
}

TEST(test_image_processing, test_matrix_tools)
{
  // transforms, etc
  cv::Mat rvec(3, 2, cv::DataType<double>::type, cv::Scalar::all(0.5));
  // cv::Mat ones(1, 2, cv::DataType<double>::type, cv::Scalar::all( 2.0 ));
  // cv::Mat rvech(4, 2, cv::DataType<double>::type, cv::Scalar::all( 0.0 ));
  // cv::Mat rvec2(3, 2, cv::DataType<double>::type, cv::Scalar::all( 1.0 ));
  // cv::vconcat(rvec, ones, rvech);
  cv::Mat rvech = ips_cam::toHomo(rvec);
  std::cout << rvec << std::endl;
  std::cout << rvech << std::endl;

  rvech *= 2.0;

  std::cout << rvech << std::endl;

  ips_cam::homoDivide(rvech);

  // homo divide - inplace
  // for(int n = 0; n < rvech.cols; n++)
  // {
  //     cv::Mat c = rvech.col(n);
  //     int num_rows = c.rows;
  //     // do stuff to c
  //     double denom = c.at<double>(3);
  //     for(int i=0; i<num_rows; i++){
  //         c.at<double>(i) = c.at<double>(i) / denom;
  //     }
  // }
  std::cout << rvech << std::endl;

  // remove the last row
  cv::Mat rvec2 = ips_cam::fromHomo(rvech);
  // cv::Mat rvec2 = rvech.rowRange(0,3).clone();
  std::cout << rvec2 << std::endl;

  // remove a column from a matrix
  cv::Mat pmat(3, 4, cv::DataType<double>::type);
  cv::randn(pmat, cv::Scalar(0.0), cv::Scalar(1.0));
  std::cout << pmat << std::endl;

  // cv::Mat pmat_col = pmat.col(2).clone();
  cv::Mat pmat_col = ips_cam::extractColumn(pmat, 2);
  // cv::Mat pmat_col = extractColumn(pmat, 7);
  std::cout << pmat_col << std::endl;

  // cv::Mat pmat_trimmed = removeColumn(pmat,2);
  cv::Mat pmat_trimmed = ips_cam::removeColumn(pmat, 2);
  // cv::hconcat(pmat.colRange(0,2),pmat.colRange(3,4),pmat_trimmed);

  std::cout << pmat_trimmed << std::endl;

  cv::Mat mv_prod;
  mv_prod = pmat * rvech;
  std::cout << mv_prod << std::endl;
}

TEST(test_image_processing, test_drawing)
{
  cv::Mat frame = cv::imread("../../../im_ref.png", cv::IMREAD_COLOR);

  cv::imshow("raw frame", frame);
  cv::waitKey(2000);

  int32_t data[8] = {500, 500, 550, 550, 500, 550, 550, 500};

  cv::Mat rvec(2, 4, cv::DataType<int32_t>::type, data);
  std::cout << rvec.t() << std::endl;

  cv::fillPoly(frame, rvec.t(), cv::Scalar(0, 0, 255));

  cv::imshow("red box", frame);
  cv::waitKey(10000);
}

TEST(test_image_processing, test_stuff)
{
  double loc[3] = {-148, -148, 0};
  cv::Mat location = cv::Mat(3, 1, CV_64F, loc);
  double ext[2] = {40, 40};
  cv::Mat extent = cv::Mat(2, 1, CV_64F, ext);

  cv::Mat rect = MakeWorldRectangleXY(location, extent);
  std::cout << rect << std::endl;
}

TEST(test_image_processing, test_bench_aruco)
{
  cv::Mat frame;

  std::cout << get_current_dir_name() << std::endl;

  // frame = cv::imread("../../../im_aruco.png", cv::IMREAD_COLOR);
  // frame = cv::imread("../../../im_ref2_aruco1.png", cv::IMREAD_COLOR);
  frame = cv::imread("../../../im_ref2.png", cv::IMREAD_COLOR);

  ips_cam::IndoorCoordSystem ics = SetupICS();

  std::map<int, double> tags;
  tags[1] = 0.0;

  ips_cam::ObjectTracker tagFinder = ips_cam::ObjectTracker(ics, tags);


  auto start = std::chrono::steady_clock::now();

  int maxIters = 500;
  std::vector<double> data(maxIters);

  for (int i = 0; i < maxIters; i++) {
    std::vector<ips_cam::TagPose> tp = tagFinder.Track(frame);
    data[i] = std::nan("");
    if (tp.size() > 0) {data[i] = tp[0].theta;}
  }

  auto finish = std::chrono::steady_clock::now();
  double elapsed_seconds =
    std::chrono::duration_cast<std::chrono::duration<double>>(finish - start).count();

  std::cout << elapsed_seconds << std::endl;

  std::cout << data[42] << std::endl;

  std::cout << "fps: " << maxIters / elapsed_seconds << std::endl;
}

TEST(test_image_processing, test_find_aruco)
{
  cv::Mat frame;

  std::cout << get_current_dir_name() << std::endl;

  // frame = cv::imread("../../../im_aruco.png", cv::IMREAD_COLOR);
  frame = cv::imread("../../../im_ref2_aruco1.png", cv::IMREAD_COLOR);
  // frame = cv::imread("../../../im_ref2.png", cv::IMREAD_COLOR);

  cv::imshow("hi", frame);
  cv::waitKey(2000);

  ips_cam::IndoorCoordSystem ics = SetupICS();

  std::map<int, double> tags;
  tags[1] = 0.0;

  ips_cam::ObjectTracker tagFinder = ips_cam::ObjectTracker(ics, tags);

  ips_cam::MarkerDetections foundTags = tagFinder.FindMarkers(frame);

  std::cout << foundTags.markerIds.size() << std::endl;

  cv::Mat outputImage = frame.clone();
  cv::aruco::drawDetectedMarkers(outputImage, foundTags.markerCorners, foundTags.markerIds);

  cv::imshow("markers", outputImage);
  cv::waitKey(2000);

  // draw axis for each marker
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(
    foundTags.markerCorners,
    0.05, ics.cameraIntrinsics.camera_matrix,
    ics.cameraIntrinsics.dist_coeffs, rvecs, tvecs);
  // draw axis for each marker
  for (uint i = 0; i < foundTags.markerIds.size(); i++) {
    cv::aruco::drawAxis(
      outputImage,
      ics.cameraIntrinsics.camera_matrix,
      ics.cameraIntrinsics.dist_coeffs, rvecs[i], tvecs[i], 0.1);
  }


  cv::imshow("marker poses", outputImage);
  cv::waitKey(1000);
  std::vector<cv::Mat> worldCorners;
  tagFinder.ImageToWorld(foundTags, worldCorners);
  std::vector<ips_cam::TagPose> poses = tagFinder.FindMarkerPoses(foundTags, worldCorners);

  if (foundTags.markerIds.size() > 0) {
    std::cout << worldCorners[0] << std::endl;
    std::cout << poses[0] << std::endl;
    // std::vector<TagPose> poses2 = tagFinder.Track(frame);
    // std::cout << poses2[0] << std::endl;
  }
}

TEST(test_image_processing, test_image_to_world)
{
  // cast some world points to an image and use
  // the ImageToWorld tech to get them back
  // these are in mm. A square is 198 mm.
  double world_points_data[] =
  {-100, 37, 1000, 750, 422, 240, 512, 677, 82, -98, 0, 10, 20, -20, 50};
  cv::Mat world_points = cv::Mat(3, 5, CV_64F, world_points_data);

  cv::Mat world_points_homo = ips_cam::toHomo(world_points);

  ips_cam::IndoorCoordSystem ics = SetupICS();

  cv::Mat camMatrix = ics.cameraIntrinsics.camera_matrix * ics.extMat;

  cv::Mat cam_points_homo = camMatrix * world_points_homo;
  // std::cout << cam_points_homo << std::endl;

  ips_cam::homoDivide(cam_points_homo);

  // std::cout << cam_points_homo << std::endl;

  cv::Mat cvec = ips_cam::extractRow(world_points, 2);

  // now invert
  ips_cam::ImagePointsToWorldPoints iptwp = ips_cam::ImagePointsToWorldPoints(ics, cvec, 5);

  cv::Mat world_points_2 = iptwp.ToWorld(cam_points_homo);

  std::cout << world_points_homo << std::endl;
  std::cout << world_points_2 << std::endl;

  // the test is to compare the x and y world points
  // - note we treat z as known throughout.
  cv::Mat world_points_homo_diff = world_points_homo.rowRange(0, 2).clone();
  cv::Mat world_points_2_diff = world_points_2.rowRange(0, 2).clone();

  double error = cv::norm(world_points_homo_diff, world_points_2_diff, cv::NORM_INF);

  std::cout << error << std::endl;

  ASSERT_TRUE(error <= 0.1);
}

TEST(test_image_processing, test_rigid_xform)
{
  // assume a set of ICS world space coordinates returned by detectMarkers/etc
  // which correspond to:
  // (-markerLength/2, markerLength/2, 0),
  // (markerLength/2, markerLength/2, 0),
  // (markerLength/2, -markerLength/2, 0),
  // (-markerLength/2, -markerLength/2, 0)

  // define the pattern as a matrix of column vectors; x,x,x,x,y,y,y,y
  double tag_points_data[] = {-0.5, 0.5, 0.5, -0.5, 0.5, 0.5, -0.5, -0.5};
  cv::Mat tag_points = cv::Mat(2, 4, CV_64F, tag_points_data);

  double centerx = 10.0;
  double centery = 10.0;
  double theta = -113;
  double scale = 1.4;
  double theta_rad = theta * 3.14159265358979323846 / 180.0;

  cv::Mat rotmat = cv::Mat(2, 3, cv::DataType<double>::type);
  rotmat.at<double>(0, 0) = scale * std::cos(theta_rad);
  rotmat.at<double>(0, 1) = scale * std::sin(theta_rad);
  rotmat.at<double>(1, 1) = scale * std::cos(theta_rad);
  rotmat.at<double>(1, 0) = -scale * std::sin(theta_rad);
  rotmat.at<double>(0, 2) = centerx;
  rotmat.at<double>(1, 2) = centery;

  cv::Mat tag_points_homo = ips_cam::toHomo(tag_points);

  cv::Mat my_points = rotmat * tag_points_homo;
  cv::Mat my_points_noise = my_points.clone();
  float snr = 10;
  // add some noise
  cv::randn(my_points_noise, 0.0, scale / snr);
  my_points += my_points_noise;
  std::cout << tag_points << std::endl;
  std::cout << my_points << std::endl;

  // fit a transform, get pose
  ips_cam::TagPoseEstimator tpe = ips_cam::TagPoseEstimator();
  ips_cam::TagPose my_pose = tpe.estimate(my_points);
  std::cout << my_pose << std::endl;
}

TEST(test_image_processing, test_yaml_read)
{
  // load calibration data
  ips_cam::CameraIntrinsics camIntrinsics =
    ips_cam::load_camera_intrinsics("../../../camera_intrinsics.yml");
  std::cout << camIntrinsics.camera_matrix << std::endl;
  std::cout << camIntrinsics.dist_coeffs << std::endl;
  ASSERT_EQ(camIntrinsics.camera_matrix.dims, 2);

  bool success = false;
  try {
    ips_cam::load_camera_intrinsics("no_such_file.yml");
  } catch (std::invalid_argument & e) {
    success = true;
  }
  ASSERT_TRUE(success);
}

TEST(test_image_processing, test_yaml_read2)
{
  ips_cam::IcsParams ip =
    ips_cam::load_ics_params("~/IndoorPositioningSystem/ips_config/ics_params.yml");
  ips_cam::TrackingParams tp =
    ips_cam::load_tracking_params("~/IndoorPositioningSystem/ips_config/tracking.yml");
}

TEST(test_image_processing, test_yaml_read_exp)
{
  // load calibration data
  ips_cam::load_thingy("../../../thingy.yml");
}

TEST(ExceptionTest, ThrowsRuntimeError)
{
  ASSERT_THROW(functionThatThrows(), std::runtime_error);
}
