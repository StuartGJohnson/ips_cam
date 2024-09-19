#include <opencv2/opencv.hpp>
#include <gtest/gtest.h>
#include <opencv2/aruco.hpp>

#include <string>
#include <limits.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <ips_cam/image_processor.hpp>
#include <cstdlib> // For getenv

namespace ips_cam
{

// Function to expand tilde to home directory
std::string expand_tilde(const std::string &path) {
    if (!path.empty() && path[0] == '~') {
        const char *home = std::getenv("HOME");  // Get the HOME environment variable
        if (home) {
            return std::string(home) + path.substr(1);  // Replace ~ with the value of $HOME
        }
    }
    return path;  // Return the path unchanged if no tilde
}


void check_file_existence(std::string filename)
{

    // check file existence
    std::ifstream file;
    file.open(filename);
    if (file)
    {
        file.close();
    }
    else
    {
        throw std::invalid_argument("non-existent file!");
    }

}

std::string expand_and_check(std::string filename1)
{
    std::string filename = expand_tilde(filename1);
    check_file_existence(filename);
    return filename;
}

CameraIntrinsics load_camera_intrinsics(std::string filename1)
{
    std::string filename = expand_and_check(filename1);

    CameraIntrinsics camIntrinsics;
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["camera_matrix"] >> camIntrinsics.camera_matrix;
    fs["dist_coeffs"] >> camIntrinsics.dist_coeffs;
    fs["image_height"] >> camIntrinsics.image_height;
    fs["image_width"] >> camIntrinsics.image_width;
    // camIntrinsics.dist_coeffs *= 0.0;

    return camIntrinsics;
}

IcsParams load_ics_params(std::string filename1)
{
    std::string filename = expand_and_check(filename1);

    int origin_tag;
    double origin_tag_z;
    IcsParams p;
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["intrinsics_file"] >> p.intrinsics_file;
    fs["checkerboard_image_file"] >> p.checkerboard_image_file;
    fs["origin_image_file"] >> p.origin_image_file;
    fs["cbExtentX"] >> p.cbExtentX;
    fs["cbExtentY"] >> p.cbExtentY;
    fs["cbBlockSize"] >> p.cbBlockSize;
    fs["origin_tag"] >> origin_tag;
    fs["origin_tag_z"] >> origin_tag_z;
    p.originTag.insert(std::make_pair(origin_tag, origin_tag_z));

    return p;
}

TrackingParams load_tracking_params(std::string filename1)
{
    std::string filename = expand_and_check(filename1);

    TrackingParams p;
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["tag"] >> p.tag;
    fs["tag_z"] >> p.tag_z;
    if (p.tag.size() != p.tag_z.size())
    {
        throw std::invalid_argument("In load_tracking_params: map construction failed.");
    }
    for (unsigned int i=0; i<p.tag.size(); i++)
    {
        p.tag_lookup.insert(std::make_pair(p.tag[i], p.tag_z[i]));
    }

    return p;
}

void load_thingy(std::string filename)
{
    // check file existence
    std::ifstream file;
    file.open(filename);
    if (file)
    {
        file.close();
    }
    else
    {
        throw std::invalid_argument("non-existent file!");
    }

    std::string thing3;
    std::vector<int> thing1;
    std::vector<double> thing2;
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["thing1"] >> thing1;
    fs["thing2"] >> thing2;
    fs["thing3"] >> thing3;
    std::map<int, double> thing_map;
    if (thing1.size() != thing2.size())
    {
        throw std::invalid_argument("In thingy: map construction failed.");
    }
    for (unsigned int i=0; i<thing1.size(); i++)
    {
        thing_map.insert(std::make_pair(thing1[i], thing2[i]));
    }

    std::cout << thing3 << std::endl;

    // display map
    std::for_each(thing_map.begin(), thing_map.end(), [](const std::pair<int,double>& p)
    {
        std::cout << p.first << "," << p.second << "\n";
    });

    int key = 1;

    if (thing_map.count(key) == 1)
    {
        std::cout << thing_map[key] << std::endl;
    }

}

cv::Mat toHomoMat(std::vector<cv::Point2f> points)
{
    int numCols = points.size();
    cv::Mat out = cv::Mat(3, numCols, CV_64F);
    int col = 0;
    for (cv::Point2f val : points)
    {
        out.at<double>(0, col) = val.x;
        out.at<double>(1, col) = val.y;
        out.at<double>(2, col) = 1.0;
        col++;
    }
    return out;
}

cv::Mat toHomo(cv::Mat columnVectors)
{
    // augment a matrix of column vectors with a final row of ones
    int mCols = columnVectors.cols;
    int mRows = columnVectors.rows;
    int mType = columnVectors.type();
    cv::Mat ones(1, mCols, mType, cv::Scalar::all(1));
    cv::Mat out(mRows + 1, mCols, mType);
    cv::vconcat(columnVectors, ones, out);
    return out;
}

cv::Mat fromHomo(cv::Mat homoColumnVectors)
{
    // De-augment a matrix of column vectors by removing the final row of ones - or
    // whatever is in the final row. Note this method does not do the homogeneous divide.
    // Note this returns a new matrix with a copy of the relevant data.
    int mRows = homoColumnVectors.rows;
    return homoColumnVectors.rowRange(0, mRows - 1).clone();
}

void homoDivide(cv::Mat homoColumnVectors)
{
    // Divide each column of homogeneous column vectors by the last row value.
    // In-place operation!
    int num_rows = homoColumnVectors.rows;
    for (int n = 0; n < homoColumnVectors.cols; n++)
    {
        cv::Mat c = homoColumnVectors.col(n);
        //std::cout << c << std::endl;
        double denom = homoColumnVectors.at<double>(num_rows - 1, n);
        for (int i = 0; i < num_rows; i++)
        {
            homoColumnVectors.at<double>(i,n) = 
                homoColumnVectors.at<double>(i,n) / denom;
        }
        //std::cout << c << std::endl;
    }
}

cv::Mat extractColumn(cv::Mat inputMatrix, int col)
{
    // Extract column col from inputMatrix. Copies relevant
    // content into a new matrix (or column vector, if you will).
    return inputMatrix.col(col).clone();
}

cv::Mat extractRow(cv::Mat inputMatrix, int row)
{
    // Extract column col from inputMatrix. Copies relevant
    // content into a new matrix (or column vector, if you will).
    return inputMatrix.row(row).clone();
}

cv::Mat removeColumn(cv::Mat inputMatrix, int col)
{
    // Remove column col from inputMatrix. Copies relevant
    // content into a new matrix.
    cv::Mat out;
    int numCols = inputMatrix.cols;
    if (col == 0)
    {
        out = inputMatrix.colRange(1, numCols).clone();
    }
    else if (col == numCols - 1)
    {
        out = inputMatrix.colRange(0, numCols - 1).clone();
    }
    else
    {
        // Concatenate columns to the left and to the right.
        // Question: do I need a clone() here?
        cv::hconcat(inputMatrix.colRange(0, col), inputMatrix.colRange(col + 1, numCols), out);
    }
    return out;
}

cv::Mat composeCameraExtrinsicMatrix(cv::Mat rvec, cv::Mat tvec)
{
    // Construct the camera extrinsics matrix from the output of such things as cv::solvePnP()

    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    int mType = rvec.type();

    // compose with tvec to get extrinsics
    cv::Mat extMat(3, 4, mType, cv::Scalar::all(0.0));
    cv::hconcat(rmat, tvec, extMat);
    return extMat;
}


cv::Mat composeExtrinsicFlip(int cbExtentX, int cbExtentY, double cbBlockMeters)
{
    // the pattern finder has chosen an offending orientation; correct!
    double xtrans = (cbExtentX - 1) * cbBlockMeters;
    double ytrans = (cbExtentY - 1) * cbBlockMeters;
    double data[] = {-1, 0, 0, xtrans, 0, -1, 0, ytrans, 0, 0, 1, 0, 0,0,0,1};
    cv::Mat extFlip(4, 4, CV_64F, data);
    // note the data is local, so clone on exit
    return extFlip.clone();
}

cv::Mat composeCameraExtrinsicMatrixFull(cv::Mat rvec, cv::Mat tvec)
{
    // Construct the camera extrinsics matrix from the output of such things as cv::solvePnP().
    // In this case, add a final row for operations in camera coordinates.

    cv::Mat firstExtMat = composeCameraExtrinsicMatrix(rvec, tvec);

    double data[] = {0, 0, 0, 1};
    int mType = firstExtMat.type();
    cv::Mat lastRow(1, 4, mType, data);
    cv::Mat extMat(4, 4, mType);
    cv::vconcat(firstExtMat, lastRow, extMat);
    // note the data is local, so clone on exit (although it seems to work without)
    return extMat.clone();
}


ObjectTracker::ObjectTracker(IndoorCoordSystem ics_init, std::map<int, double> tags)
{

    ics = ics_init;

    trackedTags = tags;

    for (auto const& tag : tags)
    {
        //thing_map.insert(std::make_pair(thing1[i], thing2[i]));
        imgToWorld.emplace(tag.first, ImagePointsToWorldPoints(ics, tag.second, 4));
    }

    // prep for calling the object tracker computation over and over
    detectorParams = cv::aruco::DetectorParameters::create();
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);

    tagPoseEstimator = TagPoseEstimator();


    // in this case we use a single z
    // todo: construct a vector of these with an
    // entry for each tag id # in the FOV.
    //imgToWorld = ImagePointsToWorldPoints(ics_init, z, 4);

}

MarkerDetections ObjectTracker::FindMarkers(cv::Mat inputImage)
{

    MarkerDetections markerDetections;

    // note that I leave it up to the caller to convert the image
    // to monochrome. detectMarkers() will do this if it needs to.
    cv::aruco::detectMarkers(
        inputImage, dictionary,
        markerDetections.markerCorners,
        markerDetections.markerIds,
        detectorParams,
        markerDetections.rejectedCandidates);

    return markerDetections;

}

std::vector<TagPose> ObjectTracker::Track(cv::Mat inputImage)
{

    MarkerDetections detections = FindMarkers(inputImage);

    std::vector<cv::Mat> worldCorners;

    ImageToWorld(detections, worldCorners);

    std::vector<TagPose> out = FindMarkerPoses(detections, worldCorners);

    return out;

}


std::vector<TagPose> ObjectTracker::FindMarkerPoses(MarkerDetections detections, std::vector<cv::Mat>& worldCorners)
{
    // from the detected markers (image space), compute the 
    // poses of the markers assuming they are in a known Z-plane
    // of the ICS.

    std::vector<TagPose> tagPoses;

    // step 1: use the ics to go from image space to world coordinates
    //ImageToWorld(detections, worldCorners);

    // step 2: do a least-squares fit of the world coordinates to get
    // pose of each detected object.

    for (unsigned i=0; i < worldCorners.size(); i++)
    {
        TagPose currentTag = tagPoseEstimator.estimate(worldCorners[i]);
        currentTag.tag = detections.markerIds[i];
        // this tag has to be in the tag dicationary, since it generated
        // a set of worldCorners!
        currentTag.z = trackedTags[currentTag.tag];
        tagPoses.push_back(currentTag);
    }

    return tagPoses;

}

void ObjectTracker::ImageToWorld(MarkerDetections detections, std::vector<cv::Mat>& worldCorners)
{

    for (unsigned int i=0; i < detections.markerCorners.size(); i++)
    {
        auto detectedCorners = detections.markerCorners[i];
        int tag = detections.markerIds[i];
        // extract the matrix of column vectors for the corners
        // of this marker in image space
        cv::Mat imageCornersHomo = toHomoMat(detectedCorners);
        auto it = imgToWorld.find(tag);
        if (it != imgToWorld.end())
        {
            ImagePointsToWorldPoints& instance = it->second;
            cv::Mat thing = instance.ToWorld(imageCornersHomo);
            worldCorners.push_back(thing);
        }
    }

}

ImagePointsToWorldPoints::ImagePointsToWorldPoints(IndoorCoordSystem ics, double z, int numPoints)
{
    cv::Mat zvec = cv::Mat(1, numPoints, CV_64F);
    zvec = z;
    Init(ics, zvec, numPoints);

}

ImagePointsToWorldPoints::ImagePointsToWorldPoints(IndoorCoordSystem ics, cv::Mat zvec, int numPoints)
{
    Init(ics, zvec, numPoints);
}

void ImagePointsToWorldPoints::Init(IndoorCoordSystem ics, cv::Mat zvec, int numPoints)
{

    // form the full camera projection matrix
    cv::Mat p = ics.cameraIntrinsics.camera_matrix * ics.extMat;
    cv::Mat pxy = removeColumn(p, 2);
    cv::Mat pz = extractColumn(p, 2);
    // invert pxy
    cv::invert(pxy, pxy_inverse);
    // row vector
    cv::Mat onesRow = cv::Mat(1, numPoints, CV_64F);
    onesRow = 1.0;
    // col vector
    onesCol = cv::Mat(3, 1, CV_64F);
    onesCol = 1.0;
    adj_xy_z = pxy_inverse * pz * zvec;
    scale_numerator = onesRow + extractRow(adj_xy_z, 2);
}

cv::Mat ImagePointsToWorldPoints::ToWorld(cv::Mat imagePointsHomo)
{

    cv::Mat raw_xy = pxy_inverse * imagePointsHomo;

    // compute the scale factor from the last row
    // of the above matrices
    cv::Mat scale = scale_numerator / extractRow(raw_xy, 2);

    cv::Mat scale_all = onesCol * scale;

    cv::Mat worldPoints = scale_all.mul(raw_xy) - adj_xy_z;

    return worldPoints;
}

TagPoseEstimator::TagPoseEstimator()
{

    // define the pattern as a matrix of column vectors; x,x,x,x,y,y,y,y
    double tag_points_data[] = {-0.5, 0.5, 0.5, -0.5, 0.5, 0.5, -0.5, -0.5};
    cv::Mat tagCoords = cv::Mat(2, 4, CV_64F, tag_points_data);

    // prep for new data

    // A simple least-squares fit of worldCoords estimates from image processing.
    // The intent is to give a fast, robust estimate of the tag pose given 2D pose
    // constraints - i.e., that the tag is in the plane of the ICS. Since the tag has
    // been found, we assume we do not need fancy RANSACing or point indexing here.

    // build design matrix
    // this is a loop over points
    int ncols = tagCoords.cols;
    cv::Mat designMat = cv::Mat(ncols * 2, 4, cv::DataType<double>::type);
    rhs = cv::Mat(ncols * 2, 1, cv::DataType<double>::type);
    int currentRow = 0;
    for (int i = 0; i < ncols; i++)
    {
        designMat.at<double>(currentRow, 0) = tagCoords.at<double>(0, i);
        designMat.at<double>(currentRow, 1) = tagCoords.at<double>(1, i);
        designMat.at<double>(currentRow, 2) = 1.0;
        designMat.at<double>(currentRow, 3) = 0.0;
        currentRow += 1;
        designMat.at<double>(currentRow, 0) = tagCoords.at<double>(1, i);
        designMat.at<double>(currentRow, 1) = -tagCoords.at<double>(0, i);
        designMat.at<double>(currentRow, 2) = 0.0;
        designMat.at<double>(currentRow, 3) = 1.0;
        currentRow += 1;
    }

    cv::Mat designMatTranspose;
    cv::transpose(designMat, designMatTranspose);

    // prep for doing least squares via the normal equations
    cv::Mat inverseTmp;
    cv::invert(designMatTranspose * designMat, inverseTmp);
    //cv::Mat m = inverseTmp * designMatTranspose * rhs;
    estimatorMatrix = inverseTmp * designMatTranspose;

}

TagPose TagPoseEstimator::estimate(cv::Mat worldCoords)
{

    int ncols = worldCoords.cols;
    int currentRow = 0;
    for (int i = 0; i < ncols; i++)
    {
        rhs.at<double>(currentRow) = worldCoords.at<double>(0, i);
        currentRow += 1;
        rhs.at<double>(currentRow) = worldCoords.at<double>(1, i);
        currentRow += 1;
    }

    cv::Mat m = estimatorMatrix * rhs;

    // unpack the model matrix into pose
    TagPose tagPose;
    tagPose.x = m.at<double>(2, 0);
    tagPose.y = m.at<double>(3, 0);
    // note the negation accounts for the fact that we want the rotation
    // angle from world space to tag space.
    tagPose.theta = -std::atan2(m.at<double>(1, 0), m.at<double>(0, 0));
    return tagPose;

}

IndoorCoordSystem EstablishIndoorCoordinateSystem(
    IcsParams icsParams)
{
    std::string checkerboard_image_file = expand_and_check(icsParams.checkerboard_image_file);
    std::string origin_image_file = expand_and_check(icsParams.origin_image_file);
    std::string intrinsics_file = expand_and_check(icsParams.intrinsics_file);
    cv::Mat cbPatternImage = cv::imread(checkerboard_image_file, cv::IMREAD_COLOR);
    cv::Mat cbOriginImage = cv::imread(origin_image_file, cv::IMREAD_COLOR);
    CameraIntrinsics camIntrinsics = load_camera_intrinsics(intrinsics_file);
   

    IndoorCoordSystem ics = EstablishIndoorCoordinateSystem(
        icsParams.cbExtentX, icsParams.cbExtentY, icsParams.cbBlockSize,
        camIntrinsics,
        cbPatternImage, cbOriginImage, icsParams.originTag);

    return ics;

}

IndoorCoordSystem EstablishIndoorCoordinateSystem(
    int cbExtentX, int cbExtentY, double cbBlockMeters,
    CameraIntrinsics camIntrinsics,
    cv::Mat cbPatternImage, cv::Mat cbOriginImage, std::map<int, double> originTag)
{
    cv::Size pattern_size(cbExtentX, cbExtentY); // Chessboard pattern size
    std::vector<cv::Point3d> object_points;
    for (int i = 0; i < pattern_size.height; ++i)
    {
        for (int j = 0; j < pattern_size.width; ++j)
        {
            object_points.push_back(cv::Point3d(cbBlockMeters * j, cbBlockMeters * (pattern_size.height - i - 1), 0));
        }
    }

    cv::Mat gray;
    cv::cvtColor(cbPatternImage, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray, pattern_size, corners);
    if (found)
    {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
    }

    std::vector<cv::Point2d> image_points;

    cv::Mat(corners).convertTo(image_points, cv::Mat(image_points).type());

    IndoorCoordSystem ics = IndoorCoordSystem();

    cv::solvePnP(object_points, image_points, camIntrinsics.camera_matrix, camIntrinsics.dist_coeffs, ics.rvec, ics.tvec, false, cv::SOLVEPNP_IPPE);

    ics.cameraIntrinsics = camIntrinsics;
    ics.SetExtrinsics();

    ObjectTracker tracker = ObjectTracker(ics, originTag);

    std::vector<TagPose> tagPoses = tracker.Track(cbOriginImage);

    // assume one tag, dump the position

    //std::cout << tagPoses[0] << std::endl;

    // either this tag is near 0,0 - or it is near the other possible origin at
    // cbBlockMeters * (cbExtentX-1, cbExtentY-1), or - I'm confused!

    cv::Point2d refCorrect = cv::Point2d(0.0, 0.0) * cbBlockMeters;
    cv::Point2d refIncorrect = cv::Point2d(cbExtentX-1, cbExtentY-1) * cbBlockMeters;
    // todo - redefine tag to use Point2d
    cv::Point2d tagLoc = cv::Point2d(tagPoses[0].x, tagPoses[0].y);

    double errorCorrect = cv::norm(tagLoc - refCorrect);
    double errorIncorrect = cv::norm(tagLoc - refIncorrect);
    double tol = cbBlockMeters * 0.5;

    if (errorCorrect <= tol)
    {
        // excellent!
        return ics;
    }
    else if (errorIncorrect <= tol)
    {
        // rotate and translate to the other coordinate system and
        // update the extrinsic matrices
        ics.SetFlippedExtrinsics(cbExtentX, cbExtentY, cbBlockMeters);
        return ics;
    }
    else
    {
        // oops!
        throw std::domain_error("ICS Origin Marker not at an expected location!");
    }

}

}




