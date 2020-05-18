
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "camFusion.hpp"
#include "dataStructures.h"
#include "render/render.h"
//#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/crop_box.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/common/transforms.h>

//class Ptr;

using namespace std;

//Compute the median
double median(vector<double> medi) {
    int size = medi.size();
    double tmedian;
    if (size % 2 == 0) { // even
        tmedian = (medi[medi.size() / 2 - 1] + medi[medi.size() / 2]) / 2;
    }
    else //odd
        tmedian = medi[medi.size() / 2];
    return (tmedian);
}

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

//    // display image
//    string windowName = "3D Objects";
//    cv::namedWindow(windowName, 1);
//    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


void registercloud_from_vector(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<LidarPoint> &lidarPoints){

    cloud->width = lidarPoints.size();
    cloud->height = 1;
    cloud->points.resize (cloud->width*cloud->height);

    for (int i=0; i<lidarPoints.size(); ++i){
        cloud->points[i].x =  lidarPoints[i].x;
        cloud->points[i].y =  lidarPoints[i].y;
        cloud->points[i].z =  lidarPoints[i].z;
    }
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    //Find all matches of current key matched points
    for (auto kptMatch : kptMatches){
//        cout << kptMatch.trainIdx << endl;
//        cout << kptsCurr[kptMatch.trainIdx].pt << endl;
        if (boundingBox.roi.contains(kptsCurr[kptMatch.trainIdx].pt))
        boundingBox.kptMatches.push_back(kptMatch);
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{

    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
//                cout << "distRatio: " << distRatio << endl;
                distRatios.push_back(distRatio);

            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = std::numeric_limits<double>::quiet_NaN();
        return;
    }

    // compute camera-based TTC from distance ratios
//    double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();
//    cout << "mean" << meanDistRatio << endl;
//    double dT = 1 / frameRate;
//    TTC = -dT / (1 - meanDistRatio);

    // STUDENT TASK (replacement for meanDistRatio)
    std::sort(distRatios.begin(), distRatios.end());
//    double medianDistRatio = distRatios[distRatios.size() / 2];
    double medianDistRatio = median(distRatios);
//    cout << "median" <<  medianDistRatio << endl;
    TTC = (-1.0 / frameRate) / (1 - medianDistRatio);
    if (TTC < 0){
        TTC = 0; //delete -inf
    }

    cout << "Camera Time to Collision: " << TTC << endl;
    cout << " " << endl;


}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    //========================================
    //Visualization
    //========================================
//    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D LiDAR Viewer"));
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr Prevcloud(new pcl::PointCloud<pcl::PointXYZ>);
//    registercloud_from_vector(Prevcloud, lidarPointsPrev);
//    renderPointCloud(viewer, Prevcloud, "PrevCloud", Color(0,1,0));
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr Currcloud(new pcl::PointCloud<pcl::PointXYZ>);
//    registercloud_from_vector(Currcloud, lidarPointsCurr);
//    renderPointCloud(viewer, Currcloud, "CurrentCloud", Color(1,0,0));
//    viewer->spin();
    //========================================
    //========================================

    // Checking the LiDAR y-axis information to filter the point cloud

    double dt = 1/frameRate; //Sampling time
    double lane_width = 3.0; //Assumption

    vector<double> PrevPoints_x, CurrPoints_x;

    for (int i=0; i<lidarPointsPrev.size(); ++i){
        if (abs(lidarPointsPrev[i].y) <= lane_width/2.0){
            PrevPoints_x.push_back(lidarPointsPrev[i].x);
        }
    }

    for (int i=0; i<lidarPointsCurr.size(); ++i){
        if (abs(lidarPointsCurr[i].y) <= lane_width/2.0){
            CurrPoints_x.push_back(lidarPointsCurr[i].x);
        }
    }

    // Computer the average
    double avg_prev = 0;
    double avg_curr = 0;

    if (PrevPoints_x.size() > 0){
        for (auto PrevX : PrevPoints_x)
            avg_prev += PrevX;
    }
    avg_prev = avg_prev / PrevPoints_x.size();

    if (CurrPoints_x.size() > 0){
        for (auto CurrX : CurrPoints_x)
            avg_curr += CurrX;
    }
    avg_curr = avg_curr / CurrPoints_x.size();

//    cout << "Avg Current: " << avg_curr << endl;
//    cout << "Avg Previous: " << avg_prev << endl;


    TTC = avg_curr * dt / (avg_prev - avg_curr);
    cout << "LiDAR Time to Collision: " << TTC << endl;
    cout << " " << endl;

}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
//    cout << "TEST" << endl;

    int prev_num = prevFrame.boundingBoxes.size();
    int curr_num = currFrame.boundingBoxes.size();
//    std::multimap<int, int> mmap {};
    int matched_point[prev_num][curr_num] = { };

    for (auto match_pts : matches){
        cv::KeyPoint prev_keypoints = prevFrame.keypoints[match_pts.queryIdx];
        cv::KeyPoint curr_keypoints = currFrame.keypoints[match_pts.trainIdx];

        int prev_BBOXID = -1;
        int curr_BBOXID = -1;

        bool prev_ismatched = false;
        bool curr_ismatched = false;

        std::vector<int> prev_bboxID_data, curr_bboxID_data;

        // Check the previous bounding box
        for (auto bbox : prevFrame.boundingBoxes){
            if (bbox.roi.contains(prev_keypoints.pt)){
                prev_BBOXID = bbox.boxID;
                prev_bboxID_data.push_back(prev_BBOXID);
                prev_ismatched = true;
            }
        }

        // Check the current bounding box
        for (auto bbox : currFrame.boundingBoxes){
            if (bbox.roi.contains(curr_keypoints.pt)){
                curr_BBOXID = bbox.boxID;
                curr_bboxID_data.push_back(curr_BBOXID);
                curr_ismatched = true;
            }
        }

        if (prev_ismatched == true && curr_ismatched == true){
            for (auto prev_bboxID_i : prev_bboxID_data){
                for (auto curr_bboxID_i :curr_bboxID_data){
                    matched_point[prev_bboxID_i][curr_bboxID_i] += 1;
                }
            }
        }
    }

    for (int prev_i = 0; prev_i < prev_num; prev_i++){
        int max_count = 0;
        int max_id = 0;
        for (int curr_i = 0; curr_i < curr_num; curr_i++){
            if (matched_point[prev_i][curr_i] > max_count) {
                max_count = matched_point[prev_i][curr_i];
                max_id = curr_i;
            }
        }
        bbBestMatches[prev_i] = max_id;
    }
//    for (int i = 0; i < prev_num; i++)
//        cout << "Box " << i << " matches " << bbBestMatches[i]<< " box" << endl;
 }
