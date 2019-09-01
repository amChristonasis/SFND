
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


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

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        if (kptsPrev[it->trainIdx].pt.x > boundingBox.roi.x && kptsPrev[it->trainIdx].pt.x < boundingBox.roi.x + boundingBox.roi.height)
        {
            if (kptsPrev[it->trainIdx].pt.y > boundingBox.roi.y && kptsPrev[it->trainIdx].pt.y < boundingBox.roi.y + boundingBox.roi.width)
            {
                if (kptsCurr[it->trainIdx].pt.x > boundingBox.roi.x && kptsCurr[it->trainIdx].pt.x < boundingBox.roi.x + boundingBox.roi.height)
                {
                    if (kptsCurr[it->trainIdx].pt.y > boundingBox.roi.y && kptsCurr[it->trainIdx].pt.y < boundingBox.roi.y + boundingBox.roi.width)
                    {
                        boundingBox.keypoints.push_back(kptsCurr[it->trainIdx]);
                        boundingBox.kptMatches.push_back(*it);
                    }
                }
            }
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC)
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
                if (distRatio > 0.2 && distRatio < 5)
                    distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    float min_z_prev = lidarPointsPrev[0].z;
    float max_z_prev = lidarPointsPrev[0].z;
    float min_z_curr = lidarPointsCurr[0].z;
    float max_z_curr = lidarPointsCurr[0].z;
    float min_y_prev = lidarPointsPrev[0].y;
    float max_y_prev = lidarPointsPrev[0].y;
    float min_y_curr = lidarPointsCurr[0].y;
    float max_y_curr = lidarPointsCurr[0].y;

    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it) {
        if (it->z > max_z_prev) max_z_prev = it->z;
        if (it->z < min_z_prev) min_z_prev = it->z;
        if (it->y > max_y_prev) max_y_prev = it->y;
        if (it->y < min_y_prev) min_y_prev = it->y;
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it) {
        if (it->z > max_z_curr) max_z_curr = it->z;
        if (it->z < min_z_curr) min_z_curr = it->z;
        if (it->y > max_y_curr) max_y_curr = it->y;
        if (it->y < min_y_curr) min_y_curr = it->y;
    }

    double window_width = 2.0;
    double window_height = 1.0;

    std::vector<int> lidarPointsPrevWindow, lidarPointsCurrWindow;
  
  	for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if (it->z < window_height/2 + (min_z_prev + max_z_prev)/2 && it->z > - window_height/2 + (min_z_prev + max_z_prev)/2
            && it->y < window_width/2 + (min_y_prev + max_y_prev)/2 && it->y > -window_width/2 + (min_y_prev + max_y_prev)/2)
        	lidarPointsPrevWindow.push_back(it->x);
    }
    double average_prev = accumulate(lidarPointsPrevWindow.begin(), lidarPointsPrevWindow.end(), 0.0)/lidarPointsPrevWindow.size(); 

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if (it->z < window_height/2 + (min_z_curr + max_z_curr)/2 && it->z > - window_height/2 + (min_z_curr + max_z_curr)/2
            && it->y < window_width/2 + (min_y_curr + max_y_curr)/2 && it->y > -window_width/2 + (min_y_curr + max_y_curr)/2)
        	lidarPointsCurrWindow.push_back(it->x);
    }
    double average_curr = accumulate(lidarPointsCurrWindow.begin(), lidarPointsCurrWindow.end(), 0.0)/lidarPointsCurrWindow.size();

    TTC = average_curr / ((average_prev - average_curr)*frameRate);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // multimap<int, int> multimap_bb_matches; // trainIdx; bb nb
    // for (auto it = matches.begin(); it != matches.end(); ++it){
    //     multimap_bb_matches.insert(pair<int, int>(prevFrame.boundingBoxes[it->trainIdx].boxId, currFrame.boundingBoxes[it->trainIdx].boxId));
    // }
    // for (auto it = multimap_bb_matches.begin(); it != multimap_bb_matches.end(); ++it){

    // }
    std::vector< std::vector<int> > bins;
    int max = 0;
    int indMax = 0;
    std::vector<bool> processed;
    int threshold = 100;
    std::cout << "Enter" << std::endl;
    for (auto itCurrent = currFrame.boundingBoxes.begin(); itCurrent != currFrame.boundingBoxes.end(); ++itCurrent) {
        for (auto itPrevious = prevFrame.boundingBoxes.begin(); itPrevious != prevFrame.boundingBoxes.end(); ++itPrevious) {
            for (auto itMatch = matches.begin(); itMatch != matches.end(); ++itMatch) {
                if (itCurrent->roi.contains(currFrame.keypoints[itMatch->trainIdx].pt)) {
                    if (itPrevious->roi.contains(prevFrame.keypoints[itMatch->queryIdx].pt)) {
                        bins[itCurrent->boxID][itPrevious->boxID]++;
                    }
                }
            }
        }
    }
    std::cout << "Pass 1" << std::endl;

    for (int i=0; i<currFrame.boundingBoxes.size(); i++){
        for (int j=0; j<prevFrame.boundingBoxes.size(); j++){
            if (bins[i][j]>max){
                max = bins[i][j];
                indMax = j;
            }
        }
        std::cout << "max = " <<  max << std::endl;
        if(max>=threshold && !processed[indMax]){
            bbBestMatches.insert({prevFrame.boundingBoxes[indMax].boxID,currFrame.boundingBoxes[i].boxID});
            processed[indMax] = true;
        }
        max = 0;
        indMax = 0;
    }
    std::cout << "Pass 2" << std::endl;
}