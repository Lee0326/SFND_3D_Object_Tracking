
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
            cv::circle(topviewImg, cv::Point(x, y), 2, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,255), 2);

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
    // ...
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    unordered_set<int> indices_prev;
    unordered_set<int> indices_curr;
    int maxIterations = 100;
    float distanceTol = 0.2;
    double min_prev = 1e8;
    double min_curr = 1e8;
    Ransac(indices_prev, lidarPointsPrev, maxIterations, distanceTol);
    Ransac(indices_curr, lidarPointsCurr, maxIterations, distanceTol);
    for (auto index:indices_prev)
       min_prev = lidarPointsPrev[index].x < min_prev? lidarPointsPrev[index].x:min_prev;
    for (auto index:indices_curr)
       min_curr = lidarPointsCurr[index].x < min_curr? lidarPointsCurr[index].x:min_curr;
    TTC = min_curr/(frameRate*(min_prev - min_curr));
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    vector<cv::DMatch> matches_temp = matches;
    for (vector<cv::DMatch>::iterator it1=matches_temp.begin(); it1!=matches_temp.end(); ++it1)
    {
        vector<BoundingBox> enclosed_boxes_prev;
        vector<BoundingBox> enclosed_boxes_curr;
        int enclose_count_prev = 0;
        int enclose_count_curr = 0;
        for (auto bb_prev:prevFrame.boundingBoxes)
        {
            if (bb_prev.roi.contains(prevFrame.keypoints[it1->queryIdx].pt)) enclose_count_prev += 1;
        }
        for (auto bb_curr:currFrame.boundingBoxes)
        {
            if (bb_curr.roi.contains(currFrame.keypoints[it1->trainIdx].pt)) enclose_count_curr += 1;
        }
        // cout << "the previous count is: " << enclose_count_prev << endl;
        // cout << "the current count is: " << enclose_count_curr << endl;
        if ((enclose_count_prev != 1) || (enclose_count_curr != 1)) 
        {
            matches_temp.erase(it1);
            it1--;
        }
    }
    

    pair<int, int> map_can;
    for (auto BoundingBox_prev:prevFrame.boundingBoxes)
    {
        map<int , size_t> map_count;
        int max_count = 10;
        for (auto BoundingBox_curr:currFrame.boundingBoxes)
        {
            int train_Box_Idx = BoundingBox_curr.boxID;
            for (vector<cv::DMatch>::iterator it=matches_temp.begin(); it!=matches_temp.end(); ++it)
            {
                if (BoundingBox_prev.roi.contains(prevFrame.keypoints[it->queryIdx].pt) &&
                BoundingBox_curr.roi.contains(currFrame.keypoints[it->trainIdx].pt)) ++map_count[train_Box_Idx];

            }
            if (map_count[train_Box_Idx] > max_count)
            {
                max_count = map_count[train_Box_Idx];
                map_can.first = BoundingBox_prev.boxID;
                map_can.second = BoundingBox_curr.boxID;
            }
        }
        bbBestMatches.insert(map_can);
    }
    // ...
    // for (auto bestmatch:bbBestMatches) 
    // {
    //     cout << "the best matches is: " << bestmatch.first << " and " << bestmatch.second << endl;
    // }
    // cout << "the size of bouding boxes of prev: " << prevFrame.boundingBoxes.size() << endl;
    // cout << "the size of bouding boxes of curr: " << currFrame.boundingBoxes.size() << endl;
}

void Ransac(std::unordered_set<int> &indices, std::vector<LidarPoint> &lidarPointsPrev, int maxIterations,const float distanceTol)
{	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit plane

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted lane with most inliers
	while (maxIterations--)
	{
		std::unordered_set<int> inliers;
		while (inliers.size() <3)
			inliers.insert(rand()%(lidarPointsPrev.size()));

		float x1, x2, x3, y1, y2, y3, z1, z2, z3;
		auto itr = inliers.begin();
		x1 = lidarPointsPrev[*itr].x;
		y1 = lidarPointsPrev[*itr].y;
		z1 = lidarPointsPrev[*itr].z;
		itr++;
		x2 = lidarPointsPrev[*itr].x;
		y2 = lidarPointsPrev[*itr].y;
		z2 = lidarPointsPrev[*itr].z;
		itr++;
		x3 = lidarPointsPrev[*itr].x;
		y3 = lidarPointsPrev[*itr].y;
		z3 = lidarPointsPrev[*itr].z;

		float A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		float D = -(A*x1 + B*y1 + C*z1);

		for (int index = 0; index < lidarPointsPrev.size(); index++)
		{
			if (inliers.count(index)>0)
				continue;
		    LidarPoint point = lidarPointsPrev[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;
			float d = fabs(A*x4 + B*y4 + C*z4 +D)/sqrt(A*A + B*B + C*C);
			if (d <= distanceTol)
				inliers.insert(index);
		}

		if (inliers.size() > indices.size())
		{
			indices = inliers;
		}
	}
    //std::cout << "using own created ransac" << std::endl;
}
