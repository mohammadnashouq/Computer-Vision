#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core/core.hpp"

#include "opencv2/highgui/highgui.hpp"

#include "opencv2/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"


using namespace std;


using namespace cv;
/*
typedef struct RANSACDiffs {
    int inliersNum;  //number of inliers
    vector<float> distances; //point-plane distances
    vector<bool> isInliers; //vector of inlier/outlier segmenttion results; true: inlier false: outlier
} RANSACDiffs;

RANSACDiffs HomographyRANSACDifferences(vector<pair<Point2f, Point2f>> pts, Mat H, float threshold) {
    int num = pts.size();
    Mat invTr = H.inv();

    
    RANSACDiffs ret;
    

    vector<bool> isInliers;
    vector<float> distances;

    int inlierCounter = 0;
    for (int idx = 0; idx < num; idx++) {
        
        Mat pt(3, 1, CV_32F);
        
        

        pair< Point2f , Point2f> pair_of_points = pts.at(idx);
        Point2f first_point = pair_of_points.first;
        Point2f second_point = pair_of_points.second;
        
        pt.at<float>(0, 0) = first_point.x;
        pt.at<float>(1, 0) = first_point.y;
        pt.at<float>(2, 0) = 1.0;

        Mat  first_point_projection = invTr * pt;
        float x_diff = second_point.x - first_point_projection.at<float>(0,0);
        float y_diff = second_point.y - first_point_projection.at<float>(1, 0);
        
        float diff = fabs(sqrt(x_diff* x_diff + y_diff * y_diff));
        distances.push_back(diff);
        if (diff < threshold) {
            isInliers.push_back(true);
            inlierCounter++;
        }
        else {
            isInliers.push_back(false);
        }

    }//end for idx;

    ret.distances = distances;
    ret.isInliers = isInliers;
    ret.inliersNum = inlierCounter;
    return ret;
}
*/
/*

Mat calcHomography(vector<pair<Point2f, Point2f> > pointPairs) {
    const int ptsNum = pointPairs.size();
    Mat A(2 * ptsNum, 9, CV_32F);
    for (int i = 0; i < ptsNum; i++) {
        float u1 = pointPairs[i].first.x;
        float v1 = pointPairs[i].first.y;

        float u2 = pointPairs[i].second.x;
        float v2 = pointPairs[i].second.y;

        A.at<float>(2 * i, 0) = u1;
        A.at<float>(2 * i, 1) = v1;
        A.at<float>(2 * i, 2) = 1.0f;
        A.at<float>(2 * i, 3) = 0.0f;
        A.at<float>(2 * i, 4) = 0.0f;
        A.at<float>(2 * i, 5) = 0.0f;
        A.at<float>(2 * i, 6) = -u2 * u1;
        A.at<float>(2 * i, 7) = -u2 * v1;
        A.at<float>(2 * i, 8) = -u2;

        A.at<float>(2 * i + 1, 0) = 0.0f;
        A.at<float>(2 * i + 1, 1) = 0.0f;
        A.at<float>(2 * i + 1, 2) = 0.0f;
        A.at<float>(2 * i + 1, 3) = u1;
        A.at<float>(2 * i + 1, 4) = v1;
        A.at<float>(2 * i + 1, 5) = 1.0f;
        A.at<float>(2 * i + 1, 6) = -v2 * u1;
        A.at<float>(2 * i + 1, 7) = -v2 * v1;
        A.at<float>(2 * i + 1, 8) = -v2;

    }

    Mat eVecs(9, 9, CV_32F), eVals(9, 9, CV_32F);
    cout << A << endl;
    eigen(A.t() * A, eVals, eVecs);

    cout << eVals << endl;
    cout << eVecs << endl;


    Mat H(3, 3, CV_32F);
    for (int i = 0; i < 9; i++) H.at<float>(i / 3, i % 3) = eVecs.at<float>(8, i);

    cout << H << endl;

    //Normalize:
    H = H * (1.0 / H.at<float>(2, 2));
    cout << H << endl;

    return H;
}
*/
/*
Mat EstimateHomographyRANSAC(vector<pair<Point2f, Point2f>> pts, float threshold, int iterateNum) {
    int num = pts.size();

    float bestSampleInlierNum = 0;
    Mat bestHomogrphy;

    for (int iter = 0; iter < iterateNum; iter++) {


        float rand1 = (float)(rand()) / RAND_MAX;
        float rand2 = (float)(rand()) / RAND_MAX;
        float rand3 = (float)(rand()) / RAND_MAX;
        float rand4 = (float)(rand()) / RAND_MAX;


        //Generate three different(!) random numbers:
        int index1 = (int)(rand1 * num);
        int index2 = (int)(rand2 * num);
        while (index2 == index1) { rand2 = (float)(rand()) / RAND_MAX; index2 = (int)(rand2 * num); }
        int index3 = (int)(rand3 * num);
        while ((index3 == index1) || (index3 == index2)) { rand3 = (float)(rand()) / RAND_MAX; index3 = (int)(rand3 * num); }
        int index4 = (int)(rand4 * num);
        while ((index4 == index1) || (index4 == index2) || (index4 == index3)) { rand4 = (float)(rand()) / RAND_MAX; index4 = (int)(rand4 * num); }



        pair<Point2f, Point2f> pt1 = pts.at(index1);
        pair<Point2f, Point2f> pt2 = pts.at(index2);
        pair<Point2f, Point2f> pt3 = pts.at(index3);
        pair<Point2f, Point2f> pt4 = pts.at(index4);

        //In each RANSAC cycle, a minimal sample with 3 points are formed

        vector<pair<Point2f, Point2f>> minimalSample;

        minimalSample.push_back(pt1);
        minimalSample.push_back(pt2);
        minimalSample.push_back(pt3);
        minimalSample.push_back(pt4);

        Mat Homography_sample = calcHomography(minimalSample);

        //            printf("Plane params: %f %f %f %f \n",samplePlane[0],samplePlane[1],samplePlane[2],samplePlane[3]);

                    //Compute consensus set

        RANSACDiffs sampleResult = HomographyRANSACDifferences(pts, Homography_sample, threshold);

        //            printf("NumInliers: %d \n",sampleResult.inliersNum);

                    //Check the new test is larger than the best one.

        if (sampleResult.inliersNum > bestSampleInlierNum) {
            bestSampleInlierNum = sampleResult.inliersNum;
            bestHomogrphy = Homography_sample;
            
        }//end if

        //delete[] samplePlane;

    }//end for iter

    //Finally, the plane is refitted from thew best consensus set
    RANSACDiffs bestResult = HomographyRANSACDifferences(pts, bestHomogrphy, threshold);


    vector<pair<Point2f, Point2f>> inlierPts;


    for (int idx = 0; idx < num; idx++) {
        if (bestResult.isInliers.at(idx)) {
            inlierPts.push_back(pts.at(idx));
        }
    }

    Mat final_homography = calcHomography(inlierPts);

    return final_homography;

}

*/