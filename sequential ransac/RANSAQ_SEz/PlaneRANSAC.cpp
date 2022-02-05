#include "MatrixReaderWriter.h"
#include "PlaneEstimation.h"
#include "PLYWriter.h"
#include <opencv2/opencv.hpp>
#include <vector>


using namespace cv;

#define dominant_plane_num 3  //RANSAC threshold (if Velodyne scans are processed, the unit is meter)
// Tunnal 
//#define THERSHOLD 0.3  //RANSAC threshold (if Velodyne scans are processed, the unit is meter)

//#define RANSAC_ITER  200    //RANSAC iteration

//#define FILTER_LOWEST_DISTANCE 0.3 //threshold for pre-filtering

// tuninig parameters
//#define THERSHOLD 3  //RANSAC threshold (if Velodyne scans are processed, the unit is meter)

//#define RANSAC_ITER  1000    //RANSAC iteration

//#define FILTER_LOWEST_DISTANCE 0.3 //threshold for pre-filtering

// street parameters
//#define THERSHOLD 0.2  //RANSAC threshold (if Velodyne scans are processed, the unit is meter)

//#define RANSAC_ITER  1000    //RANSAC iteration

//#define FILTER_LOWEST_DISTANCE 0.3 //threshold for pre-filtering

// ParkingPlace_corr parameters
//#define THERSHOLD 0.3  //RANSAC threshold (if Velodyne scans are processed, the unit is meter)

//#define RANSAC_ITER  2000    //RANSAC iteration

//#define FILTER_LOWEST_DISTANCE 0.3 //threshold for pre-filtering


// ParkingPlace   parameters
//#define THERSHOLD 0.3  //RANSAC threshold (if Velodyne scans are processed, the unit is meter)

//#define RANSAC_ITER  200    //RANSAC iteration

//#define FILTER_LOWEST_DISTANCE 0.3 //threshold for pre-filtering

// perfect garage parameters
//#define THERSHOLD 0.3  //RANSAC threshold (if Velodyne scans are processed, the unit is meter)

//#define RANSAC_ITER  200    //RANSAC iteration

//#define FILTER_LOWEST_DISTANCE 0.3 //threshold for pre-filtering

// dense   parameters
//#define THERSHOLD 1.4  //RANSAC threshold (if Velodyne scans are processed, the unit is meter)

//#define RANSAC_ITER  200    //RANSAC iteration

//#define FILTER_LOWEST_DISTANCE 0.3 //threshold for pre-filtering

// before cross   parameters
//2.6 -- 2000

// chaine bring
//2.5  3000 0.3
// Garden parameters
// 1 300 o,3

//ParkingPlace_corr
// 1 300 0.3
//tunning parameters
// 4 2000 0.3
// Street parameters
// 1.4 200 0.3
#define THERSHOLD 1.4  //RANSAC threshold(if Velodyne scans are processed, the unit is meter)

#define RANSAC_ITER  200 //RANSAC iteration

#define FILTER_LOWEST_DISTANCE 0.3 //threshold for pre-filtering

vector<Point3f>  get_outlier_points(vector<Point3f> points, RANSACDiffs differences) {

    vector<Point3f> new_points;
    for (int i = 0; i < points.size(); i++) {

        if (not differences.isInliers.at(i))

            new_points.push_back(points.at(i));

    }
    return new_points;
}


int main(int argc, char** argv){
    
    if (argc!=3){
        printf("Usage:\n PlanRansac input.xyz output.ply\n");
        exit(EXIT_FAILURE);
    }
    
    MatrixReaderWriter mrw(argv[1]);
    
    int num=mrw.rowNum;
    
    cout<< "Rows:" << num <<endl;
    cout<< "Cols:" << mrw.columnNum << endl;



    //Read data from text file
    
    vector<Point3f> points;
    vector<Point3f> totla_points;
    vector<Point3f> final_points;
    vector<Point3i> final_color;

    for (int idx = 0; idx < num; idx++) {
        double x = mrw.data[3 * idx];
        double y = mrw.data[3 * idx + 1];
        double z = mrw.data[3 * idx + 2];

        float distFromOrigo = sqrt(x * x + y * y + z * z);


        //First filter: minimal work distance for a LiDAR limited.        

        if (distFromOrigo > FILTER_LOWEST_DISTANCE) {
            Point3f newPt;
            newPt.x = x;
            newPt.y = y;
            newPt.z = z;
            points.push_back(newPt);
        }

        
    };
    
    totla_points = points;
    
    //Number of points:
    for (int dominant_plane = 0; dominant_plane < dominant_plane_num; dominant_plane++)
    {

        num = points.size();

        cout << "--------------- Number of point in iteration " << dominant_plane << " num  " << num<< endl;


        

        //RANSAC-based robust estimation

        float* planeParams = EstimatePlaneRANSAC(points, THERSHOLD, RANSAC_ITER);


        printf("Plane params RANSAC:\n A:%f B:%f C:%f D:%f \n", planeParams[0], planeParams[1], planeParams[2], planeParams[3]);

        //Compute differences of the fitted plane in order to separate inliers from outliers

        RANSACDiffs differences = PlanePointRANSACDifferences(points, planeParams, THERSHOLD);

        //delete[] planeParams;


        //Inliers and outliers are coloured by green and red, respectively

        vector<Point3i> colorsRANSAC;
        if (dominant_plane == 0) {
        for (int idx = 0; idx < num; idx++) {
            Point3i newColor;

            if (differences.isInliers.at(idx)) {
                newColor.x = 255;
                newColor.y = 0;
                newColor.z = 0;
                final_points.push_back(points.at(idx));
                final_color.push_back(newColor);
            }
            else {
                newColor.x = 0;
                newColor.y = 0;
                newColor.z = 255;
            }

            colorsRANSAC.push_back(newColor);


        }
        }
        else {
            if (dominant_plane == 1) {
                for (int idx = 0; idx < num; idx++) {
                    Point3i newColor;

                    if (differences.isInliers.at(idx)) {
                        newColor.x = 0;
                        newColor.y = 255;
                        newColor.z = 0;
                        final_points.push_back(points.at(idx));
                        final_color.push_back(newColor);
                    }
                    else {
                        newColor.x = 0;
                        newColor.y = 0;
                        newColor.z = 255;
                    }

                    colorsRANSAC.push_back(newColor);


                }
            }
            else
            {
                if (dominant_plane == 2) {
                    for (int idx = 0; idx < num; idx++) {
                        Point3i newColor;

                        if (differences.isInliers.at(idx)) {
                            newColor.x = 255;
                            newColor.y = 192;
                            newColor.z = 203;
                            final_points.push_back(points.at(idx));
                            final_color.push_back(newColor);
                        }
                        else {
                            newColor.x = 0;
                            newColor.y = 0;
                            newColor.z = 255;
                            final_points.push_back(points.at(idx));
                            final_color.push_back(newColor);
                        }

                        colorsRANSAC.push_back(newColor);


                    }
                }

            }
        }

        //Write results into a PLY file. 
        //It can be isualized by open-source 3D application Meshlab (www.meshlab.org)
        
        
        //WritePLY_i(argv[2], points, colorsRANSAC, dominant_plane);
        points = get_outlier_points(points, differences);

    }
    WritePLY(argv[2], final_points, final_color);

}
