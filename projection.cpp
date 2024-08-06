/*
    Sachin Palahalli Chandrakumar
    Spring 2024
    Code to implement projection of virtual objects
*/

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>

using namespace std;
using namespace cv;

int read_image_data_csv( char *filename, std::vector<char *> &filenames, std::vector<std::vector<float>> &data, int echo_file );

/**
 * @brief reads caliberation csv file and gets the values as matrix.
 * This function take the path where the caliberation data  is present and then 
 * loads all the data present in the file to memory
 * @param[in] fileName The csv file name.
 * @param[in] matrixNames The caliberation and distortion file names
 * @param[in] matrixData The 2D list of all caliberation data
 * @return 0.
*/
int getCaliberationData(string fileName, std::vector<char*> &matrixNames, std::vector<std::vector<float>> &matrixData){
    
    int echo_file = 0;
    read_image_data_csv( &fileName[0], matrixNames, matrixData, echo_file );
    int totalFiles = matrixNames.size();
    int totalData = matrixData.size();
    if(totalFiles != totalData){
        cout<<" Number of Image names and the filter vectores present sould be same!";
        exit(-1);
    }
    return 0;
}

/**
 * @brief stores different color combinations
 * @param[in] colorVector vector containing color combinations
 * @return 0.
*/
int getColorVector(vector<Scalar> &colorVector){
    for(int i=0;i<4;i++){
        colorVector.push_back(Scalar(0, 0, 0));
    }
    for(int i=0;i<4;i++){
        colorVector.push_back(Scalar(0, 0, 255));
    }
    for(int i=0;i<4;i++){
        colorVector.push_back(Scalar(0, 255, 0));
    }
    for(int i=0;i<4;i++){
        colorVector.push_back(Scalar(0, 255, 255));
    }
    for(int i=0;i<4;i++){
        colorVector.push_back(Scalar(255, 0, 0));
    }
    for(int i=0;i<4;i++){
        colorVector.push_back(Scalar(255, 0, 255));
    }
    for(int i=0;i<4;i++){
        colorVector.push_back(Scalar(255, 255, 0));
    }
    for(int i=0;i<4;i++){
        colorVector.push_back(Scalar(100, 0, 255));
    }

    return 0;
}

/**
 * @brief screates chair virtual object
 * @param[in] frame image frame
 * @param[in] objectPoints 3-d object points
 * @param[in] translation_vector translation vector of image
 * @param[in] camera_matrix caliberation matrix values
 * @param[in] distortionCoefficients distortionCoefficients
 * @return 0.
*/
int create_chair(Mat &frame, Mat rotation_vector, Mat translation_vector, Mat camera_matrix, Mat distortionCoefficients){
    std::vector<cv::Point3f> objectPoints;
            
    //creating chair

    //creating seat at z 4
    objectPoints.push_back(cv::Point3f(0, 0, 4)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(3, 0, 4));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(3, -4, 4)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(0, -4, 4)); // (y0,x4,z4) last 4 points reqresent square on z4 

    objectPoints.push_back(cv::Point3f(0, 0, 5)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(3, 0, 5));  // (y3,x0,z5)
    objectPoints.push_back(cv::Point3f(3, -4, 5)); // (y3,x4,z5)
    objectPoints.push_back(cv::Point3f(0, -4, 5)); // (y0,x4,z5) last 4 points reqresent square on z5 axis
    
    
    objectPoints.push_back(cv::Point3f(3, -4, 7)); // (y3,x4,z5)
    objectPoints.push_back(cv::Point3f(3, 0, 7)); // (y0,x4,z5) last 4 points reqresent square on z5 axis

    //top red line
    objectPoints.push_back(cv::Point3f(4, -4, 7)); // (y3,x4,z5)
    objectPoints.push_back(cv::Point3f(4, 0, 7)); // (y0,x4,z5) last 4 points reqresent square on z5 axis

    objectPoints.push_back(cv::Point3f(4, -4, 4)); // (y3,x4,z5)
    objectPoints.push_back(cv::Point3f(4, 0, 4)); // (y0,x4,z5) last 4 points reqresent square on z5 axis

    //points for base of chair stand
    objectPoints.push_back(cv::Point3f(0, 0, 0)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(3, 0, 0));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(3, -4, 0)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(0, -4, 0)); // (y0,x4,z4) last 4 points reqresent square on z4 

    int cylynderPoints = 32;
    double radius = 0.5;
    int height = 4;
    double angle, x,y,z;
    // front left leg
    for(int i=0; i< cylynderPoints; i++){
        angle = i * (2 * CV_PI / cylynderPoints);
        x = 0 + radius*(cos(angle));
        y = 0 + radius*(sin(angle));
        z = 0;
        objectPoints.push_back(cv::Point3f(x, y, z)); //bottom
        objectPoints.push_back(cv::Point3f(x, y, z+4)); //top
    }

    // back left leg
    for(int i=0; i< cylynderPoints; i++){
        angle = i * (2 * CV_PI / cylynderPoints);
        x = 3 + radius*(cos(angle));
        y = 0 + radius*(sin(angle));
        z = 0;
        objectPoints.push_back(cv::Point3f(x, y, z)); //bottom
        objectPoints.push_back(cv::Point3f(x, y, z+4)); //top
    }

    // back right leg
    for(int i=0; i< cylynderPoints; i++){
        angle = i * (2 * CV_PI / cylynderPoints);
        x = 3 + radius*(cos(angle));
        y = -4 + radius*(sin(angle));
        z = 0;
        objectPoints.push_back(cv::Point3f(x, y, z)); //bottom
        objectPoints.push_back(cv::Point3f(x, y, z+4)); //top
    }

    // front right leg
    for(int i=0; i< cylynderPoints; i++){
        angle = i * (2 * CV_PI / cylynderPoints);
        x = 0 + radius*(cos(angle));
        y = -4 + radius*(sin(angle));
        z = 0;
        objectPoints.push_back(cv::Point3f(x, y, z)); //bottom
        objectPoints.push_back(cv::Point3f(x, y, z+4)); //top
    }


    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(objectPoints, rotation_vector, translation_vector, camera_matrix, distortionCoefficients, imagePoints);

    //connecting squares
    for (int i = 0; i < 8; i ++) {
        if((i+1)%4 == 0){
            cout<<"i = "<<i<<endl;
            cv::line(frame, imagePoints[i], imagePoints[i -3], Scalar(0, 255, 0), 2);    
        }else{
            cv::line(frame, imagePoints[i], imagePoints[i + 1], Scalar(0, 255, 0), 2);
        }
    }
    for (int i = 0; i < 4; i ++){
    cv::line(frame, imagePoints[i], imagePoints[i + 4], Scalar(0, 255, 0), 2);
    }

        // create top of chair
    cv::line(frame, imagePoints[8], imagePoints[9], Scalar(0, 0, 255), 2);
    cv::line(frame, imagePoints[8], imagePoints[6], Scalar(0, 0, 255), 2);
    cv::line(frame, imagePoints[9], imagePoints[5], Scalar(0, 0, 255), 2);

    //drawind back top red chair part
    cv::line(frame, imagePoints[10], imagePoints[11], Scalar(0, 0, 255), 2);
    cv::line(frame, imagePoints[12], imagePoints[13], Scalar(0, 0, 255), 2);
    cv::line(frame, imagePoints[10], imagePoints[12], Scalar(0, 0, 255), 2);
    cv::line(frame, imagePoints[11], imagePoints[13], Scalar(0, 0, 255), 2);
    cv::line(frame, imagePoints[12], imagePoints[2], Scalar(0, 0, 255), 2);
    cv::line(frame, imagePoints[13], imagePoints[1], Scalar(0, 0, 255), 2);
    cv::line(frame, imagePoints[8], imagePoints[10], Scalar(0, 0, 255), 2);
    cv::line(frame, imagePoints[9], imagePoints[11], Scalar(0, 0, 255), 2);

    // drawing chair legs
    for (int i = 18; i < imagePoints.size(); i += 2) {
        cv::line(frame, imagePoints[i], imagePoints[i + 1], Scalar(255, 255, 0), 2);
    }

    return 0;
}


/**
 * @brief creates car virtual object
 * @param[in] frame image frame
 * @param[in] objectPoints 3-d object points
 * @param[in] translation_vector translation vector of image
 * @param[in] camera_matrix caliberation matrix values
 * @param[in] distortionCoefficients distortionCoefficients
 * @return 0.
*/
int create_car(Mat &frame, Mat rotation_vector, Mat translation_vector, Mat camera_matrix, Mat distortionCoefficients, vector<Scalar> colorVector, int frameIndex){

    std::vector<cv::Point3f> objectPoints;
            
    //creating car (z axis will have y axis values, y axis will have z axis values)

    //creating seat at z 4
    objectPoints.push_back(cv::Point3f(2, 0, 0)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(2, 0, 4));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(2, -8, 4)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(2, -8, 0)); // (y0,x4,z4) last 4 points reqresent square on z4 
    objectPoints.push_back(cv::Point3f(2.2, 0, 0)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(2.2, 0, 4));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(2.2, -8, 4)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(2.2, -8, 0)); // (y0,x4,z4) last 4 points reqresent square on z4 

    objectPoints.push_back(cv::Point3f(3, 0, 0)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(3, 0, 4));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(3, -8, 4)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(3, -8, 0)); // (y0,x4,z4) last 4 points reqresent square on z4 
    objectPoints.push_back(cv::Point3f(4.5, -2, 0)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(4.5, -2, 4));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(4.5, -6, 4)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(4.5, -6, 0)); // (y0,x4,z4) last 4 points reqresent square on z4 

    objectPoints.push_back(cv::Point3f(3, -1, 0)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(3, -1, 4));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(3, -7, 4)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(3, -7, 0)); // (y0,x4,z4) last 4 points reqresent square on z4 
    objectPoints.push_back(cv::Point3f(3.5, -1, 0)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(3.5, -1, 4));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(3.5, -7, 4)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(3.5, -7, 0)); // (y0,x4,z4) last 4 points reqresent square on z4 

    objectPoints.push_back(cv::Point3f(3, -2, 0)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(3, -2, 4));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(3, -6, 4)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(3, -6, 0)); // (y0,x4,z4) last 4 points reqresent square on z4 

    // MID LINE
    objectPoints.push_back(cv::Point3f(3, -4, 0)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(3, -4, 4));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(4.5, -4, 0)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(4.5, -4, 4)); // (y0,x4,z4) last 4 points reqresent square on z4 

    // objectPoints.push_back(cv::Point3f(3.5, -2, 4)); // (y0,x0,z4)
    // objectPoints.push_back(cv::Point3f(3.5, -2.5, 4)); // (y3,x4,z4)
    // objectPoints.push_back(cv::Point3f(4, -2.5, 4)); // (y3,x0,z4)
    // objectPoints.push_back(cv::Point3f(4, -2, 4));  // (y0,x4,z4) last 4 points reqresent square on z4 




    int cylynderPoints = 32;
    double radius = 0.5;
    double width = 0.5;
    double angle, x,y,z;
    // left back wheel
    for(int i=0; i< cylynderPoints; i++){
        angle = i * (2 * CV_PI / cylynderPoints);
        x = 2 + radius*(cos(angle));
        y = -1.5 + radius*(sin(angle));
        z = -0.5;
        objectPoints.push_back(cv::Point3f(x, y, z)); //bottom
        objectPoints.push_back(cv::Point3f(x, y, z+width)); //top
    }

    // left front wheel
    for(int i=0; i< cylynderPoints; i++){
        angle = i * (2 * CV_PI / cylynderPoints);
        x = 2 + radius*(cos(angle));
        y = -1.5 + radius*(sin(angle));
        z = 3.5;
        objectPoints.push_back(cv::Point3f(x, y, z)); //bottom
        objectPoints.push_back(cv::Point3f(x, y, z+width)); //top
    }

    // right front wheel
    for(int i=0; i< cylynderPoints; i++){
        angle = i * (2 * CV_PI / cylynderPoints);
        x = 2 + radius*(cos(angle));
        y = -6.5 + radius*(sin(angle));
        z = 3.5;
        objectPoints.push_back(cv::Point3f(x, y, z)); //bottom
        objectPoints.push_back(cv::Point3f(x, y, z+width)); //top
    }

    // right back wheel
    for(int i=0; i< cylynderPoints; i++){
        angle = i * (2 * CV_PI / cylynderPoints);
        x = 2 + radius*(cos(angle));
        y = -6.5 + radius*(sin(angle));
        z = -0.5;
        objectPoints.push_back(cv::Point3f(x, y, z)); //bottom
        objectPoints.push_back(cv::Point3f(x, y, z+width)); //top
    }

    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(objectPoints, rotation_vector, translation_vector, camera_matrix, distortionCoefficients, imagePoints);

    //connecting squares
    for (int i = 0; i < 4; i ++) {
        if((i+1)%4 == 0){
            // cout<<"i = "<<i<<endl;
            cv::line(frame, imagePoints[i], imagePoints[i -3], Scalar(0, 255, 0), 2);    
            cv::line(frame, imagePoints[i+4], imagePoints[i + 4 -3], Scalar(0, 255, 0), 2);  
            cv::line(frame, imagePoints[i+8], imagePoints[i + 8 -3], Scalar(0, 255, 0), 2);    
            cv::line(frame, imagePoints[i+12], imagePoints[i + 12 -3], Scalar(0, 255, 0), 2);    
        }else{
            cv::line(frame, imagePoints[i], imagePoints[i + 1], Scalar(0, 255, 0), 2);
            cv::line(frame, imagePoints[i+4], imagePoints[i + 4 + 1], Scalar(0, 255, 0), 2);
            cv::line(frame, imagePoints[i+8], imagePoints[i + 8 + 1], Scalar(0, 255, 0), 2);
            cv::line(frame, imagePoints[i+12], imagePoints[i + 12 + 1], Scalar(0, 255, 0), 2);
        }
        cv::line(frame, imagePoints[i], imagePoints[i + 4], Scalar(0, 255, 0), 2);
        cv::line(frame, imagePoints[i+4], imagePoints[i + 8], Scalar(0, 255, 0), 2);
        cv::line(frame, imagePoints[i+16], imagePoints[i + 20], Scalar(0, 255, 0), 2);
        // cv::line(frame, imagePoints[i+8], imagePoints[i + 12], Scalar(0, 255, 0), 2);
    }
    cv::line(frame, imagePoints[20], imagePoints[21], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[22], imagePoints[23], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[8], imagePoints[20], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[9], imagePoints[21], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[10], imagePoints[22], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[11], imagePoints[23], Scalar(0, 255, 0), 2);

    cv::line(frame, imagePoints[12], imagePoints[20], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[13], imagePoints[21], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[14], imagePoints[22], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[15], imagePoints[23], Scalar(0, 255, 0), 2);

    //WINDOWS
    cv::line(frame, imagePoints[12], imagePoints[24], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[13], imagePoints[25], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[14], imagePoints[26], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[15], imagePoints[27], Scalar(0, 255, 0), 2);

    // MID LINE
    cv::line(frame, imagePoints[28], imagePoints[30], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[29], imagePoints[31], Scalar(0, 255, 0), 2);
    // cv::line(frame, imagePoints[14], imagePoints[26], Scalar(0, 255, 0), 2);
    // cv::line(frame, imagePoints[15], imagePoints[27], Scalar(0, 255, 0), 2);

    int count=0;
    int colorIndex = frameIndex%cylynderPoints;

    int start_index;
    int endIndex = 32;
    int totalPoints;
    for(int k=0; k<4;k++){
        start_index = endIndex;
        endIndex = start_index + 2*cylynderPoints;
        totalPoints = endIndex - start_index;
        count=0;
        colorIndex = frameIndex%cylynderPoints;
        for (int i = start_index; i < endIndex; i += 2) {
            colorIndex = colorIndex%cylynderPoints;
            cv::line(frame, imagePoints[i], imagePoints[i + 1], colorVector[colorIndex], 2);
            if(count <= totalPoints/4){
                // cout<<"count = "<<count<<endl;
                // cout<<"total points = "<<totalPoints<<endl;
                cv::line(frame, imagePoints[i], imagePoints[i + cylynderPoints-1], colorVector[colorIndex], 2);
            }
            colorIndex++;
            count++;
        }
    }



    return 0;
}

/**
 * @brief integrates car virtual object
 * @param[in] frame image frame
 * @param[in] objectPoints 3-d object points
 * @param[in] translation_vector translation vector of image
 * @param[in] camera_matrix caliberation matrix values
 * @param[in] distortionCoefficients distortionCoefficients
 * @return 0.
*/
int integrate_car(Mat &frame, Mat rotation_vector, Mat translation_vector, Mat camera_matrix, Mat distortionCoefficients, vector<Scalar> colorVector, int frameIndex){

    std::vector<cv::Point3f> objectPoints;
            
    //creating car (z axis will have y axis values, y axis will have z axis values)

    //creating seat at z 4
    objectPoints.push_back(cv::Point3f(2+3, 0, 0)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(2+3, 0, 4));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(2+3, -8, 4)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(2+3, -8, 0)); // (y0,x4,z4) last 4 points reqresent square on z4 
    objectPoints.push_back(cv::Point3f(2.2+3, 0, 0)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(2.2+3, 0, 4));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(2.2+3, -8, 4)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(2.2+3, -8, 0)); // (y0,x4,z4) last 4 points reqresent square on z4 

    objectPoints.push_back(cv::Point3f(3+3, 0, 0)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(3+3, 0, 4));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(3+3, -8, 4)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(3+3, -8, 0)); // (y0,x4,z4) last 4 points reqresent square on z4 
    objectPoints.push_back(cv::Point3f(4.5+3, -2, 0)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(4.5+3, -2, 4));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(4.5+3, -6, 4)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(4.5+3, -6, 0)); // (y0,x4,z4) last 4 points reqresent square on z4 

    objectPoints.push_back(cv::Point3f(3+3, -1, 0)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(3+3, -1, 4));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(3+3, -7, 4)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(3+3, -7, 0)); // (y0,x4,z4) last 4 points reqresent square on z4 
    objectPoints.push_back(cv::Point3f(3.5+3, -1, 0)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(3.5+3, -1, 4));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(3.5+3, -7, 4)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(3.5+3, -7, 0)); // (y0,x4,z4) last 4 points reqresent square on z4 

    objectPoints.push_back(cv::Point3f(3+3, -2, 0)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(3+3, -2, 4));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(3+3, -6, 4)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(3+3, -6, 0)); // (y0,x4,z4) last 4 points reqresent square on z4 

    // MID LINE
    objectPoints.push_back(cv::Point3f(3+3, -4, 0)); // (y0,x0,z4)
    objectPoints.push_back(cv::Point3f(3+3, -4, 4));  // (y3,x0,z4)
    objectPoints.push_back(cv::Point3f(4.5+3, -4, 0)); // (y3,x4,z4)
    objectPoints.push_back(cv::Point3f(4.5+3, -4, 4)); // (y0,x4,z4) last 4 points reqresent square on z4 

    // objectPoints.push_back(cv::Point3f(3.5, -2, 4)); // (y0,x0,z4)
    // objectPoints.push_back(cv::Point3f(3.5, -2.5, 4)); // (y3,x4,z4)
    // objectPoints.push_back(cv::Point3f(4, -2.5, 4)); // (y3,x0,z4)
    // objectPoints.push_back(cv::Point3f(4, -2, 4));  // (y0,x4,z4) last 4 points reqresent square on z4 




    int cylynderPoints = 32;
    double radius = 0.5;
    double width = 0.5;
    double angle, x,y,z;
    // left back wheel
    for(int i=0; i< cylynderPoints; i++){
        angle = i * (2 * CV_PI / cylynderPoints);
        x = 2+3 + radius*(cos(angle));
        y = -1.5 + radius*(sin(angle));
        z = -0.5;
        objectPoints.push_back(cv::Point3f(x, y, z)); //bottom
        objectPoints.push_back(cv::Point3f(x, y, z+width)); //top
    }

    // left front wheel
    for(int i=0; i< cylynderPoints; i++){
        angle = i * (2 * CV_PI / cylynderPoints);
        x = 2+3 + radius*(cos(angle));
        y = -1.5 + radius*(sin(angle));
        z = 3.5;
        objectPoints.push_back(cv::Point3f(x, y, z)); //bottom
        objectPoints.push_back(cv::Point3f(x, y, z+width)); //top
    }

    // right front wheel
    for(int i=0; i< cylynderPoints; i++){
        angle = i * (2 * CV_PI / cylynderPoints);
        x = 2+3 + radius*(cos(angle));
        y = -6.5 + radius*(sin(angle));
        z = 3.5;
        objectPoints.push_back(cv::Point3f(x, y, z)); //bottom
        objectPoints.push_back(cv::Point3f(x, y, z+width)); //top
    }

    // right back wheel
    for(int i=0; i< cylynderPoints; i++){
        angle = i * (2 * CV_PI / cylynderPoints);
        x = 2+3 + radius*(cos(angle));
        y = -6.5 + radius*(sin(angle));
        z = -0.5;
        objectPoints.push_back(cv::Point3f(x, y, z)); //bottom
        objectPoints.push_back(cv::Point3f(x, y, z+width)); //top
    }

    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(objectPoints, rotation_vector, translation_vector, camera_matrix, distortionCoefficients, imagePoints);

    //connecting squares
    for (int i = 0; i < 4; i ++) {
        if((i+1)%4 == 0){
            // cout<<"i = "<<i<<endl;
            cv::line(frame, imagePoints[i], imagePoints[i -3], Scalar(0, 255, 0), 2);    
            cv::line(frame, imagePoints[i+4], imagePoints[i + 4 -3], Scalar(0, 255, 0), 2);  
            cv::line(frame, imagePoints[i+8], imagePoints[i + 8 -3], Scalar(0, 255, 0), 2);    
            cv::line(frame, imagePoints[i+12], imagePoints[i + 12 -3], Scalar(0, 255, 0), 2);    
        }else{
            cv::line(frame, imagePoints[i], imagePoints[i + 1], Scalar(0, 255, 0), 2);
            cv::line(frame, imagePoints[i+4], imagePoints[i + 4 + 1], Scalar(0, 255, 0), 2);
            cv::line(frame, imagePoints[i+8], imagePoints[i + 8 + 1], Scalar(0, 255, 0), 2);
            cv::line(frame, imagePoints[i+12], imagePoints[i + 12 + 1], Scalar(0, 255, 0), 2);
        }
        cv::line(frame, imagePoints[i], imagePoints[i + 4], Scalar(0, 255, 0), 2);
        cv::line(frame, imagePoints[i+4], imagePoints[i + 8], Scalar(0, 255, 0), 2);
        cv::line(frame, imagePoints[i+16], imagePoints[i + 20], Scalar(0, 255, 0), 2);
        // cv::line(frame, imagePoints[i+8], imagePoints[i + 12], Scalar(0, 255, 0), 2);
    }
    cv::line(frame, imagePoints[20], imagePoints[21], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[22], imagePoints[23], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[8], imagePoints[20], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[9], imagePoints[21], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[10], imagePoints[22], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[11], imagePoints[23], Scalar(0, 255, 0), 2);

    cv::line(frame, imagePoints[12], imagePoints[20], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[13], imagePoints[21], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[14], imagePoints[22], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[15], imagePoints[23], Scalar(0, 255, 0), 2);

    //WINDOWS
    cv::line(frame, imagePoints[12], imagePoints[24], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[13], imagePoints[25], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[14], imagePoints[26], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[15], imagePoints[27], Scalar(0, 255, 0), 2);

    // MID LINE
    cv::line(frame, imagePoints[28], imagePoints[30], Scalar(0, 255, 0), 2);
    cv::line(frame, imagePoints[29], imagePoints[31], Scalar(0, 255, 0), 2);
    // cv::line(frame, imagePoints[14], imagePoints[26], Scalar(0, 255, 0), 2);
    // cv::line(frame, imagePoints[15], imagePoints[27], Scalar(0, 255, 0), 2);

    int count=0;
    int colorIndex = frameIndex%cylynderPoints;

    int start_index;
    int endIndex = 32;
    int totalPoints;
    for(int k=0; k<4;k++){
        start_index = endIndex;
        endIndex = start_index + 2*cylynderPoints;
        totalPoints = endIndex - start_index;
        count=0;
        colorIndex = frameIndex%cylynderPoints;
        for (int i = start_index; i < endIndex; i += 2) {
            colorIndex = colorIndex%cylynderPoints;
            cv::line(frame, imagePoints[i], imagePoints[i + 1], colorVector[colorIndex], 2);
            if(count <= totalPoints/4){
                // cout<<"count = "<<count<<endl;
                // cout<<"total points = "<<totalPoints<<endl;
                cv::line(frame, imagePoints[i], imagePoints[i + cylynderPoints-1], colorVector[colorIndex], 2);
            }
            colorIndex++;
            count++;
        }
    }



    return 0;
}


int main(int argc, char *argv[]){


    char windowName[256] = "projection";
    namedWindow(windowName, WINDOW_NORMAL);
    vector<char*> matrixNames;
    vector<vector<float>> matrixData;
    string fileName = "/Users/sachinpc/Documents/Northeastern University/Semesters/Fourth Semester/PRCV/Projects/4/utils/caliberationData/caliberationDataFile.csv";
    getCaliberationData(fileName, matrixNames, matrixData);
    vector<float> caliberationMatrixValues = matrixData[0];
    vector<float> distortion_coefficients = matrixData[1];
    Mat camera_matrix(3, 3, CV_64FC1);
    Mat distortionCoefficients(distortion_coefficients.size(), 1, CV_64F, cv::Scalar(0));
    // command line comments map
    unordered_map<string,int> command_map;
    command_map["project_3d_axes"] = 1;
    command_map["project_vobj"] = 2;
    command_map["video"] = 3;
    command_map["s_image"] = 4;

    int command = command_map[argv[1]];
    // string videoFile = "/Users/sachinpc/Documents/Northeastern University/Semesters/Fourth Semester/PRCV/Projects/4/utils/videos/8.MOV"

    char imgFileName[256], videoFileName[256];
    VideoCapture video;
    if(command == 4){
        strcpy(imgFileName, argv[2]);
    }else{
        if(command == 3){
            strcpy(videoFileName, argv[2]);
        }
        if(command == 1 || command == 2){
            strcpy(videoFileName,"/Users/sachinpc/Documents/Northeastern University/Semesters/Fourth Semester/PRCV/Projects/4/utils/videos/9.MOV");
        }
        video.open(videoFileName);
        // std::string imgFileName = "/Users/sachinpc/Documents/Northeastern University/Semesters/Fourth Semester/PRCV/Projects/4/utils/photos/10.png";
        if (!video.isOpened()) {
            cout<< "Cannot read the video file" << endl;
            return 0;
        }
    }

    int k=0;
    for(int i=0;i < 3;i++){
        for(int j=0;j<3;j++){
            camera_matrix.at<double>(i, j) = caliberationMatrixValues[k]; 
            k++;
        }
    }

    for(int i=0; i<distortion_coefficients.size();i++){
        distortionCoefficients.at<double>(i, 0) = distortion_coefficients[i];
    }

    cout << "Camera materix" << camera_matrix <<endl;
    cout << "Distortion coefficients = " << distortionCoefficients<<endl;

    vector<Scalar> colorVector;
    getColorVector(colorVector);


    Mat frame, greyScale;
    bool patternfound = false;
    Size patternsize(6,9);
    int frameNum=0;
    while(true){
        if(command == 4){
            frame = imread(imgFileName);
        }else{
            video.read(frame);
        }
        // imshow(windowName, frame);
        if (frame.empty()) {
            cout<<"Frame empty"<<endl;
            break;
        }
        vector<Point2f> corner_set;
        vector<Vec3f> point_set;
        cvtColor(frame,greyScale, COLOR_BGR2GRAY);
        patternfound = cv::findChessboardCorners(greyScale, patternsize, corner_set);
        cout<<"patternfound = "<<patternfound<<endl;
        if(patternfound){
            cornerSubPix(greyScale, corner_set, Size(11, 11), Size(-1, -1),
                TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        }
        cout<<"Number of Corners for this Frame = "<<corner_set.size()<<endl;
        for (int i = 0; i < patternsize.height; ++i) {
            for (int j = 0; j < patternsize.width; ++j) {
                point_set.push_back(cv::Vec3f(j, -i, 0));
            }
        }
        
        Mat rotation_vector, translation_vector;
        solvePnP(point_set, corner_set, camera_matrix, distortionCoefficients, rotation_vector, translation_vector);
        cout<<"Rotation vector:"<<rotation_vector<<endl;
        cout<<"Translation vector:"<<translation_vector<<endl;

        if(command == 1){
            vector<Point3f> objectPoints;
            vector<Point2f> imagePoints;
            objectPoints.push_back(cv::Point3f(0, 0, 0));
            objectPoints.push_back(cv::Point3f(1, 0, 0));  
            objectPoints.push_back(cv::Point3f(0, -1, 0));
            objectPoints.push_back(cv::Point3f(0, 0, 1));  

            objectPoints.push_back(cv::Point3f(5, 0, 0));
            objectPoints.push_back(cv::Point3f(6, 0, 0));  
            objectPoints.push_back(cv::Point3f(5, -1, 0));
            objectPoints.push_back(cv::Point3f(5, 0, 1));  

            objectPoints.push_back(cv::Point3f(0, -8, 0));
            objectPoints.push_back(cv::Point3f(1, -8, 0));  
            objectPoints.push_back(cv::Point3f(0, -9, 0));
            objectPoints.push_back(cv::Point3f(0, -8, 1));  

            objectPoints.push_back(cv::Point3f(5, -8, 0));
            objectPoints.push_back(cv::Point3f(6, -8, 0));  
            objectPoints.push_back(cv::Point3f(5, -9, 0));
            objectPoints.push_back(cv::Point3f(5, -8, 1));  

        
            projectPoints(objectPoints, rotation_vector, translation_vector, camera_matrix, distortionCoefficients, imagePoints);
            line(frame, imagePoints[0], imagePoints[1], Scalar(0, 255, 0), 2); 
            line(frame, imagePoints[0], imagePoints[2], Scalar(255, 0, 0), 2); 
            line(frame, imagePoints[0], imagePoints[3], Scalar(0, 0, 255), 2);

            line(frame, imagePoints[4], imagePoints[5], Scalar(0, 255, 0), 2); 
            line(frame, imagePoints[4], imagePoints[6], Scalar(255, 0, 0), 2); 
            line(frame, imagePoints[4], imagePoints[7], Scalar(0, 0, 255), 2);

            line(frame, imagePoints[8], imagePoints[9], Scalar(0, 255, 0), 2); 
            line(frame, imagePoints[8], imagePoints[10], Scalar(255, 0, 0), 2); 
            line(frame, imagePoints[8], imagePoints[11], Scalar(0, 0, 255), 2);

            line(frame, imagePoints[12], imagePoints[13], Scalar(0, 255, 0), 2); 
            line(frame, imagePoints[12], imagePoints[14], Scalar(255, 0, 0), 2); 
            line(frame, imagePoints[12], imagePoints[15], Scalar(0, 0, 255), 2);


        }
        else if(command == 2 || command == 3 || command == 4){
            // create_chair(frame, rotation_vector, translation_vector, camera_matrix, distortionCoefficients);
            create_car(frame, rotation_vector, translation_vector, camera_matrix, distortionCoefficients, colorVector, frameNum);
        }
        frameNum++;
        if(frameNum == 32000){
            frameNum = 0;
        }
        if(command == 4){
            imshow(windowName, frame); //display the image
            while(true){
                char ch = waitKey(0);
                if(ch == 'q'){
                    break;
                }
            }   
            break;
        }else{
            char ch = waitKey(30); 
            if(ch == 'q'){
                break;
            }
            imshow(windowName, frame);
        }
    }

    if(command != 4){
        video.release();
    }
    destroyAllWindows();
    printf("Terminating\n");
    return 0;
}

