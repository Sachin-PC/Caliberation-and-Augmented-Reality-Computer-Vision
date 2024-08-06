/*
    Sachin Palahalli Chandrakumar
    Spring 2024
    Code to find harris features and detect corners
*/

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]){


    VideoCapture video("/Users/sachinpc/Documents/Northeastern University/Semesters/Fourth Semester/PRCV/Projects/4/utils/videos/9.MOV");
    // std::string imgFileName = "/Users/sachinpc/Documents/Northeastern University/Semesters/Fourth Semester/PRCV/Projects/4/utils/photos/checkerboard.png";

    if (!video.isOpened()) {
        cout<< "Cannot read the video file" << endl;
        return 0;
    }

    char windowName[256] = "projection";
    namedWindow(windowName, WINDOW_NORMAL);
    cv::Mat frame, greyScale, dst;
    
    while(true){
        video.read(frame);
        if (frame.empty()) {
            cout<<"Frame empty"<<endl;
            break;
        }
        cvtColor(frame,greyScale, cv::COLOR_BGR2GRAY);
        cornerHarris(greyScale,dst,2,3,0.04);
        normalize( dst, dst, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
        for(int i=0;i <dst.rows; i++){
            for(int j=0; j<dst.cols; j++){
                if(dst.at<float>(i,j) > 150){
                    circle(frame,Point(j,i),8,Scalar(255,255),2);
                }
            }
        }

       char ch = waitKey(30); 
        if(ch == 'q'){
            break;
        }
        imshow(windowName, frame);
    }

    video.release();
    destroyAllWindows();
    printf("Terminating\n");
    return 0;
}