/*
    Sachin Palahalli Chandrakumar
    Spring 2024
    Code to implement augmented reality and caliberation
*/

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>

using namespace std;
using namespace cv;

// Defining the prototypes of methods used in this file
int read_image_data_csv( char *filename, std::vector<char *> &filenames, std::vector<std::vector<float>> &data, int echo_file );
int append_image_data_csv( char *filename, char *image_filename, std::vector<double> &image_data, int reset_file );

int main(int argc, char *argv[]){

    VideoCapture video("/Users/sachinpc/Documents/Northeastern University/Semesters/Fourth Semester/PRCV/Projects/4/utils/videos/9.MOV");

    // std::string imgFileName = "/Users/sachinpc/Documents/Northeastern University/Semesters/Fourth Semester/PRCV/Projects/4/utils/photos/10.png";
    if (!video.isOpened()) {
        cout<< "Cannot read the video file" << endl;
        return 0;
    }

    char windowName[256] = "augmentedReality";
    namedWindow(windowName, WINDOW_NORMAL);
    cv::Mat frame, greyScale;
    bool patternfound = false;
    Size patternsize(6,9);
	vector<vector<Vec3f> > point_list;
	vector<vector<Point2f> > corner_list;
    string outputImageName = "frame";
    string outputImageDir =  "/Users/sachinpc/Documents/Northeastern University/Semesters/Fourth Semester/PRCV/Projects/4/utils/saved_frames/";
    string outputImagePath;
    int fnum=0;
    bool quit = false;
    Mat camera_matrix(3, 3, CV_64FC1);
    Mat distortionCoefficients(2, 1, CV_64FC1, Scalar(0));
    //Initial caliberation matrix
    camera_matrix.at<double>(0,0) = 1;
    camera_matrix.at<double>(0,1) = 0;
    camera_matrix.at<double>(0,2) = frame.cols/2;
    camera_matrix.at<double>(1,0) = 0;
    camera_matrix.at<double>(1,1) = 1;
    camera_matrix.at<double>(1,2) = frame.rows/2;
    camera_matrix.at<double>(2,0) = 0;
    camera_matrix.at<double>(2,1) = 0;
    camera_matrix.at<double>(2,2) = 1;
    
    cout<<"Initial camera matrix = "<<camera_matrix<<endl;
    double reprojection_error;

    while (true) {
        //read frame
        video.read(frame);
        // cout<<"frame.cols/2; = "<<frame.cols/2<<endl;
        // cout<<"frame.rows/2; = "<<frame.rows/2<<endl;
        if (frame.empty()) {
            cout<<"Frame empty"<<endl;
            break;
        }

        char ch = waitKey(30); 
        switch(ch){
            case 'q': 
                quit = true;
                cout<<"EXIt"<<endl;
                break;
            case 's':
                vector<Point2f> corner_set;
                vector<Vec3f> point_set;
                //get greayscale image
                cvtColor(frame,greyScale, COLOR_BGR2GRAY);
                //find pattern
                patternfound = cv::findChessboardCorners(greyScale, patternsize, corner_set);
                cout<<"patternfound = "<<patternfound<<endl;
                if(patternfound){
                    cornerSubPix(greyScale, corner_set, Size(11, 11), Size(-1, -1),
                    TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                    //draw chessboard corners found
                    drawChessboardCorners(frame, patternsize, Mat(corner_set), patternfound);
                    cout<<"Number of Corners for this Frame = "<<corner_set.size()<<endl;
                    corner_list.push_back(corner_set);
                    for (int i = 0; i < patternsize.height; ++i) {
                        for (int j = 0; j < patternsize.width; ++j) {
                            point_set.push_back(cv::Vec3f(j, -i, 0));
                        }
                    }
                    point_list.push_back(point_set);
                }
                fnum += 1;
                outputImagePath = outputImageDir + outputImageName + std::to_string(fnum)+".jpg";
                imwrite(outputImagePath, frame);
                cout<<"Frame saved succesfully!"<<endl;
                break;
        }
        if(quit){
            break;
        }
        if(fnum >= 5 && ch == 's'){
            //once desired frames are obtained, caliberate camera
            vector<Mat> rotation_vectors, translation_vectors;
            reprojection_error = cv::calibrateCamera(point_list, corner_list, greyScale.size(), camera_matrix, distortionCoefficients, rotation_vectors, translation_vectors);

            cout<<"camera_matrix values:\n"<<camera_matrix<<endl;
            cout<<"distortionCoefficients values:\n"<<distortionCoefficients<<endl;
            cout<<"reprojection_error = "<<reprojection_error <<endl;
        }

        //diaply image
        imshow(windowName, frame);
    }


    cout<<"Finale camera_matrix values:\n"<<camera_matrix<<endl;
    cout<<"Final distortionCoefficients values:\n"<<distortionCoefficients<<endl;
    cout<<"Final reprojection_error = "<<reprojection_error <<endl;

    string featuresFileName = "/Users/sachinpc/Documents/Northeastern University/Semesters/Fourth Semester/PRCV/Projects/4/utils/caliberationData/caliberationDataFile.csv";
    string objectLabel = "camera_matrix";
    vector<double> caliberationMatrixValues;
    vector<double> distortion_coefficients;
    int k=0;
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            caliberationMatrixValues.push_back(camera_matrix.at<double>(i,j));
            k++;
        }
    }

    for(int i=0; i<distortionCoefficients.rows;i++){
        distortion_coefficients.push_back(distortionCoefficients.at<double>(i,0));
    }

    int reset_file = 1;
    //store caliberation matrix and distortion coefficients to csv file
    append_image_data_csv(&featuresFileName[0], &objectLabel[0],  caliberationMatrixValues, reset_file);
    objectLabel = "distortion_coefficients";
    reset_file = 0;
    append_image_data_csv(&featuresFileName[0], &objectLabel[0],  distortion_coefficients, reset_file);
    video.release();
    destroyAllWindows();
    cout<<"Terminating"<<endl;
}