#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>

void tokenize(const std::string& str,
              std::vector<std::string>& tokens,
              const std::string& delimiters);

int main()
{
    //Get the path to the test image files
    std::string image_1 = "/home/jesse/Desktop/ECEN_631/Assignment_2/my_images/guvcview_image-1.jpg";
    std::string image_2 = "/home/jesse/Desktop/ECEN_631/Assignment_2/my_images/guvcview_image-26.jpg";
    std::string image_3 = "/home/jesse/Desktop/ECEN_631/Assignment_2/my_images/guvcview_image-11.jpg";

    //Get the path to the camera calibration file
    std::string file_path = "/home/jesse/vision/tools_under_development/build-camera_calibration-Desktop_Qt_5_7_0_GCC_64bit-Debug/camera_calibration_parameters.txt";
    std::string line;

    //define a vector of vectors to hold all of the data (tokens) in the file
    std::vector<std::vector<std::string>> tokens;
    std::string delimiters(", ;\t");

    //Build the vector of token vectors
    std::ifstream infile(file_path.c_str());
    while (std::getline(infile, line, '\n'))
    {
        if(line == "") continue;            //skip blank lines
        std::vector<std::string> temp;      //temp string vector to pass tokenize
        tokenize(line, temp, delimiters);
        tokens.push_back(temp);             //pushback the new line of tokens back onto the tokens vector
    }

    //Read the Camera Calibration Data

    //Read Camera Intrinsic Parameters
    double fx = std::stod(tokens[2][0]);
    double fy = std::stod(tokens[3][1]);
    double cx = std::stod(tokens[2][2]);
    double cy = std::stod(tokens[3][2]);

    //Read Camera Distortion Parameters
    double k1 = std::stod(tokens[6][0]);
    double k2 = std::stod(tokens[7][0]);
    double p1 = std::stod(tokens[8][0]);
    double p2 = std::stod(tokens[9][0]);
    double k3 = std::stod(tokens[10][0]);

    //Set up the Camera Intrinsic Matrix
    //double Intrinsic_Array [9] = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
    cv::Mat Camera_Matrix(3, 3, CV_64F);
    Camera_Matrix = cv::Mat::eye(3, 3, CV_64F);
    Camera_Matrix.at<double>(0,0) = fx;
    Camera_Matrix.at<double>(0,2) = cx;
    Camera_Matrix.at<double>(1,1) = fy;
    Camera_Matrix.at<double>(1,2) = cy;

    //Set up the Camera Distortion Vector
    //double Distortion_Array [5] = {k1, k2, p1, p2, k3};
    cv::Mat Distortion_Vector(5, 1, CV_64F);
    Distortion_Vector = cv::Mat::zeros(5, 1, CV_64F);
    Distortion_Vector.at<double>(0,0) = k1;
    Distortion_Vector.at<double>(1,0) = k2;
    Distortion_Vector.at<double>(2,0) = p1;
    Distortion_Vector.at<double>(3,0) = p2;
    Distortion_Vector.at<double>(4,0) = k3;

    //Define new Camera Matrix
    //cv::Mat New_Intrinsic_Matrix(3,3, CV_64F);

    //Read in the images and convert to grayscale

    cv::Mat First;
    cv::Mat First_resized;
    cv::Mat First_gray;
    cv::Mat Second;
    cv::Mat Second_resized;
    cv::Mat Second_gray;
    cv::Mat Third;
    cv::Mat Third_resized;
    cv::Mat Third_gray;

    First = cv::imread(image_1,1);
    cv::resize(First,First_resized,cv::Size(640,480),0,0,CV_INTER_LINEAR);
    cv::cvtColor(First_resized,First_gray,6);

    Second = cv::imread(image_2,1);
    cv::resize(Second,Second_resized,cv::Size(640,480),0,0,CV_INTER_LINEAR);
    cv::cvtColor(Second_resized,Second_gray,6);

    Third = cv::imread(image_3,1);
    cv::resize(Third,Third_resized,cv::Size(640,480),0,0,CV_INTER_LINEAR);
    cv::cvtColor(Third_resized,Third_gray,6);

    //Undistort the images
    cv::Mat First_undist;
    cv::Mat Second_undist;
    cv::Mat Third_undist;

    cv::undistort(First_gray,First_undist,Camera_Matrix,Distortion_Vector,cv::noArray());\
    cv::undistort(Second_gray,Second_undist,Camera_Matrix,Distortion_Vector,cv::noArray());
    cv::undistort(Third_gray,Third_undist,Camera_Matrix,Distortion_Vector,cv::noArray());

    //Difference the original vs the undistorted image
    cv::Mat First_diff;
    cv::Mat Second_diff;
    cv::Mat Third_diff;

    cv::absdiff(First_gray,First_undist,First_diff);
    cv::absdiff(Second_gray,Second_undist,Second_diff);
    cv::absdiff(Third_gray,Third_undist,Third_diff);

    //Display the images
    cv::imshow("First Image Difference",First_diff);
    cv::imshow("Second Image Difference",Second_diff);
    cv::imshow("Third Image Difference",Third_diff);

    //cv::imwrite("/home/jesse/Desktop/ECEN_631/Assignment_2/task_6/first_diff.jpg",First_diff);

    cv::waitKey(0);

    return 0;
}

//Definition tokenize function
void tokenize(const std::string& str,
              std::vector<std::string>& tokens,
              const std::string& delimiters = " ")
{
    // Skip delimiters at beginning.
    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);

    // Find first "non-delimiter".
    std::string::size_type pos     = str.find_first_of(delimiters, lastPos);

    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));

        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);

        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }

}
