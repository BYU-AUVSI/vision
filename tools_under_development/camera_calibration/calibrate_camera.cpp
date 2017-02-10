#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>


int main()
{
    //Define number of calibration images
    int num_images = 40;

    //Store imageSize
    cv::Size imageSize;

    //Define board Size(width x height) of inner chessboard corners
    cv::Size board_size = cv::Size(9,7);

    //Define arbitrary Chessboard square width (unitless)
    int square_width = 100;

    //Define image objects
    cv::Mat image;
    cv::Mat image_resized;
    cv::Mat image_gray;
    cv::Mat image_color;

    //Create imagePoints and objectPoints vectors
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;

    //Populate ObjPts vector for images (assumes calibration chessboard doesn't change and all corners visible)
    // *ObjPts for each image are identical
    std::vector<cv::Point3f> ObjPts;

    for(int y=0;y<board_size.height;y++)
    {
        for(int x=0;x<board_size.width;x++)
        {
            ObjPts.push_back(cv::Point3f(x*square_width,y*square_width,0));
        }
    }


    for (int i=0;i<num_images;i++)
    {
        //Generate file path to the image (string)
        int image_number = i+1;
        std::string number = std::to_string(image_number);
        std::string path_begin = "/home/jesse/Desktop/ECEN_631/Assignment_2/my_images/guvcview_image-"; //change to path of the calibration images you are using
        std::string path_end = ".jpg";
        std::string file = path_begin + number + path_end;

        //Vector to store corners
        std::vector<cv::Point2f> corners;

        //Read the image
        image = cv::imread(file, 1);
        cv::resize(image,image_resized,cv::Size(640,480),0,0,CV_INTER_LINEAR);  //(768x494)
        image_color = image_resized;

        //Get the image size
        imageSize = cv::Size(image_resized.cols, image_resized.rows);

        //std::cout << image_resized.cols << " " << image_resized.rows << std::endl;

        //Convert_to_Grayscale
        cv::cvtColor(image_resized, image_gray, 6); //convert to grayscale (COLOR_BGR2GRAY == 6)

        //Find Chessboard corners
        bool corners_found = cv::findChessboardCorners(image_gray,board_size,corners,1 + 2);

        if(corners_found)
        {
            //Refine corner locations
            cv::cornerSubPix(image_gray, corners, cv::Size(11, 11), cv::Size(-1, -1),cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

            //Draw on the colored image so it's pretty
            cv::drawChessboardCorners(image_color, board_size, corners, corners_found);

            if(corners.size() == board_size.width*board_size.height)
            {
                //Initialize vector to hold ImagePoints for each calibration image
                std::vector<cv::Point2f> ImgPts;

                //Populate the ImgPts and ObjPts Vectors
                for(int i=0;i<corners.size();i++)
                {
                    cv::Point2f ImgPt;
                    ImgPt.x = corners[i].x;
                    ImgPt.y = corners[i].y;

                    //Push each ImgPt onto the ImgPts vector
                    ImgPts.push_back(ImgPt);
                }
                //Push the ImagePts vector onto the imagePoints vector
                imagePoints.push_back(ImgPts);
            }
            else
            {
                std::cout << "Board size and corner size mismatch (all corners may not have been detected).";
            }
            //Push the ObjPts vector onto the objectPoints vector
            objectPoints.push_back(ObjPts);
        }
        else
        {
            std::cout << "Corners not found on image " << i << std::endl;
        }


        //show the image
        cv::imshow("Chess Board", image_color);
        cv::waitKey(500);

    }
    //ObjectPoints and ImagePoints gathered, time to calibrate
    //initialize rvecs and tvecs
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    //initialize Intrinsic_Matrix and Distortion_Coefficients
    cv::Mat Intrinsic_Matrix(3,3, CV_64F);
    cv::Mat Distortion_Coefficients(5,1,CV_64F);


    //calibrate using calibrateCamera function
    cv::calibrateCamera(objectPoints,imagePoints,imageSize,Intrinsic_Matrix,Distortion_Coefficients,rvecs,tvecs);

    //Write Calibration Intrinsic and Distortion parameters to .txt file

    std::string filename = "camera_calibration_parameters.txt";
    std::FILE* file;
    file = std::fopen(filename.c_str(),"w");
    std::fprintf(file,"Camera Calibration File\n\n");
    std::fprintf(file,"Camera Intrinsic Parameters:\n\n");
    for(int i=0;i<Intrinsic_Matrix.rows;i++)
    {
        for(int j=0;j<Intrinsic_Matrix.cols;j++)
        {
            std::fprintf(file,"%lf ", Intrinsic_Matrix.at<double>(i,j));
        }
        std::fprintf(file,"\n");
    }
    std::fprintf(file,"\n");
    std::fprintf(file,"Camera Distortion Coefficients:\n\n");
    for(int i=0;i<Distortion_Coefficients.rows;i++)
    {
        std::fprintf(file,"%lf ", Distortion_Coefficients.at<double>(i));
        std::fprintf(file,"\n");
    }
    std::fclose(file);

    //Write Calibration Intrinsic and Distortion parameters to .xml file (this is the preferred method)
    cv::FileStorage fs("camera_calibration_parameters.xml", cv::FileStorage::WRITE);
    fs << "Camera_Matrix" << Intrinsic_Matrix;
    fs << "Distortion_Coefficients" << Distortion_Coefficients;

    //If we were to read this file it would look like:
    //cv::FileStorage fs("camera_calibration_parameters.xml", cv::FileStorage::READ);
    //fs["Camera_Matrix"] >> Intrinsic_Matrix;
    //fs["Distortion_Coefficients"] >> Distortion_Coefficients;


    return 0;
}
