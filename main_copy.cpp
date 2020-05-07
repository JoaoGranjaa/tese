#include "mainwindow.h"
#include <QApplication>

#include "aruco.h"
#include <iostream>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char **argv)
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    //if (argc != 2 ){ std::cerr<<"Usage: inimage"<<std::endl;return -1;}

    Mat image= imread("/home/joao/Desktop/ArUco/m5.jpg");
    aruco::MarkerDetector MDetector;
    MDetector.setDictionary("ARUCO_MIP_36h12");

    //detect
    std::vector<aruco::Marker> markers=MDetector.detect(image);

    std::cout << "Hey", markers.size();

    //print info to console
    for(size_t i=0;i<markers.size();i++){
       std::cout<<markers[i]<<std::endl;
      //draw in the image
       markers[i].draw(image);
    }

    imshow("image",image);
    waitKey(0);


    return 0;
}
