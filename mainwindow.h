#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;

#include <iostream>
#include <sstream>
#include <fstream>
#include <QFile>
#include <vector>
using namespace cv;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    struct group {
        vector<int> ids;
        vector<float> orientationId;
        vector<float> distanceId;
    };

private slots:
    void on_openCamera_clicked();

    void on_closeCamera_clicked();

    void update_window(Mat frame);

    void openCameraFunction();

    bool loadCameraCalibration(string name, Mat& cameraMatrix, Mat& distortionCoefficients);

    int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distortionCoefficients);

    void updateAUV(vector<group> &AUV, int markerId, float orientationAngle, float markerDistance);

    float getOrientationAngle(Mat frame, float side, Vec3d rvec, Vec3d tvec, const Mat& cameraMatrix, const Mat& distCoeffs);

    vector<int> findIndexInAUV(vector<group> &AUV, int markerId);

    vector<Point3f> getCornersInCameraWorld(float side, Vec3d rvec, Vec3d tvec);

    vector<float> checkDifferentMarkers(vector<int> markerIds);

    void printAUV(vector<group> &AUV);

    Mat eulerAnglesToRotationMatrix(Vec3d &theta);

    Vec3f rotationMatrixToEulerAngles(Mat &R);

    bool isRotationMatrix(Mat &R);

    void on_extractSamples_clicked();

    void on_stopSamples_clicked();

    void writeInFileDistances(int nFrames, vector<int> markerIds, vector<float> markerDistances, int realDistance, int realAngle, int realStepAngle);

    QVector<float> distancesToWrite(int nFrames, vector<int> markerIds, vector<float> markerDistances, int realDistance);

    void on_increaseDistance_clicked();

private:
    Ui::MainWindow *ui;

    QTimer *timer;
    VideoCapture cap;

    Mat frame;
    QImage qt_image;

    Mat cameraMatrix = Mat::eye(3,3, CV_64F);

    Mat distortionCoefficients;

    bool recordFrames = false;

    int framesToRecord = 50;

    int frames = framesToRecord;
    int realDistance = 1000;
    int realStepDistance = 250;
    int realAngle = -75;
    int realStepAngle = 5;

    QVector<float> nMarkersDetected{0,0,0,0};
    QVector<float> distAvg{0,0,0,0};
    QVector<float> errorAvg{0,0,0,0};
    QVector<float> errorMax{0,0,0,0};
    QVector<float> errorAvgPerc{0,0,0,0};
    QVector<float> errorMaxPerc{0,0,0,0};
    //cv::Ptr<cv::aruco::Dictionary> dictionary;
    //cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5, 7, 0.04, 0.01, dictionary);
};

#endif // MAINWINDOW_H
