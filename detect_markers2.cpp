#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <geometry_msgs/Vector3.h>
#include <iostream>

/*
rosrun robot detect_markers2 -c="/home/blingshock/openCVtutorials/intrinsic.yml" -d=5 -ci=1 -l=.196
*/
using namespace std;
using namespace cv;

namespace {
const char* about = "Basic marker detection";
const char* keys  =
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{r        |       | show rejected candidates too }"
        "{refine   |       | Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1,"
        "CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}";
}

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}



/**
 */
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "detect_marker2");
    ros::NodeHandle n;
    ros::Publisher pubR= n.advertise<geometry_msgs::Vector3>("/rvec2",10);
    ros::Publisher pubT= n.advertise<geometry_msgs::Vector3>("/tvec2",10);
    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        geometry_msgs::Vector3 msgRot;
        geometry_msgs::Vector3 msgTran;

        CommandLineParser parser(argc, argv, keys);
        parser.about(about);

        cout << "argc = " << argc << endl;
        if(argc < 2) {
            parser.printMessage();
            return 0;
        }
        int dictionaryId = parser.get<int>("d");
        bool showRejected = parser.has("r");
        bool estimatePose = parser.has("c");
        float markerLength = parser.get<float>("l");

        Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
        if(parser.has("dp")) {
            bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
            if(!readOk) {
                cerr << "Invalid detector parameters file" << endl;
                return 0;
            }
        }

        if (parser.has("refine")) {
            //override cornerRefinementMethod read from config file
            detectorParams->cornerRefinementMethod = parser.get<int>("refine");
        }
        std::cout << "Corner refinement method (0: None, 1: Subpixel, 2:contour, 3: AprilTag 2): " << detectorParams->cornerRefinementMethod << std::endl;

        int camId = parser.get<int>("ci");

        String video;
        if(parser.has("v")) {
            video = parser.get<String>("v");
        }

        if(!parser.check()) {
            parser.printErrors();
            return 0;
        }

        Ptr<aruco::Dictionary> dictionary =
            aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

        Mat camMatrix, distCoeffs;
        if(estimatePose) {
            bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
            if(!readOk) {
                cerr << "Invalid camera file" << endl;
                return 0;
            }
        }

        VideoCapture inputVideo;
        int waitTime;
        if(!video.empty()) {
            inputVideo.open(video);
            waitTime = 0;
        } else {
            inputVideo.open(camId);
            waitTime = 10;
        }

        double totalTime = 0;
        int totalIterations = 0;

        while(inputVideo.grab()) {
            Mat image, imageCopy;
            inputVideo.retrieve(image);

            double tick = (double)getTickCount();

            vector< int > ids;
            vector< vector< Point2f > > corners, rejected;
            vector< Vec3d > rvecs, tvecs;

            // detect markers and estimate pose
            aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
            if(estimatePose && ids.size() > 0)
                aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,
                                                 tvecs);
            double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
            totalTime += currentTime;
            totalIterations++;
            if(totalIterations % 30 == 0) {
                cout << "Detection Time = " << currentTime * 1000 << " ms "
                     << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
            }

            // draw results
            image.copyTo(imageCopy);
            if(ids.size() > 0) {
                aruco::drawDetectedMarkers(imageCopy, corners, ids);

                msgRot.x = rvecs[0][0];
                msgRot.y = rvecs[0][1];
                msgRot.z = rvecs[0][2];
                msgTran.x = tvecs[0][0];
                msgTran.y = tvecs[0][1];
                msgTran.z = tvecs[0][2];
                cout << "FDSLKDFAJFDAJK" << endl;

                pubR.publish(msgRot);
                pubT.publish(msgTran);

                if(estimatePose) {
                    for(unsigned int i = 0; i < ids.size(); i++)
                    {
                        aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                                        markerLength * 0.5f);
                        cout << ids[i] << endl;

                    }
                    cout << endl << endl;

                }
            }

            if(showRejected && rejected.size() > 0)
                aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

            imshow("out", imageCopy);
            char key = (char)waitKey(waitTime);
            if(key == 27) break;
        }
    }

    return 0;
}
