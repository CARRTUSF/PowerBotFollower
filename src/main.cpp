/* 
 * File:   main.cpp
 * Author: Andoni Aguirrezabal
 *
 * Created on August 6, 2014, 1:50 PM
 */

#include "aruco/aruco.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

int main(int argc, char** argv) {
    VideoCapture camCap(0);

    aruco::MarkerDetector MDetector;
    vector<aruco::Marker> markers;
    cv::Point2f markerCenter;

    if(!camCap.isOpened()) {
        cout << "Cannot open the web camera" << endl;
        return -1;
    }

    double imgWidth = camCap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double imgHeight = camCap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    while(true) {
        cv::Mat newFrame;

        if(!camCap.read(newFrame)) {
            //PASS
        } else {
            
        }
    }

    return 0;
}
