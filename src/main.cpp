/* 
 * File:   main.cpp
 * Author: Andoni Aguirrezabal
 *
 * Created on August 6, 2014, 1:50 PM
 */

#include "aruco/aruco.h"
#include "SockStream.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

int main(int argc, char** argv) {
    cv::VideoCapture camCap(1);

    aruco::MarkerDetector MDetector;
    vector<aruco::Marker> markers;
    cv::Point2f markerCenter;
    cv::namedWindow("CAMERA",CV_WINDOW_AUTOSIZE);
    
    sending_udpsocket clientSocket("192.168.0.104:3251");
    sockstream networkOut(clientSocket);
    receiving_udpsocket serverSocket("0.0.0.0:3250");
    sockstream networkIn(serverSocket);
    string inputBuffer;
    
    if(!camCap.isOpened()) {
        cout << "Cannot open the web camera" << endl;
        return -1;
    }

    if(!networkOut.is_open()) {
        cout << "Could not open the socket for sending" << endl;
        return -1;
    }

    double imgWidth = camCap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double imgHeight = camCap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
    
    while(true) {
        cv::Mat newFrame;
        getline(networkIn, inputBuffer);

        if (inputBuffer.compare("REQ") == 0) {
            if(!camCap.read(newFrame)) {
                cout << "ERROR: Frame Dropped" << endl;
                networkOut << "NOM" << endl;
            } else {
                cv::imshow("CAMERA", newFrame);
                MDetector.detect(newFrame, markers);

                if(markers.size() > 0) {
                    double xRel = 0;
                    
                    markers[0].draw(newFrame,cv::Scalar(0,0,255),2);
                    
                    cv::Point2f centerPosition = markers[0].getCenter();
                    xRel = (double)((centerPosition.x - (imgWidth/2.0))
                                    / ((double)imgWidth));

                    cout << markers[0] << endl;
                    networkOut << xRel << endl;
                } else {
                    cout << "No Marker!" << endl;
                    networkOut << "NOM" << endl;
                }
                cv::waitKey(2);
            }
        }
    }

    return 0;
}
