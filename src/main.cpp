/* 
 * File:   main.cpp
 * Author: Andoni Aguirrezabal
 *
 * Created on August 6, 2014, 1:50 PM
 */

#include "aruco/aruco.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "PointCloudCapture/PointCloudCapture.h"
#include "pcl/visualization/pcl_visualizer.h"

#include "PowerBotClient.h"

using namespace std;

int main(int argc, char** argv) {
    aruco::MarkerDetector MDetector;
    vector<aruco::Marker> markers;
    cv::Point2f markerCenter;

    PointCloudCapture* cam = new PointCloudCapture();
    InputData src;
    
    pcl::PointXYZRGBA pos;

    PowerBotClient pbClient;
    if(pbClient.connect()) {
        std::cout << "Connected to PowerBot!\n";
    } else {
        std::cout << "Could not connect to PowerBot!\n";
        return 0;
    }
    
    cam->startCapture();
    cam->getFrame(src);

    while(pbClient.getRunningWithLock()) {
        pbClient.requestUpdate();
        cam->getFrame(src);
        MDetector.detect(src.srcImg, markers);
        for (unsigned int i=0;i < markers.size();i++) {
            //cout << markers[i] << endl;
            markers[i].draw(src.srcImg, cv::Scalar(0,0,255), 2);
            if(markers[i].id == 213) {
                markerCenter = markers[i].getCenter();
                int centerX = markerCenter.x;
                int centerY = markerCenter.y;
                pcl::PointXYZRGBA pos = src.srcCloud->points[markerCenter.y 
                                      * src.srcCloud->width + markerCenter.x];
                //cout << "Four Corners: \n" << markers[i][0] << "\n"
                //                           << markers[i][1] << "\n"
                //                           << markers[i][2] << "\n"
                //                           << markers[i][3] << "\n";
                //cout << "Image Center: {" << markerCenter.x << "," << markerCenter.y << "}\n";
                cout << "Marker Center {X,Y,Z} = {" << pos.x << "," << pos.y << "," << pos.z << "}\n";
                pos.x = (pos.x * (-1000));
                pos.z = (pos.z * 1000);
                if(pos.z >= 3000.0) {
                    pos.z = 1000;
                }
                pbClient.transformPoints(pos.x, pos.z);
                cout << "Map Coordinates {X,Y} = {" << pos.x << "," << pos.z << "}\n";
                //PowerBot Navigation Code will go here
            }
        }
    }

    cam->stopCapture();
    
    return 0;
}

void findCenter(double &outX, double &outY, double &outZ) {
    
}