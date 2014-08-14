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

using namespace std;

int main(int argc, char** argv) {
    aruco::MarkerDetector MDetector;
    vector<aruco::Marker> markers;
    cv::Point2f markerCenter;
    
    PointCloudCapture* cam = new PointCloudCapture();
    InputData src;
    
    pcl::PointXYZRGBA pos;

    cam->startCapture();
    cam->getFrame(src);

    do{
        
        cout << "capturing..." << endl;
        cam->getFrame(src);
        cout << "detecting..." << endl;
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
//                cout << "Four Corners: \n" << markers[i][0] << "\n"
//                                           << markers[i][1] << "\n"
//                                           << markers[i][2] << "\n"
//                                           << markers[i][3] << "\n";
//                cout << "Image Center: {" << markerCenter.x << "," << markerCenter.y << "}\n";
                cout << "{X,Y,Z} = {" << pos.x << "," << pos.y << "," << pos.z << "}\n";
            }
        }
    } while(true);
    
    cam->stopCapture();

    return 0;
}
