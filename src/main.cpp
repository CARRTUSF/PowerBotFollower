/* 
 * File:   main.cpp
 * Author: Andoni Aguirrezabal
 *
 * Created on August 6, 2014, 1:50 PM
 */

#include "aruco/aruco.h"
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/centroid.h>

#include "PointCloudCapture/PointCloudCapture.h"
#include "PowerBotClient.h"

using namespace std;

Eigen::Vector4f getCenter(int centerX, int centerY, int imgWidth, int imgHeight,
                          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr srcCloud) {
    int pixelWidth = 10;
    int minX = 0, maxX = 0, minY = 0, maxY = 0;
    
    pcl::PointXYZRGBA tempPos;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
    
    Eigen::Vector4f centroid;
    
    //minX
    if((centerX - pixelWidth) <= 0) {
        minX = 0;
    } else {
        minX = centerX - pixelWidth;
    }
    
    //maxX
    if((centerX + pixelWidth) >= imgWidth) {
        maxX = imgWidth;
    } else {
        maxX = centerX + pixelWidth;
    }
    
    //minY
    if((centerY - pixelWidth) <= 0) {
        minY = 0;
    } else {
        minY = centerY - pixelWidth;
    }
    
    //maxY
    if((centerY + pixelWidth) >= imgHeight) {
        maxY = imgHeight;
    } else {
        maxY = centerY + pixelWidth;
    }
    
//    cout << "Center {X, Y}: {" << centerX << ", " << centerY << "}\n";
//    cout << "{MinX, MaxX}: {" << minX << ", " << maxX << "}\n";
//    cout << "{MinY, MaxY}: {" << minY << ", " << maxY << "}\n";
    
    for(int i = minX; i <= maxX; i++) {
        for(int j = minY; j <= maxY; j++) {
            tempPos = srcCloud->points[j * srcCloud->width + i];
//            cout << "Checking: {" << tempPos.x << ", " << tempPos.y << ", "
//                     << tempPos.z << "}\n";
            if (std::isnan(tempPos.x) || std::isnan(tempPos.y)) {
                //PASS
            } else {
//                cout << "Adding: {" << tempPos.x << ", " << tempPos.y << ", "
//                     << tempPos.z << "}\n";
                tempCloud->points.push_back(tempPos);
            }
        }
    }
    
//    cout << "PC Size: " << tempCloud->size() << "\n";
    pcl::compute3DCentroid(*tempCloud, centroid);
    return centroid;
}

int main(int argc, char** argv) {
    aruco::MarkerDetector MDetector;
    vector<aruco::Marker> markers;
    cv::Point2f markerCenter;

    PointCloudCapture* cam = new PointCloudCapture();
    pcl::PointXYZ pos;
    InputData src;
    
    int imgWidth = 0;
    int imgHeight = 0;

    PowerBotClient pbClient;
    if(pbClient.connect()) {
        std::cout << "Connected to PowerBot!\n";
    } else {
        std::cout << "Could not connect to PowerBot!\n";
        return 0;
    }
    
    cam->startCapture();
    cam->getFrame(src);

    imgWidth = src.srcImg.size().width;
    imgHeight = src.srcImg.size().height;
    
    while(pbClient.getRunningWithLock()) {
        pbClient.requestUpdate();
        cam->getFrame(src);
        MDetector.detect(src.srcImg, markers);
        for (unsigned int i=0;i < markers.size();i++) {
            //cout << markers[i] << endl;
            markers[i].draw(src.srcImg, cv::Scalar(0,0,255), 2);
            if(markers[i].id == 1023) {
                markerCenter = markers[i].getCenter();
//                pcl::PointXYZRGBA pos = src.srcCloud->points[markerCenter.y 
//                                      * src.srcCloud->width + markerCenter.x];
                pos.getArray4fMap() = getCenter(markerCenter.x, markerCenter.y,
                                                imgWidth, imgHeight, src.srcCloud);
                //cout << "Four Corners: \n" << markers[i][0] << "\n"
                //                           << markers[i][1] << "\n"
                //                           << markers[i][2] << "\n"
                //                           << markers[i][3] << "\n";
                cout << "Image Center: {" << markerCenter.x << "," << markerCenter.y << "}\n";
                cout << "Marker Center (WORLD) {X,Y,Z} = {" << pos.x << "," << pos.y << "," << pos.z << "}\n";

                pos.x = (pos.x * (-1000));
                pos.z = (pos.z * 1000) - 500;
                if(pos.z < 0) { pos.z = 0; }
                
                if((abs(pos.x) >= 2500) || (abs(pos.z) >= 3500) ||
                    std::isnan(pos.x) || std::isnan(pos.z)) {
                    //PASS
                } else {
                    pbClient.transformPoints(pos.x, pos.z);
                    cout << "Desired PowerBot Coordinates {X,Y} = {" << pos.x << "," << pos.z << "}\n";
                    pbClient.moveTo(pos.x, pos.z);
                    usleep(1500000);
                }
            }
        }
    }

    cam->stopCapture();
    
    return 0;
}
