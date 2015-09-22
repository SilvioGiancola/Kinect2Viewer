/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


#include <iostream>
#include <signal.h>



#include <pcl/visualization/pcl_visualizer.h>


#include <QFile>
#include <QTextStream>
#include <QTime>
#include <mykinect.h>

using namespace std;

MyKinect kin;
bool play = false;


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event)
{
    if (event.keyDown())
    {
        cout << "KeyboardEvent: " << event.getKeySym ()  << endl;

        if (event.getKeySym () == "h" )
        {
            cout << "HELP - Press:" << endl;
            cout << "   - o for Opening Kinect Connection" << endl;
            cout << "   - c for Closing Kinect Connection" << endl;
            cout << "   - p for Play/Pause Kinect Connection" << endl;
            cout << "   - h for Help" << endl;
        }

        if (event.getKeySym () == "o" )
            kin.Open()  ;

        if (event.getKeySym () == "c" )
            kin.Close();

        if (event.getKeySym () == "p" )
            play = !play;



    }
    return;
}

int main(int argc, char *argv[])
{

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->registerKeyboardCallback (keyboardEventOccurred);
    viewer->setBackgroundColor (0.5, 0.5, 0.5);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();



    PointCloudT::Ptr KinectPointCloud(new PointCloudT);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> KinectColor(KinectPointCloud);
    viewer->addPointCloud(KinectPointCloud, KinectColor, "Kinect2");

    // viewer->addText("",50,50,50,255,255,255,"AcqTime");
    // viewer->addText("",50,50,50,255,255,255,"FPS");

    while (!viewer->wasStopped ())
    {

        if (kin._open)
        {
            if (play)
                KinectPointCloud = kin.Grab();
        }
        else
        {
            KinectPointCloud.reset(new PointCloudT());
        }

        pcl::visualization::PointCloudColorHandlerRGBField<PointT> KinectColor(KinectPointCloud);
        viewer->updatePointCloud(KinectPointCloud,KinectColor,"Kinect2");



        // cout << QString("Acquisition Time = %1 ms").arg(t.elapsed()).toStdString() << endl;
        //  viewer->updateText(QString("Acquisition Time = %1 ms").arg(t.elapsed()).toStdString(),50,50,20,255,255,255,"AcqTime");
        //  viewer->updateText(QString("FPS = %1 Hz").arg(1000.0f/t.elapsed()).toStdString(),50,50,20,255,255,255,"FPS");

        viewer->spinOnce (1);



    }



    /*dev->stop();
    dev->close();

    delete registration;
*/



    return 0;
}
