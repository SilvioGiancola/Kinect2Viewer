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
#include <pcl/io/pcd_io.h>


#include <QFile>
#include <QTextStream>
#include <QTime>
#include <mykinect.h>

using namespace std;

MyKinect kin;
bool play = false;
bool save = false;
uint XStep = 150;
uint Size = 30;


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

    if (event.keyDown())
    {
        cout << "KeyboardEvent: " << event.getKeySym ()  << endl;

 /*       if (event.getKeySym () == "h" )
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

*/

    }
    return;
}


void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

    if (event.getButton () == pcl::visualization::MouseEvent::LeftButton && event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
        std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

        if (event.getY() < Size+10)
        {
            if (event.getX() < XStep*1)
            {
                if (kin._open)
                {
                    if (kin.Close() == SUCCESS)
                        viewer->updateText("OPEN",   XStep*0, 10, Size, 1, 1, 1, "OPEN/CLOSE");
                }
                else
                {
                    if (kin.Open() == SUCCESS)
                        viewer->updateText("CLOSE",  XStep*0, 10, Size, 1, 1, 1, "OPEN/CLOSE");
                }
            }


            else if (event.getX() < XStep*2)
            {
                play = !play;
                if (play)
                    viewer->updateText("PAUSE",  XStep*1, 10, Size, 1, 1, 1, "PLAY/PAUSE");

                else
                    viewer->updateText("PLAY",   XStep*1, 10, Size, 1, 1, 1, "PLAY/PAUSE");
            }


            else if (event.getX() < XStep*3)
            {
                save = !save;
                if (save)
                    viewer->updateText("SAVE",  XStep*2, 10, Size, 1, 1, 1, "SAVE");

                else
                    viewer->updateText("SAVE",  XStep*2, 10, Size, 0.3, 0.3, 0.3, "SAVE");
            }
        }
    }
}

#include <QTime>

int main(int argc, char *argv[])
{    
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0.5, 0.5, 0.5);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setCameraPosition(0.5,0.5,-2.5, // mi posiziono dietro ad un Kinect
                              0.5,0.5,0.5, // guardo un punto centrale
                              0,1,0);   // orientato con la y verso l'alto

    viewer->addText("OPEN",  XStep*0, 10, Size, 1, 1, 1, "OPEN/CLOSE");
    viewer->addText("PLAY",  XStep*1, 10, Size, 1, 1, 1, "PLAY/PAUSE");
    viewer->addText("SAVE",  XStep*2, 10, Size, 0.3, 0.3, 0.3, "SAVE");
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);


    PointCloudT::Ptr KinectPointCloud(new PointCloudT);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> KinectColor(KinectPointCloud);
    viewer->addPointCloud(KinectPointCloud, KinectColor, "Kinect2");

   //  viewer->addText("",XStep,0,10,1,1,1,"AcqTime");
     viewer->addText("FPS",XStep,0,10,1,1,1,"FPS");

     QTime t;
     t.start();

    while (!viewer->wasStopped ())
    {
        t.restart();
        if (kin._open)
        {
            if (play)
            {
                KinectPointCloud = kin.Grab();

                if (save)
                {
                    pcl::io::savePCDFileBinary(KinectPointCloud->header.frame_id, *KinectPointCloud);
                }
                viewer->updateText(QString("FPS = %1 Hz").arg(1000.0f/t.elapsed()).toStdString(),XStep,0,10,1,1,1,"FPS");
            }
        }
        else
            KinectPointCloud.reset(new PointCloudT());


        pcl::visualization::PointCloudColorHandlerRGBField<PointT> KinectColor(KinectPointCloud);
        viewer->updatePointCloud(KinectPointCloud,KinectColor,"Kinect2");



        viewer->spinOnce (1);
        // cout << QString("Acquisition Time = %1 ms").arg(t.elapsed()).toStdString() << endl;
        //  viewer->updateText(QString("Acquisition Time = %1 ms").arg(t.elapsed()).toStdString(),50,50,20,1,1,1,"AcqTime");

    }

    return 0;
}
