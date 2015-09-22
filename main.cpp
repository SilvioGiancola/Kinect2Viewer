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

#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <Eigen/Dense>
#include <pcl/visualization/pcl_visualizer.h>

//#include <pcl/surface/mls.h>
//#include <pcl/kdtree/kdtree_flann.h>


/*
//#include <pcl/common/transforms.h>
#include <pcl/registration/lum.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/voxel_grid.h>     //sample in 3D -> Voxels
*/


#include <QFile>
#include <QTextStream>
#include <QTime>

typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

double off_x =0, off_y=0;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event)
{
    if (event.keyDown())
    {
        std::cout << "KeyboardEvent: ";

        if (event.getKeySym () == "i" )
            off_y= off_y+0.1;

        else if (event.getKeySym () == "k")
            off_y= off_y-0.1;

        else if (event.getKeySym () == "j")
            off_x= off_x-0.1;

        else if (event.getKeySym() == "l")
            off_x= off_x+0.1;
    }
    return;
}

int main(int argc, char *argv[])
{
    std::string program_path(argv[0]);
    size_t executable_name_idx = program_path.rfind("Protonect");

    std::string binpath = "/";

    if(executable_name_idx != std::string::npos)
    {
        binpath = program_path.substr(0, executable_name_idx);
    }

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;

    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    std::string serial = freenect2.getDefaultDeviceSerialNumber();

    for(int argI = 1; argI < argc; ++argI)
    {
        const std::string arg(argv[argI]);

        if(arg == "cpu")
        {
            if(!pipeline)
                pipeline = new libfreenect2::CpuPacketPipeline();
        }
        else if(arg == "gl")
        {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
            if(!pipeline)
                pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
            std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
        }
        else if(arg == "cl")
        {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
            if(!pipeline)
                pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
            std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
        }
        else if(arg.find_first_not_of("0123456789") == std::string::npos) //check if parameter could be a serial number
        {
            serial = arg;
        }
        else
        {
            std::cout << "Unknown argument: " << arg << std::endl;
        }
    }

    if(pipeline)
    {
        dev = freenect2.openDevice(serial, pipeline);
    }
    else
    {
        dev = freenect2.openDevice(serial);
    }

    if(dev == 0)
    {
        std::cout << "failure opening device!" << std::endl;
        return -1;
    }

    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    libfreenect2::FrameMap frames;
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), bigdepth(512, 424, 4);

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;


    libfreenect2::Freenect2Device::IrCameraParams paramIr = dev->getIrCameraParams();
    /*  paramIr.fx = 365.8662;
    paramIr.fy = 365.8662;
    paramIr.cx = 257.4943;
    paramIr.cy = 211.7746 ;
    paramIr.k1 = 0.08975793;
    paramIr.k2 =-0.2699445;
    paramIr.k3 = 0.09321829;*/


    libfreenect2::Registration* registration = new libfreenect2::Registration(paramIr, dev->getColorCameraParams());
    cout << dev->getColorCameraParams().cx << endl;
    cout << dev->getColorCameraParams().cy << endl;
    cout << dev->getColorCameraParams().fx << endl;
    cout << dev->getColorCameraParams().fy << endl;

    cout << dev->getIrCameraParams().cx << endl;
    cout << dev->getIrCameraParams().cy << endl;
    cout << dev->getIrCameraParams().fx << endl;
    cout << dev->getIrCameraParams().fy << endl;
    cout << dev->getIrCameraParams().k1 << endl;
    cout << dev->getIrCameraParams().k2 << endl;
    cout << dev->getIrCameraParams().k3 << endl;
    cout << dev->getIrCameraParams().p1 << endl;
    cout << dev->getIrCameraParams().p2 << endl;


    cv::Mat LookUpTableX = cv::Mat(424, 512, CV_32FC1);
    cv::Mat LookUpTableY = cv::Mat(424, 512, CV_32FC1);
    QFile file(QString("/home/silvio/PointClouds/ParamKinect/%1_Intrinsics.txt").arg(QString::fromUtf8(dev->getSerialNumber().c_str())) );

    if(file.exists() && file.open(QIODevice::ReadOnly))
    {
        QTextStream in(&file);

        int i = 0;
        while(!in.atEnd()) {
            QString line = in.readLine();
            QStringList fields = line.split(" ");
            if (fields.size() == 3)
            {
                LookUpTableX.at<float>(i/512,i%512) = fields.at(1).split("[X]").at(0).toFloat();
                LookUpTableY.at<float>(i/512,i%512) = fields.at(2).split("[Y]").at(0).toFloat();
                i++;
            }
        }
        file.close();
    }


    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->registerKeyboardCallback (keyboardEventOccurred);
    viewer->setBackgroundColor (0.5, 0.5, 0.5);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();


    QTime t;
    t.start();
  //  PointCloudT::Ptr PC1(new PointCloudT);
  //  pcl::visualization::PointCloudColorHandlerRGBField<PointT> col1(PC1);
  //  viewer->addPointCloud(PC1, col1, "Kinect1");
    PointCloudT::Ptr PC2(new PointCloudT);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> col2(PC2);
    viewer->addPointCloud(PC2, col2, "Kinect2");

   viewer->addText("FPS",50,50,50,255,255,255,"DIST");

    while (!viewer->wasStopped ())
    {

        t.restart();

        listener.waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        registration->apply(rgb,depth,&undistorted,&registered);


        cv::Mat MatDepth = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 1000.0f;
        cv::Mat MatUndist = cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data) / 1000.0f;
        cv::Mat MatReg = cv::Mat(registered.height, registered.width, CV_8UC4, registered.data);

        std::cout << QString("FPS = %1 Hz").arg(1000.0f/t.elapsed()).toStdString() << std::endl;
/*
       cv::cvtColor( MatReg,MatReg, CV_BGRA2RGB);



        PC2.reset(new PointCloudT);

        PC2->resize(512*424);
        PC2->width = 512;
        PC2->height = 424;

        for (int i = 0; i< MatUndist.rows ;i++)
        {
            for (int j = 0; j< MatUndist.cols ;j++)
            {

                cv::Vec3b color = MatReg.at<cv::Vec3b>(i,j);



              double Z = MatUndist.at<float>(i,j);

                if (  Z!=0  && color.val[0] > 0 &&  color.val[1] > 0 && color.val[2] > 0 )
                {
                // PointT P1;

                    // From Andrea
              //      P1.x = MatUndist.at<float>(i,j) * (j-paramIr.cx -off_x) / paramIr.fx;
               //     P1.y =-MatUndist.at<float>(i,j) * (i-paramIr.cy -off_y) / paramIr.fy;
               //     P1.z = MatDepth.at<float>(i,j);

                //    P1.r = color.val[0];
                //    P1.g = color.val[1];
                //    P1.b = color.val[2];
                //    PC1->push_back(P1);



                    // Other Method
                    PointT P2;
                    P2.r = color.val[0];
                    P2.g = color.val[1];
                    P2.b = color.val[2];
                    P2.z = Z;

                    P2.x = MatUndist.at<float>(i,j) * (j-paramIr.cx -off_x) / paramIr.fx;
                    P2.y = -MatUndist.at<float>(i,j) * (i-paramIr.cy -off_y) / paramIr.fy;

                    PC2->at(j,i) = P2;


                }
            }
        }

        // double meandist = DistTot/PC2->size() ;
        //  viewer->updateText(QString("%1x%2 : %3").arg(off_x).arg(off_y).arg(meandist).toStdString(),50,50,50,255,255,255,"DIST");
      //  viewer->updateText(QString("%1 x %2 x %3 : %4x%5").arg(DistTotX/PC2->size()).arg(DistTotY/PC2->size()).arg(DistTotZ/PC2->size()).arg(off_x).arg(off_y).toStdString(),50,50,50,255,255,255,"DIST");



        //

     //   pcl::visualization::PointCloudColorHandlerRGBField<PointT> col1(PC1);
      //  viewer->updatePointCloud(PC1,col1,"Kinect1");

        pcl::visualization::PointCloudColorHandlerRGBField<PointT> col2(PC2);
        viewer->updatePointCloud(PC2,col2,"Kinect2");

     //   viewer->setPointCloudSelected(1,"Kinect1");
    //    viewer->setPointCloudSelected(0,"Kinect2");


        //viewer->updateText(QString("Time = %1 ms").arg(t.elapsed()).toStdString(),50,50,50,255,255,255,"DIST");
     //   viewer->updateText(QString("FPS = %1 Hz").arg(1000.0f/t.elapsed()).toStdString(),50,50,50,255,255,255,"DIST");

        viewer->spinOnce (1);

*/
        listener.release(frames);


    }



    // TODO: restarting ir stream doesn't work!
    // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
    dev->stop();
    dev->close();

    delete registration;




    return 0;
}
