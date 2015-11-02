#include "mykinect.h"


MyKinect::MyKinect(std::string serial)
{
    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    _serial = serial;
    _open = false;
    _play = false;
    _save = false;
}



int MyKinect::Open(int i)
{
    if (_open)
    {
        std::cout << "Device is already open!" << std::endl;
        return ERROR;
    }

    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return ERROR;
    }


  //  pipeline = new libfreenect2::CpuPacketPipeline();

#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
    pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
    pipeline = new libfreenect2::CpuPacketPipeline();
#endif
#endif

    if (_serial == "")
        _serial = freenect2.getDeviceSerialNumber(i);

    dev = freenect2.openDevice(_serial,pipeline);



    if(dev == 0)
    {
        std::cout << "no device connected or failure opening the default one!" << std::endl;
        return ERROR;
    }

    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);
    dev->start();


    // Color
    registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());


    _open = true;

    PC.reset(new PointCloudT());
    PC->resize(512 * 424); // set the memory size to allocate
    PC->height = 424;        // set the height
    PC->width = 512;          // set the width
    PC->is_dense = false;                   // Kinect V2 returns organized and not dense point clouds

    return SUCCESS;
}

int MyKinect::Close()
{
    if (!_open)
    {
        std::cout << "already closed" << std::endl;
        return ERROR;
    }

    _open = false;
    // TODO: restarting ir stream doesn't work!
    // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
    dev->stop();
    dev->close();



    return SUCCESS;
}


PointCloudT::Ptr MyKinect::Grab()
{
    if (!_open)
    {
        std::cout << "stream not opened" << std::endl;
        PC.reset(new PointCloudT());
        return PC;
    }


    QTime t;
    t.start();



    // Acquire Frames
    libfreenect2::FrameMap frames;
    listener->waitForNewFrame(frames);

    QDateTime timestamp = QDateTime::currentDateTime();

    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];



    // Undistort and register frames
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
    registration->apply(rgb,depth,&undistorted,&registered);

    const float *undistorted_data = (float*)undistorted.data;
    const unsigned int *registered_data = (unsigned int*)registered.data;

    listener->release(frames);



    // Initialize my Point Cloud
 //   PC.reset(new PointCloudT());
 //   PC->resize(undistorted.width * undistorted.height); // set the memory size to allocate
 //   PC->height = undistorted.height;        // set the height
 //   PC->width = undistorted.width;          // set the width
 //   PC->is_dense = false;                   // Kinect V2 returns organized and not dense point clouds
    PC->header.stamp = timestamp.toMSecsSinceEpoch();                               // the stamp correspond to the acquisition time
    PC->header.frame_id = QString("%1/PointClouds/Kinect%2_%3.pcd").arg(QDir::homePath()).arg(_serial.c_str()).arg(timestamp.toString("yyyy-MM-dd-HH:mm:ss:zzz")).toStdString();


    // Set data into my Point cloud
    for (unsigned int i = 0; i < undistorted.height ;i++)
    {
        for (unsigned int j = 0; j < undistorted.width ;j++)
        {
            int index = i * undistorted.width + j;

            float depth = undistorted_data[index] / 1000.0f;
            unsigned int rgba = registered_data[index];

            PointT P;

            if ( depth != 0 && rgba != 0)
            {
                P.x = -depth * (dev->getIrCameraParams().cx - j) / dev->getIrCameraParams().fx;
                P.y =  depth * (dev->getIrCameraParams().cy - i) / dev->getIrCameraParams().fy;
                P.z =  depth;

                P.a = (rgba >> 24) & 0xFF;
                P.r = (rgba >> 16) & 0xFF;
                P.g = (rgba >> 8)  & 0xFF;
                P.b =  rgba        & 0xFF;
            }
            else
            {
                P.x = P.y = P.z = std::numeric_limits<float>::quiet_NaN();
            }

            PC->at(j,i) = P;
        }
    }

    std::cout << QDateTime::currentDateTime().toString().toStdString() << " time elapsed :" << t.elapsed() << "  PointCloud is : " << PC->size() << std::endl;


    return PC;
}
