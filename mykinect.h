#ifndef MYKINECT_H
#define MYKINECT_H



#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>


#include <QTime>
#include <QDir>
#include <QString>

typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

#define SUCCESS 0
#define ERROR 1

class MyKinect
{
public:
    MyKinect(std::string serial = "");

    int Open(int i=0);
    int Close();
    PointCloudT::Ptr Grab();

    bool _open;
    bool _play;
    bool _save;

private:
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev;
    libfreenect2::SyncMultiFrameListener *listener;
    libfreenect2::Registration *registration;
    libfreenect2::BasePacketPipeline *pipeline;




    std::string _serial;

    PointCloudT::Ptr PC;

};

#endif // MYKINECT_H
