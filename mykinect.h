#ifndef MYKINECT_H
#define MYKINECT_H




class MyKinect
{
public:
    MyKinect();

    bool Open();
    bool Close();
    bool Play();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Grab();


private:


};

#endif // MYKINECT_H
