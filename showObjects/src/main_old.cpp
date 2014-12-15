#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <cloudSegmentation/objects.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "util.h"


class ShowObjects
{

private:

    pcl::PCDReader reader;

    pcl::visualization::CloudViewer viewer;
    Parameter parameter;
    std::string workingPath;
    //std::string cloudPath;
    //pcl::PCLPointCloud2 inputCloud;
    cloudSegmentation::objects obj;
    


public:

    ShowObjects (std::string workingPath) : viewer("Object Viewer")
    {
        this->workingPath = workingPath;
        this->viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&parameter);
    }


    void CloudCallback (const cloudSegmentation::objects::ConstPtr& input)
    {
        if (!viewer.wasStopped())
        {

            obj = *input;
            parameter.maxCount = input->Count;
            //cloudPath = objectPath->Path;
            parameter.count=0;
            parameter.change = true;


        }

    }



    void run ()
    {
        ros::NodeHandle nh;
        ros::Subscriber sub = nh.subscribe<cloudSegmentation::objects>("recognized_objects", 1, &ShowObjects::CloudCallback, this);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("object_cloud", 1);
        std_msgs::String objPath;
        int idx=0,helpIdx=0;
        ros::Rate loop_rate(10);


        while (!viewer.wasStopped() && !parameter.stop)
        {
            if(parameter.change){
                // Cloud File einlesen
                /*std::stringstream path;
                path << cloudPath << "object_" <<parameter.count<<".pcd";
                reader.read(path.str(),*cloud);*/
                pcl::fromROSMsg(obj.Objects[parameter.count], *cloud);
                std::cout << "PointCloud besteht aus: " << cloud->points.size () << " Punkten." << std::endl;
                viewer.showCloud(cloud);
                parameter.change = false;
                //objPath.data = path.str();
            }
            if(parameter.send)
            {
                pub.publish(obj.Objects[parameter.count]);
                parameter.send = false;
            }
            ros::spinOnce();
            loop_rate.sleep();
        }

    }


};


int main (int argc, char** argv)
{
    ros::init (argc, argv, "ShowObjects");

    ShowObjects cloudGrabber(argv[0]);
    cloudGrabber.run();

    return 0;
}
