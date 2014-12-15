#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <cylRecognition/objects.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include "util.h"


class ShowObjects
{

private:

    pcl::PCDReader reader;

    pcl::visualization::CloudViewer viewer;
    Parameter parameter;
    std::string workingPath;
    //std::string cloudPath;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_org;
    cylRecognition::objects obj;

    //Transformationsmatrix für Rotation um 180° um die X-Achse
    Eigen::Matrix4f TransMat;
    
    


public:

    ShowObjects (std::string workingPath) : viewer("Object Viewer"), cloud_org(new pcl::PointCloud<pcl::PointXYZRGBA>())
    {
        this->workingPath = workingPath;
        this->viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&parameter);
        TransMat <<     1,    0,            0,              0,
                        0,    cos(M_PI),    -sin(M_PI),     0,
                        0,    sin(M_PI),    cos(M_PI),      0,
                        0,    0,            0,              1;
        //std::cout << TransMat << std::endl;
    }


    void CloudCallback (const cylRecognition::objects::ConstPtr& input)
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


    void orgCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input){
        /*orgCloudPath = cloudPath->data.c_str();
        reader.read (orgCloudPath,*cloud_org);*/
        //std::cout << "DEBUG_ORG1" << std::endl;
        pcl::fromROSMsg (*input, *cloud_org);
        //std::cout << "DEBUG_ORG2" << std::endl;
        std::cout << "Orginal Cloud besteht aus: " << cloud_org->points.size () << " Punkten.----------------" << std::endl;
        //std::cout << "DEBUG_ORG3" << std::endl;
        //Rotation um 180° um die X-Achse wegen Kinect
        pcl::transformPointCloud(*cloud_org,*cloud_org,TransMat);
        //std::cout << "DEBUG_ORG4" << std::endl;
    }

    void run ()
    {
        ros::NodeHandle nh;
        ros::Subscriber sub = nh.subscribe<cylRecognition::objects>("recognized_objects", 1, &ShowObjects::CloudCallback, this);
        ros::Subscriber sub2 = nh.subscribe<sensor_msgs::PointCloud2>("filtered_cloud", 1, &ShowObjects::orgCloudCallback, this);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("cup_center", 1);
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
                //std::cout << "DEBUG1" << std::endl;
                pcl::fromROSMsg(obj.Objects[parameter.count], *cloud);
                std::cout << "PointCloud von Objekt " << parameter.count << " besteht aus: " << cloud->points.size () << " Punkten." << std::endl;

                //Rotation um 180° um die X-Achse wegen Kinect
                pcl::transformPointCloud(*cloud,*cloud,TransMat);
                //std::cout << "DEBUG2" << std::endl;

                //Einfärben
                /*for(int i=0; i<cloud->points.size(); i++)
                {
                    cloud->points[i].g = 0;
                    cloud->points[i].b = 0;
                    //cloud->points[i].z += 0.05;
                }*/
                
                viewer.showCloud(cloud_org,"cloud_org");
                //std::cout << "DEBUG3" << std::endl;
                viewer.showCloud(cloud,"cloud");
                //std::cout << "DEBUG4" << std::endl;
                parameter.change = false;
                //objPath.data = path.str();
            }
            if(parameter.send)
            {
                pub.publish(obj.Centers[parameter.count]);
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
