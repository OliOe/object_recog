#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include "util.h"


class CloudGrabber
{

private:

    pcl::visualization::CloudViewer viewer;
    Parameter parameter;
    std::string workingPath;
    std_msgs::String cloudPath;
    bool sendWorkingPath;

    //pcl::PCLPointCloud2 outputPCL;
    sensor_msgs::PointCloud2 outputPCL;


public:

    CloudGrabber (std::string workingPath) : viewer("Cloud Viewer")
    {
        this->sendWorkingPath = false;
        this->workingPath = workingPath;
        this->viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&parameter);
    }


    void CloudCallback (const sensor_msgs::PointCloud2ConstPtr& input)
    {
	//pcl::PCLPointCloud2 inputPCL;

        if (!viewer.wasStopped())
        {

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_viewer(new pcl::PointCloud<pcl::PointXYZRGBA>());

	    //pcl_conversions::toPCL(*input, inputPCL);

            //const pcl::PCLPointCloud2 inputPCLconst = inputPCL;

            pcl::fromROSMsg (*input, *cloud);


            //Transformationsmatrix für Rotation um 180° um die X-Achse
            Eigen::Matrix4f TransMat;
            TransMat <<     1,    0,            0,              0,
                            0,    cos(M_PI),    -sin(M_PI),     0,
                            0,    sin(M_PI),    cos(M_PI),      0,
                            0,    0,            0,              1;

            


            if (parameter.filter)
            {
                pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
                sor.setInputCloud (cloud);
                sor.setMeanK (50);
                sor.setStddevMulThresh (1.0);
                sor.filter (*cloud_filtered);
                //Rotation um 180° um die X-Achse wegen Kinect
                pcl::transformPointCloud(*cloud_filtered,*cloud_viewer,TransMat);
                viewer.showCloud(cloud_viewer);
            }

            else
                //Rotation um 180° um die X-Achse wegen Kinect
                pcl::transformPointCloud(*cloud,*cloud_viewer,TransMat);
                viewer.showCloud(cloud_viewer);


            if (parameter.savePCL)
            {
                std::stringstream ss;

                if(!sendWorkingPath)
                {
                    unsigned pos = workingPath.find_last_of("/\\");

                    ss << workingPath.substr(0,pos+1) << "cloud.pcd";
                    cloudPath.data = ss.str();
                    sendWorkingPath = true;
                }

                //std::cout << cloud->points.size() << std::endl;
                pcl::toROSMsg(*cloud, outputPCL);

                //pcl::io::savePCDFileASCII (ss.str(), *cloud);

                parameter.savePCL = false;


            }

            if (parameter.savePCL && parameter.filter)
            {
                std::stringstream ss;

                if(!sendWorkingPath)
                {
                    unsigned pos = workingPath.find_last_of("/\\");

                    ss << workingPath.substr(0,pos+1) << "cloud.pcd";
                    cloudPath.data = ss.str();
                    sendWorkingPath = true;
                }

                //std::cout << cloud_filtered->points.size() << std::endl;
                pcl::toROSMsg(*cloud_filtered, outputPCL);

                //pcl::io::savePCDFileASCII ("cloud.pcd", *cloud_filtered);

                parameter.savePCL = false;
            }

        }

    }



    void run ()
    {
        ros::NodeHandle nh;
	
        ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("kinect/depth_registered/points", 1, &CloudGrabber::CloudCallback, this);
        ros::Publisher pub = nh.advertise<std_msgs::Float64>("tilt_angle", 1);
        ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);

        ros::Rate loop_rate(10);


        while (!viewer.wasStopped() && !parameter.stop)
        {
            if(sendWorkingPath)
            {
                pub2.publish(outputPCL);
                sendWorkingPath = false;
            }

            pub.publish(parameter.angle);
            ros::spinOnce();
            loop_rate.sleep();
        }

    }


};


int main (int argc, char** argv)
{
    ros::init (argc, argv, "CloudGrabber");

    CloudGrabber cloudGrabber(argv[0]);
    cloudGrabber.run();

    return 0;
}
