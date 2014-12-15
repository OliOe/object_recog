#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/common/centroid.h>
//#include <pcl/common/eigen.h>
#include <visualization_msgs/Marker.h> //New
#include <tf/transform_listener.h>


#include "util.h"


typedef pcl::PointXYZRGBA PointT;



class zylLokalisation
{

private:
 
    bool gotData;
    Parameter parameter;

    pcl::visualization::CloudViewer viewer;

    pcl::PCDReader reader;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;

    pcl::PointCloud<PointT>::Ptr cloud_cylinder;
    pcl::PointCloud<PointT>::Ptr cloud_cylinder_with_points;
    pcl::PointCloud<PointT>::Ptr cloud_org;
    
    std::string orgCloudPath;
    visualization_msgs::Marker center; //New

public:

    zylLokalisation() : viewer("Lokalisation Viewer"),  cloud_org(new pcl::PointCloud<PointT>()),cloud_cylinder(new pcl::PointCloud<PointT> ()),
        cloud_cylinder_with_points(new pcl::PointCloud<PointT> ())
    {
        this->gotData = false;
        this->viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&parameter);
    }



    void cloudPathCallback(const sensor_msgs::PointCloud2ConstPtr& input)
    {

        // Objekte
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
        pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);



        // Cloud File einlesen
        //reader.read (cloudPath->data.c_str(), *cloud);
        pcl::fromROSMsg (*input, *cloud);
        std::cout << "PointCloud besteht aus: " << cloud->points.size () << " Punkten." << std::endl;

        // Input-PointCloud leer?
        if(cloud->points.size () > 1)
        {
                // Punkt-Normalen ermitteln
                std::cout << "Ermittle Punkt-Normalen..." << std::endl;

                ne.setSearchMethod (tree);
                ne.setInputCloud (cloud);
                ne.setKSearch (50);
                ne.compute (*cloud_normals);


                // Erstelle Segmentation Objekt für zylindrisches Objekt
                seg.setOptimizeCoefficients (true);
                seg.setModelType (pcl::SACMODEL_CYLINDER);
                seg.setMethodType (pcl::SAC_RANSAC);
                seg.setNormalDistanceWeight (0.1);
                seg.setMaxIterations (10000);
                seg.setDistanceThreshold (0.05);
                seg.setRadiusLimits (0, 0.1);             // kleiner 10cm
                seg.setInputCloud (cloud);
                seg.setInputNormals (cloud_normals);

                // Segmentiere zylindrische Inliers und Koeffizienten
                std::cout << "Versuche zylindrisches Objekt zu Segmentieren..." << std::endl;
                seg.segment (*inliers_cylinder, *coefficients_cylinder);
                std::cout << "Zylindrische Koeffizienten: " << *coefficients_cylinder << std::endl;
                


                // Extrahiere zylindrische Inliers
                extract.setInputCloud (cloud);
                extract.setIndices (inliers_cylinder);
                extract.setNegative (false);
                extract.filter (*cloud_cylinder);


                // Punkte
                pcl::PointXYZRGBA point1, point2, point3,point4;
                Eigen::Vector4f Massepunkt;
                Eigen::Vector3f Mittelpunkt;

                // Massenpunkt der Wolke
                pcl::compute3DCentroid(*cloud_cylinder, Massepunkt);

               /* point1.x = Massepunkt[0];
                point1.y = Massepunkt[1];
                point1.z = Massepunkt[2];
                point1.g = 255;
                point1.r = point1.b = 0;



                // Stützpunkt der Symmetrieachse
                point2.x = coefficients_cylinder->values[0];
                point2.y = coefficients_cylinder->values[1];
                point2.z = coefficients_cylinder->values[2];
                point2.b = 255;
                point2.r = point2.g = 0;*/

                //--------------------------------------------------------


                // Mittelpunkt des Zylinders
                findeMittelpunktZylinder(coefficients_cylinder, Massepunkt, &Mittelpunkt);
                std::cout << "Mittelpunkt:\n" << Mittelpunkt << std::endl;

                point3.x = Mittelpunkt[0];
                point3.y = Mittelpunkt[1];
                point3.z = Mittelpunkt[2];
                point3.r = 255;
                point3.g = point3.b = 0;

                //New - Transformation und Ausgabe des Mittelpunkts
                geometry_msgs::PointStamped kinect_point;
                kinect_point.header.frame_id = "kinect_link";
                kinect_point.header.stamp = ros::Time();
                kinect_point.point.x = Mittelpunkt[2];
                kinect_point.point.y = Mittelpunkt[0]* (-1);
                kinect_point.point.z = Mittelpunkt[1]* (-1);
                geometry_msgs::PointStamped base_point;
                listener.transformPoint("base_link", kinect_point, base_point);

        		//New - Ausgabe des Mittelpunkts
        		center.header.frame_id = "base_link";
        		center.header.stamp = ros::Time();
        		center.ns = "my_namespace";
        		center.id = 0;
        		center.type = visualization_msgs::Marker::CYLINDER;
        		center.action = visualization_msgs::Marker::ADD;
                center.pose.position.x = base_point.point.x;	//Achsen vertauscht - Warum?
                center.pose.position.y = base_point.point.y; ;//* (-1);
                center.pose.position.z = base_point.point.z; ;//* (-1);
        		center.pose.orientation.x = 0.0;
        		center.pose.orientation.y = 0.0;
        		center.pose.orientation.z = 0.0;
        		center.pose.orientation.w = 0.0;
        		center.scale.x = 0.1;
        		center.scale.y = 0.1;
        		center.scale.z = 0.2;
        		center.color.a = 1.0;
        		center.color.r = 1.0;
        		center.color.g = 0.0;
        		center.color.b = 0.0;
        		center.frame_locked = true;

                /*// neue PointCloud mit Punkten
                pcl::copyPointCloud(*cloud_cylinder, *cloud_cylinder_with_points);

                point4.r = 255;
                point4.g = 127;
                point4.b = 0;
                //Füge SymetrieAchse hinzu
                for(int i=-150; i<150; i++){
                    point4.x = Mittelpunkt[0] + (i*0.001)*coefficients_cylinder->values[3];
                    point4.y = Mittelpunkt[1] + (i*0.001)*coefficients_cylinder->values[4];
                    point4.z = Mittelpunkt[2] + (i*0.001)*coefficients_cylinder->values[5];

                    cloud_cylinder_with_points->push_back(point4);
                    cloud_org->push_back(point4);
                    }
                //Kreuz um Stützpunkt besser zu sehen
                for(int i=-10; i<10; i++){
                    point4.x = coefficients_cylinder->values[0] + (i*0.001)*coefficients_cylinder->values[3];
                    point4.y = coefficients_cylinder->values[1] + (i*0.001)*coefficients_cylinder->values[4];
                    point4.z = coefficients_cylinder->values[2] + (i*0.001)*coefficients_cylinder->values[5];

                    cloud_cylinder_with_points->push_back(point4);
                    cloud_org->push_back(point4);
                    }

                for(int i=-10; i<10; i++){
                    point4.x = coefficients_cylinder->values[0] + (i*0.001)*coefficients_cylinder->values[3]*(-1);
                    point4.y = coefficients_cylinder->values[1];
                    point4.z = coefficients_cylinder->values[2] + (i*0.001)*coefficients_cylinder->values[5];

                    cloud_cylinder_with_points->push_back(point4);
                    cloud_org->push_back(point4);
                    }
                for(int i=-10; i<10; i++){
                    point4.x = coefficients_cylinder->values[0] + (i*0.001)*coefficients_cylinder->values[3]*(-1);
                    point4.y = coefficients_cylinder->values[1] + (i*0.001)*coefficients_cylinder->values[4];
                    point4.z = coefficients_cylinder->values[2];

                    cloud_cylinder_with_points->push_back(point4);
                    cloud_org->push_back(point4);
                    }
                // ---------------------------------------------------------------------------------------------------

                cloud_cylinder_with_points->push_back(point1);
                cloud_org->push_back(point1);
                cloud_cylinder_with_points->push_back(point2);
                cloud_org->push_back(point2);
                cloud_cylinder_with_points->push_back(point3);
                cloud_org->push_back(point3);*/


                // zyl. Objekt gefunden?
                if (cloud_cylinder->points.empty ())
                    std::cerr << "Kein zylindrisches Objekt gefunden!" << std::endl;
                else
                {
                    // Speichere zylindrisches Objekt als PointCloud
                    std::cout << "Zylindrische PointCloud besteht aus: " << cloud_cylinder->points.size () << " Punkten." << std::endl;
                    writer.write ("cylinder.pcd", *cloud_cylinder, false);

                    this->gotData = true;
                }

            }

            else {
                std::cerr << "Input-Cloud enthält keine Punkte!" << std::endl;
            }

    }
    void orgCloudPathCallback(const sensor_msgs::PointCloud2ConstPtr& input){
        /*orgCloudPath = cloudPath->data.c_str();
        reader.read (orgCloudPath,*cloud_org);*/
        pcl::fromROSMsg (*input, *cloud_org);
        std::cout << "Orginal Cloud besteht aus: " << cloud_org->points.size () << " Punkten.----------------" << std::endl;
    }




    void run ()
    {
        ros::NodeHandle nh;
        ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("object_cloud", 1, &zylLokalisation::cloudPathCallback, this);
        ros::Publisher center_pub = nh.advertise<visualization_msgs::Marker>( "cup_center", 0 ); //New
        ros::Subscriber sub2 = nh.subscribe<sensor_msgs::PointCloud2>("filtered_cloud", 1, &zylLokalisation::orgCloudPathCallback, this);

        ros::Rate loop_rate(10);

        while (!viewer.wasStopped() && !parameter.stop)
        {
            if(gotData)
            {
                // welche PointCloud anzeigen?
                if(parameter.showObject)
                {
                    if(parameter.showPoints)
                        viewer.showCloud(cloud_cylinder_with_points);
                    else
                        viewer.showCloud(cloud_cylinder);
                }
                else
                {
                    viewer.showCloud(cloud_org);
                }
		center_pub.publish(center); //New
            }

            ros::spinOnce();
            loop_rate.sleep();
        }

    }


};


int main (int argc, char** argv)
{
    ros::init (argc, argv, "zylLokalisation");

    zylLokalisation modelSegmentation;
    modelSegmentation.run();

    return 0;
}

