#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>


#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <cylRecognition/objects.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

//#include <pcl/common/centroid.h>
//#include <pcl/common/eigen.h>

#include <visualization_msgs/Marker.h> //New
#include <tf/transform_listener.h> //New


#include "util.h"


typedef pcl::PointXYZRGBA PointT;



class CylRecognition
{

private:

    bool gotData;
    Parameter parameter;

    //pcl::visualization::CloudViewer viewer;

    pcl::PassThrough<PointT> pass;
    pcl::SACSegmentation<PointT> seg;
    pcl::ExtractIndices<PointT> extract;
    pcl::EuclideanClusterExtraction<PointT> clust_ext;
    pcl::VoxelGrid<PointT> vox;
    tf::TransformListener listener;

    cylRecognition::objects obj;
    std::string workingPath;


public:

    CylRecognition(std::string workingPath)
    {
        this->workingPath = workingPath;
        this->gotData = false;
        this->obj.Count=0;
    }



    void cloudPathCallback(const sensor_msgs::PointCloud2ConstPtr& input)
    {

        // Objekte
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
        pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_filtered3 (new pcl::PointCloud<PointT>);
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

        visualization_msgs::Marker center; //New
        


        pcl::fromROSMsg (*input, *cloud);

        // Cloud File einlesen
        //reader.read (cloudPath->data.c_str(), *cloud);

        std::cout << "PointCloud empfangen." << std::endl;

        if(cloud->points.size () > 0)
        {

            // Passthrough-Filter für NaNs und max. Entfernung
            std::cout << "Filtere PointCloud..." << std::endl;
            //std::cout << cloud->points.size() << std::endl;
            pass.setInputCloud (cloud);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (0, 1.5);
            pass.filter (*cloud_filtered);

            //Test: filter umgehen
            //pcl::copyPointCloud(*cloud, *cloud_filtered);

            //std::cout << "DEBUG0" << std::endl;
            // mindestens 2 Punkte um einen eindeutigen Punkt zu bestimmen
            //std::cout << cloud_filtered->points.size() << std::endl;
            if(cloud_filtered->points.size() > 1)
            {
                //std::cout << "DEBUG1" << std::endl;
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_PLANE);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setMaxIterations(100);
                seg.setDistanceThreshold(0.03);

                //std::cout << "DEBUG2" << std::endl;
                int i=0, nr_points = (int) cloud_filtered->points.size();
                while(cloud_filtered->points.size()>0.3 *nr_points)
                    {
                    seg.setInputCloud(cloud_filtered);
                    seg.segment(*inliers_plane,*coefficients_plane);

                    pcl::ExtractIndices<PointT> extra;
                    extra.setInputCloud(cloud_filtered);
                    extra.setIndices(inliers_plane);
                    extra.setNegative(true);
                    extra.filter(*cloud_filtered2);
                    *cloud_filtered = *cloud_filtered2;
                    }
                //std::cout << "DEBUG3" << std::endl;
                tree->setInputCloud(cloud_filtered);

                std::vector<pcl::PointIndices> cluster_ind;

                clust_ext.setClusterTolerance(0.02);
                clust_ext.setMinClusterSize(400);
                clust_ext.setMaxClusterSize(50000);
                clust_ext.setSearchMethod(tree);
                clust_ext.setInputCloud(cloud_filtered);
                clust_ext.extract(cluster_ind);

                std::stringstream path;
                unsigned pos = workingPath.find_last_of("/\\");
                pcl::PointXYZRGBA point1;
                int nrOfCyl=0;
                int nrOfObj=0;
                //std::cout << "DEBUG4" << std::endl;
                for(std::vector<pcl::PointIndices>::const_iterator it = cluster_ind.begin(); it != cluster_ind.end(); ++it)
                {
                    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
                    pcl::NormalEstimation<PointT, pcl::Normal> ne;
                    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg2;
                    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT>);



                    pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
                    sensor_msgs::PointCloud2 object;
                    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++){
                        point1 = cloud_filtered->points[*pit];
                        point1.r=255;
                        point1.g=0;
                        point1.b=0;
                        cloud_cluster->points.push_back(point1);//cloud_filtered->points[*pit]);
                    }
                    cloud_cluster->width = cloud_cluster->points.size();
                    cloud_cluster->height=1;
                    cloud_cluster->is_dense=true;

                    /*std::stringstream pcdName;
                            pcdName <<workingPath.substr(0,pos+1)<< "object_" <<j <<".pcd";
                    writer.write<PointT>(pcdName.str(),*cloud_cluster,false);*/


                    //Zylinder-Lokalisierung
                    std::cout << "Ermittle Punkt-Normalen..." << std::endl;

                    ne.setSearchMethod (tree);
                    ne.setInputCloud (cloud_cluster);
                    ne.setKSearch (50);
                    ne.compute (*cloud_normals);

                    //ros::WallDuration(5.0).sleep();

                    // Erstelle Segmentation Objekt für zylindrisches Objekt
                    seg2.setOptimizeCoefficients (true);
                    seg2.setModelType (pcl::SACMODEL_CYLINDER);
                    seg2.setMethodType (pcl::SAC_RANSAC);
                    seg2.setNormalDistanceWeight (0.1);
                    seg2.setMaxIterations (10000);
                    seg2.setDistanceThreshold (0.01);
                    seg2.setRadiusLimits (0, 0.1);             // kleiner 10cm
                    seg2.setInputCloud (cloud_cluster);
                    seg2.setInputNormals (cloud_normals);

                    // Segmentiere zylindrische Inliers und Koeffizienten
                    std::cout << "Versuche zylindrisches Objekt zu Segmentieren..." << std::endl;
                    seg2.segment (*inliers_cylinder, *coefficients_cylinder);
                    std::cout << "Zylindrische Koeffizienten: " << *coefficients_cylinder << std::endl;


                    // Extrahiere zylindrische Inliers
                    std::cout << "Zylindrische Inliers 1" << std::endl;
                    extract.setInputCloud (cloud_cluster);
                    std::cout << "Zylindrische Inliers 2" << std::endl;
                    extract.setIndices (inliers_cylinder);
                    std::cout << "Zylindrische Inliers 3" << std::endl;
                    extract.setNegative (false);
                    std::cout << "Zylindrische Inliers 4" << std::endl;
                    extract.filter (*cloud_cylinder);


                    // Punkte
                    std::cout << "Punkte" << std::endl;
                    pcl::PointXYZRGBA point1, point2, point3,point4;
                    Eigen::Vector4f Massepunkt;
                    Eigen::Vector3f Mittelpunkt;

                    // Massenpunkt der Wolke
                    std::cout << "Massepunkt" << std::endl;
                    pcl::compute3DCentroid(*cloud_cylinder, Massepunkt);

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
	            std::cout << "asdqwe";
                    listener.transformPoint("base_link", kinect_point, base_point);
			std::cout << "asdqwe2";

                    //New - Ausgabe des Mittelpunkts
                    center.header.frame_id = "base_link";
                    center.header.stamp = ros::Time();
                    center.ns = "my_namespace";
                    center.id = 0;
                    center.type = visualization_msgs::Marker::CYLINDER;
                    center.action = visualization_msgs::Marker::ADD;
                    center.pose.position.x = base_point.point.x;    //Achsen vertauscht - Warum?
                    center.pose.position.y = base_point.point.y; ;//* (-1);
                    center.pose.position.z = base_point.point.z; ;//* (-1);

                    center.pose.orientation.x = 0.0;
                    center.pose.orientation.y = 0.0;
                    center.pose.orientation.z = 0.0;
                    center.pose.orientation.w = 0.0;
                    center.scale.x = 0.01;
                    center.scale.y = 0.01;
                    center.scale.z = 0.02;
                    center.color.a = 1.0;
                    center.color.r = 1.0;
                    center.color.g = 0.0;
                    center.color.b = 0.0;
                    center.frame_locked = true;


                    // zyl. Objekt gefunden?
                    if (cloud_cylinder->points.size () < 100)
                        std::cerr << "Objekt " << nrOfObj << " ist kein Zylinder!" << std::endl;
                    else
                    {
                        nrOfCyl++;

                        pcl::toROSMsg(*cloud_cluster, object);
                        obj.Objects.push_back(object);
                        obj.Centers.push_back(center);
                        
                        std::cout << "Zylindrische PointCloud von Objekt " << nrOfObj << " besteht aus: " << cloud_cylinder->points.size () << " Punkten." << std::endl;
                    }

                    nrOfObj++;

                }
                obj.Count=nrOfCyl;

                /*path << workingPath.substr(0,pos+1);
                obj.Path = path.str();*/

                std::cout <<"Es wurden " << nrOfObj <<" Objekte und " << nrOfCyl << " Zylinder gefunden!"<<endl;



            }
            else
            {    
                std::cerr << "Gefilterte Cloud enthält keine Punkte!" << std::endl;
            }
        }
        else 
        {
            std::cerr << "Input-Cloud enthält keine Punkte!" << std::endl;
        }

    }




    void run ()
    {
        ros::NodeHandle nh;
        ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("filtered_cloud", 1, &CylRecognition::cloudPathCallback, this);
        ros::Publisher pub = nh.advertise<cylRecognition::objects>("recognized_objects", 1);
        ros::Rate loop_rate(1);

        std::cout<<endl<< "Warte auf Daten..." <<endl;

        while (!parameter.stop && ros::ok())
        {
            if(obj.Count>0)
            {
                pub.publish(obj);
                obj.Count=0;
                obj.Objects.clear();
                obj.Centers.clear();
            }
            ros::spinOnce();
            loop_rate.sleep();
        }

    }


};


int main (int argc, char** argv)
{
    ros::init (argc, argv, "cylRecognition");

    CylRecognition cylRecognition(argv[0]);
    cylRecognition.run();

    return 0;
}

