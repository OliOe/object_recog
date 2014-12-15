#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <pcl/visualization/cloud_viewer.h>


class Parameter
{
public:

    bool stop;
    bool savePCL;
    bool filter;
    std_msgs::Float64 angle;

    Parameter();

};

Parameter::Parameter()
{
    this->stop = false;
    this->savePCL = false;
    this->filter = false;
    this->angle.data = 0.0;

}


// Funktion MUSS außerhalb der Klasse sein sonst gibt es einen Fehler beim compilen
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* daten)
{

    Parameter* parameter = static_cast<Parameter*> (daten);

    //std::cout << event.getKeyCode() << std::endl;


    if (event.getKeySym() == "Down" && event.keyDown())
    {
        if(parameter->angle.data > -30.0f)
            parameter->angle.data -= 5;
    }

    if (event.getKeySym() == "Up" && event.keyDown())
    {
        if(parameter->angle.data < 30.0f)
            parameter->angle.data += 5;
    }


    if (event.getKeySym() == "Escape" && event.keyDown())
    {
        parameter->stop = true;
    }


    if (event.getKeySym() == "s" && event.keyDown())
    {
        parameter->savePCL = true;
    }

    if (event.getKeySym() == "space" && event.keyDown())
    {
        parameter->filter = !parameter->filter;
    }
}
