#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <pcl/visualization/cloud_viewer.h>


class Parameter
{
public:
    int count;
    int maxCount;
    bool stop;
    bool send;
    bool change;

    Parameter();

};

Parameter::Parameter()
{
    this->count=0;
    this->maxCount=0;
    this->stop=false;
    this->send=false;
    this->change = false;

}


// Funktion MUSS außerhalb der Klasse sein sonst gibt es einen Fehler beim compilen
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* daten)
{

    Parameter* parameter = static_cast<Parameter*> (daten);

   // std::cout << event.getKeyCode() << std::endl;


    if (event.getKeySym() == "Up" && event.keyDown())
    {

        parameter->count++;
        if(parameter->count>=parameter->maxCount)
            parameter->count=0;
        parameter->change = true;
    }

    if (event.getKeySym() == "Down" && event.keyDown())
    {

        parameter->count--;
        if(parameter->count<0)
            parameter->count=0;
        parameter->change = true;
    }

    if (event.getKeySym() == "s" && event.keyDown())
    {
       parameter->send = true;
    }

}
