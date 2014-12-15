#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <pcl/visualization/cloud_viewer.h>


class Parameter
{
public:

    bool showObject;
    bool stop;
    bool showPoints;

    Parameter();

};

Parameter::Parameter()
{
    this->showObject = true;
    this->stop = false;
    this->showPoints = false;

}


// Funktion MUSS außerhalb der Klasse sein sonst gibt es einen Fehler beim compilen
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* daten)
{

    Parameter* parameter = static_cast<Parameter*> (daten);

    //std::cout << event.getKeyCode() << std::endl;

    if (event.getKeySym() == "Escape" && event.keyDown())
    {
        parameter->stop = true;
    }

    if (event.getKeySym() == "space" && event.keyDown())
    {
        parameter->showObject = !parameter->showObject;
    }

    if (event.getKeySym() == "x" && event.keyDown())
    {
        parameter->showPoints = !parameter->showPoints;
    }
}


void findeMittelpunktZylinder(pcl::ModelCoefficients::Ptr cyl_coeff, Eigen::Vector4f Massepunkt, Eigen::Vector3f *Mittelpunkt)
{

    float px = cyl_coeff->values[0], py = cyl_coeff->values[1], pz = cyl_coeff->values[2];
    float rx = cyl_coeff->values[3], ry = cyl_coeff->values[4], rz = cyl_coeff->values[5];
    float mx = Massepunkt[0], my = Massepunkt[1], mz = Massepunkt[2];

    float lambda = ( (mx*rx + my*ry + mz*rz) - (px*rx + py*ry + pz*rz) ) / (rx*rx + ry*ry + rz*rz);

    (*Mittelpunkt)[0] = px + (lambda * rx);
    (*Mittelpunkt)[1] = py + (lambda * ry);
    (*Mittelpunkt)[2] = pz + (lambda * rz);

}
