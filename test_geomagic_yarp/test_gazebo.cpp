/* 
 * This example shows how to interact with gazebo (eg. using the 'selection' message)
 */

#include <cmath>
#include <string>
#include <algorithm>
#include <map>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <hapticdevice/IHapticDevice.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <boost/thread/mutex.hpp>

#include "fakeGeomagicDriver.h"
#include "cartesianClient.h"

#define DEG2RAD     (M_PI/180.0)
#define RAD2DEG     (180.0/M_PI)
#define MAX_TORSO_PITCH     0.0    // [deg]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace hapticdevice;

class Test: public RFModule
{
protected:
    // geomagic  
    PolyDriver drvGeomagic;
    IHapticDevice *igeo;    
    bool fakehaptic;
    
    // cartesian
    CartesianClient client_left;
    CartesianClient client_right;
    CartesianClient *selected_part;
    Vector xd;
    Vector od;
    
    string part;
    boost::mutex mx;
    
    bool initGeomagic(const string &name, const string &geomagic)
    {
        Property optGeo("(device hapticdeviceclient)");
        optGeo.put("remote",("/"+geomagic).c_str());
        optGeo.put("local",("/"+name+"/geomagic").c_str());
        if (!drvGeomagic.open(optGeo))
            return false;
        drvGeomagic.view(igeo);              

        Matrix T=zeros(4,4);
        T(0,1)=1.0;
        T(1,2)=1.0;
        T(2,0)=1.0;
        T(3,3)=1.0;
        igeo->setTransformation(SE3inv(T));
        igeo->setCartesianForceMode();
        return true;
    }
    
    void closeGeomagic()
    {
        igeo->stopFeedback();
        igeo->setTransformation(eye(4,4));
        drvGeomagic.close();
    }
    
    bool initCartesian(const string &robot, bool swap_x, bool swap_y)
    {                        
        client_left.init(robot,"left_arm",swap_x,swap_y);
        client_right.init(robot,"right_arm",swap_x,swap_y);
       
        xd.resize(3);
        od.resize(4);
        return true;
    }
    
    void closeCartesian()
    {    
      client_left.close();
      client_right.close();
    }

public:
    Test(): RFModule(), selected_part(nullptr)
    {
    }
    
    bool configure(ResourceFinder &rf)
    {    
        string name=rf.check("name",Value("test_geomagic_yarp")).asString().c_str();
        string robot=rf.check("robot",Value("icubGazeboSim")).asString().c_str();	
        string geomagic=rf.check("geomagic",Value("geomagic")).asString().c_str();
        fakehaptic=rf.check("fakehaptic",Value("false")).asBool();
        bool swap_x=rf.check("swap_x",Value("false")).asBool();
        bool swap_y=rf.check("swap_y",Value("false")).asBool();
        if (fakehaptic)
        {
          igeo=new FakeGeomagicDriver();
          Matrix T=zeros(4,4);
          T(0,1)=1.0;
          T(1,2)=1.0;
          T(2,0)=1.0;
          T(3,3)=1.0;
          igeo->setTransformation(SE3inv(T));
          igeo->setCartesianForceMode();
        }
        else
        {  
        if (!initGeomagic(name,geomagic))
          return false;
        }
        if (!initCartesian(robot,swap_x,swap_y))
          return false;
        return true;
    }
    
    bool close()
    {
      if (!fakehaptic)
      {
        closeGeomagic();
      }
      closeCartesian();
      return true;
    }

    double getPeriod()
    {	
        return 0.01;
    }

    bool updateModule()
    { 
        Vector buttons,pos,rpy;
        igeo->getButtons(buttons);
        igeo->getPosition(pos);
        igeo->getOrientation(rpy);

        bool b0=(buttons[0]!=0.0);
        bool b1=(buttons[1]!=0.0);

        xd=pos;      
        // we keep the orientation of the left arm constant:
        // we want the middle finger to point forward (end-effector x-axis)
        // with the palm turned down (end-effector y-axis points leftward);
        // to achieve that it is enough to rotate the root frame of pi around z-axis
        od[0]=0.0; od[1]=0.0; od[2]=1.0; od[3]=M_PI;
        Vector x,o;
        
        mx.lock();
        if (selected_part!=NULL)
        {          
          selected_part->getPose(x,o);
          selected_part->goToPose(xd,od);
          std::cout<< xd[0] << " " << xd [1] << " " << xd [2] << " " << od[0] << " " << od [1] << " "<< od [2] << " "<< od [3]<< " " ;
          std::cout<< x[0] << " " << x [1] << " " << x [2] << " " << o[0] << " " << o [1] << " "<< o [2] << " "<< o [3]<< std::endl ;
        
        }
        mx.unlock();
        return true;
    }
    
    void cbGzSelection(ConstSelectionPtr &_msg)
    // Gazebo callback, called when a new selection is made
    {     
      // TODO: better name, it can be icub_0::l_forearm or similar if robot is removed and replaced in gazebo...
      // for now we only check if something called *::(l|r)_forearm is selected.
      
      std::size_t sep=_msg->name().rfind("::");
      
      if (sep!=string::npos)
      {              
      
        if (_msg->selected() && (!_msg->name().compare(sep+2,string::npos,"l_forearm")))
        {
            mx.lock();
            selected_part=&client_left;
            mx.unlock();
            yInfo("now moving %s",_msg->name().c_str());
        }
        else if (_msg->selected() && (!_msg->name().compare(sep+2,string::npos,"r_forearm")))
        {
            mx.lock();
            selected_part=&client_right;
            mx.unlock();
            yInfo("now moving %s",_msg->name().c_str());
        }
        else
        {
          mx.lock();
          selected_part=nullptr;
          mx.unlock();
        }
      }
    }
};

int main(int argc,char *argv[])
{           
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not found!");
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    Test test;
    yInfo("Trying to connect to gazebo...");
    gazebo::transport::NodePtr node;
    gazebo::client::setup();
    node=gazebo::transport::NodePtr(new gazebo::transport::Node());
    node->Init();
    gazebo::transport::SubscriberPtr sub=node->Subscribe("~/selection",&Test::cbGzSelection,&test);
    
    int r=test.runModule(rf);  
    gazebo::client::shutdown();
    return r;
}
