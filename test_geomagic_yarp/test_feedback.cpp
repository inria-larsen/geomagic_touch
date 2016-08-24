/* 
 * This example shows how to get feedback in the haptic arm
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
    Vector maxFeedback;
    Vector feedback;
    double minForce;
    double maxForce;

    // cartesian
    CartesianClient client;
    Vector xd;
    Vector od;
    
    string part;
    
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
        feedback.resize(3,0.0);
        return true;
    }
    
    void closeGeomagic()
    {
        igeo->stopFeedback();
        igeo->setTransformation(eye(4,4));
        drvGeomagic.close();                
    }
    
    bool initCartesian(const string &robot, const string &part, bool swap_x=false,bool swap_y=false)
    {                        
        client.init(robot,part,swap_x,swap_y);
       
        xd.resize(3);
        od.resize(4);
        return true;
    }
    
    void closeCartesian()
    {    
      client.close();      
    }

public:
    Test(): RFModule()
    {
    }
    
    bool configure(ResourceFinder &rf)
    {    
        string name=rf.check("name",Value("test_feedback")).asString().c_str();
        string robot=rf.check("robot",Value("icubGazeboSim")).asString().c_str();
	string part=rf.check("part",Value("left_arm")).asString().c_str();	
    	
        string geomagic=rf.check("geomagic",Value("geomagic")).asString().c_str();
        fakehaptic=rf.check("fakehaptic",Value("false")).asBool();
        minForce=fabs(rf.check("min-force-feedback",Value(0.01)).asDouble());
        maxForce=fabs(rf.check("max-force-feedback",Value(1.5)).asDouble());
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
        if (!initCartesian(robot,part,swap_x,swap_y))
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
        od[0]=0.0; od[1]=0.0; od[2]=1.0; od[3]=M_PI;
        
        Vector x,o;                
        client.getPose(x,o); // get current pose as x,o
        client.goToPose(xd,od); // new target is xd,od

        feedback=(x-xd)*45.0; // the feedback force will be proportional to the error in position                
                              // TODO make 40.0 a parameter
                               
        for (size_t i=0;i<3;i++)
        {
          if (feedback[i]>maxForce) feedback[i]=maxForce;
          if (feedback[i]<-maxForce) feedback[i]=-maxForce;
          if ((feedback[i]>=0) && (feedback[i]<minForce)) feedback[i]=0.0; // remove small values
          if ((feedback[i]<0) && (feedback[i]>-minForce)) feedback[i]=0.0; // remove small values
        }
        igeo->setFeedback(feedback);
        yInfo("current position = (%s)",x.toString(3,3).c_str());
        yInfo(" target position = (%s)",xd.toString(3,3).c_str());
        yInfo("        feedback = (%s)",feedback.toString(3,3).c_str());
        return true;
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
    int r=test.runModule(rf);  
    
    return r;
}
