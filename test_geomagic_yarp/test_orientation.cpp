/* 
 * This example is a mix of the teleop-icub sample from the hapticdevice YARP driver, and the
 * advanced motor control sample from icub-tutorial. See the original files for proper copyright infos
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
    PolyDriver         drvGeomagic;
    IHapticDevice     *igeo;
    string part;
    
    // cartesian
    PolyDriver client;
    ICartesianControl *icart;
    int startup_context_id;
    Vector xd;
    Vector od;
    Vector txd; // translation added to the xd vector, part dependent
    
    Vector x0,o0; // initial pose
    
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
    
    void limitTorsoPitch()
    {
        int axis=0; // pitch joint
        double min, max;

        // sometimes it may be helpful to reduce
        // the range of variability of the joints;
        // for example here we don't want the torso
        // to lean out more than 30 degrees forward

        // we keep the lower limit
        icart->getLimits(axis,&min,&max);
        icart->setLimits(axis,min,MAX_TORSO_PITCH);
    }
    
    bool initCartesian(const string &robot,const string & part)
    {                        
        
        Property option("(device cartesiancontrollerclient)");  
        option.put("remote","/"+robot+"/cartesianController/"+part);
        option.put("local","/cartesian_client/"+part);

        if (!client.open(option))
            return false;

        client.view(icart);

        // latch the controller context in order to preserve
        // it after closing the module
        // the context contains the dofs status, the tracking mode,
        // the resting positions, the limits and so on.
        icart->storeContext(&startup_context_id);
        
        // get the torso dofs
        Vector newDof, curDof;
        icart->getDOF(curDof);
        newDof=curDof;
        // enable the torso yaw and pitch
        // disable the torso roll
        newDof[0]=0;
        newDof[1]=0;
        newDof[2]=0;
        // impose some restriction on the torso pitch
        limitTorsoPitch();
        // send the request for dofs reconfiguration
        icart->setDOF(newDof,curDof);

        // print out some info about the controller
        Bottle info;
        icart->getInfo(info);
        fprintf(stdout,"info = %s\n",info.toString().c_str());

        xd.resize(3);
        od.resize(4);
        txd.resize(3);
        if (part=="left_arm")
        {
          txd[0]=-0.3; // x is toward the back of the robot
          txd[1]=-0.1; // y is facing right
          txd[2]=+0.1; // z is towards the head
        }
        else if (part=="right_arm")
        {
          txd[0]=-0.3;
          txd[1]=+0.1;
          txd[2]=+0.1;
        }
        
        icart->getPose(x0,o0);
        return true;
    }
    
    void closeCartesian()
    {    
        // we require an immediate stop
        // before closing the client for safety reason
        icart->stopControl();

        // it's a good rule to restore the controller
        // context as it was before opening the module
        icart->restoreContext(startup_context_id);

        client.close();
    }
    
public:
    bool configure(ResourceFinder &rf)
    {    
        string name=rf.check("name",Value("test_geomagic_yarp")).asString().c_str();
        string robot=rf.check("robot",Value("icubSim")).asString().c_str();     
        string geomagic=rf.check("geomagic",Value("geomagic")).asString().c_str();        
        part=rf.check("part",Value("left_arm")).asString().c_str();
        
        if (!initGeomagic(name,geomagic))
          return false;
        if (!initCartesian(robot,part))
          return false;
        return true;
    }
    
    bool close()
    {
        closeGeomagic();
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
        
        // we keep the orientation of the left arm constant:
        // we want the middle finger to point forward (end-effector x-axis)
        // with the palm turned down (end-effector y-axis points leftward);
        // to achieve that it is enough to rotate the root frame of pi around z-axis
        // od[0]=0.0; od[1]=0.0; od[2]=1.0; od[3]=M_PI;
        
        Vector ax(4,0.0),ay(4,0.0),az(4,0.0);
        ax[0]=1.0; ax[3]=rpy[2];
        ay[1]=1.0; ay[3]=rpy[1]*((part=="left_arm")?1.0:-1.0);
        az[2]=1.0; az[3]=rpy[0]*((part=="left_arm")?1.0:-1.0);
        Matrix Rd=axis2dcm(ax)*axis2dcm(ay)*axis2dcm(az);
        od=dcm2axis(Rd);
        
        icart->goToPose(x0,od);
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
    return test.runModule(rf);
}
