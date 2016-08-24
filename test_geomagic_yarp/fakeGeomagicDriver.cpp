// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later.
 *
 */
#include <hapticdevice/IHapticDevice.h>
#include <yarp/os/LogStream.h>
#include <yarp/math/Math.h>
#include <math.h>
#include "fakeGeomagicDriver.h"

#define GEOMAGIC_DRIVER_DEFAULT_NAME    "Default Device"
#define MAX_JOINT_TORQUE_0              350.0
#define MAX_JOINT_TORQUE_1              350.0
#define MAX_JOINT_TORQUE_2              350.0

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

FakeGeomagicDriver::FakeGeomagicDriver() : configured(false), verbosity(0),
                                   name(GEOMAGIC_DRIVER_DEFAULT_NAME),
                                   T(eye(4,4))
{
}

bool FakeGeomagicDriver::open(Searchable &config)
{
    if (!configured)
    {
        verbosity=config.check("verbosity",Value(0)).asInt();
        if (verbosity>0)
            yInfo("*** FakeGeomagic Driver: opened");
        name=config.check("device-id",
                          Value(GEOMAGIC_DRIVER_DEFAULT_NAME)).asString();
        if (verbosity>0)
            yInfo("*** FakeGeomagic Driver: name: %s", name.c_str());

       configured=true;
        return true;
    }
    else
    {
        yError("*** FakeGeomagic Driver: device already opened!");
        return false;
    }
}

bool FakeGeomagicDriver::close()
{
    if (configured)
    {
        configured=false;

        if (verbosity>0)
            yInfo("*** FakeGeomagic Driver: closed");
        return true;
    }
    else
    {
        yError("*** FakeGeomagic Driver: trying to close a device which was not opened!");
        return false;
    }
}

bool FakeGeomagicDriver::getPosition(Vector &pos)
{
  static double c=0.0;
  
  c+=0.01;
  
  pos.resize(4);
  pos[0]=-0.1*sin(c);
  pos[1]=0.1*cos(c);
  pos[2]=0.0;
  pos[3]=1.0;
  pos=T*pos;
  pos.pop_back();

  return true;
}


/*********************************************************************/
bool FakeGeomagicDriver::getOrientation(Vector &rpy)
{
    rpy.resize(3);
    rpy[0]=0;
    rpy[1]=0;
    rpy[2]=0;

    return true;
}


/*********************************************************************/
bool FakeGeomagicDriver::getButtons(Vector &buttons)
{
    buttons.resize(2);
    buttons[0]=0;
    buttons[1]=0;

    return true;
}


/*********************************************************************/
bool FakeGeomagicDriver::isCartesianForceModeEnabled(bool &ret)
{
    ret=true;
    return true;
}


/*********************************************************************/
bool FakeGeomagicDriver::setCartesianForceMode()
{
    //TODO not implemented yet
  return true;
}


/*********************************************************************/
bool FakeGeomagicDriver::setJointTorqueMode()
{
    if (verbosity>0)
        yInfo("*** FakeGeomagic Driver: Joint Torque mode enabled");
    return true;
}


/*********************************************************************/
bool FakeGeomagicDriver::getMaxFeedback(Vector &max)
{
    max.resize(3);
//     if (hDeviceData.m_isForce) {
//         max=maxForceMagnitude;
//     }
//     else {
//         max[0]=MAX_JOINT_TORQUE_0;
//         max[1]=MAX_JOINT_TORQUE_1;
//         max[2]=MAX_JOINT_TORQUE_2;
//     }
    return true;
}


/*********************************************************************/
bool FakeGeomagicDriver::setFeedback(const Vector &fdbck)
{
    if (fdbck.length()!=3)
        return false;
    return true;
}


/*********************************************************************/
bool FakeGeomagicDriver::stopFeedback()
{
  return true;
}


/*********************************************************************/
bool FakeGeomagicDriver::setTransformation(const Matrix &T)
{
    if ((T.rows()<this->T.rows()) || (T.cols()<this->T.cols()))
    {
        yError("*** FakeGeomagic Driver: requested to use the unsuitable transformation matrix %s",
               T.toString(5,5).c_str());
        return false;
    }

    this->T=T.submatrix(0,this->T.rows()-1,0,this->T.cols()-1);
    if (verbosity>0)
        yInfo("*** FakeGeomagic Driver: transformation matrix set to %s",
              this->T.toString(5,5).c_str());

    return true;
}


/*********************************************************************/
bool FakeGeomagicDriver::getTransformation(Matrix &T)
{
    T=this->T;
    return true;
}


// /*********************************************************************/
// bool FakeGeomagicDriver::getData()
// {
//     HDErrorInfo error;
// 
//     // Perform a synchronous call to copy the most current device state.
//     // This synchronous scheduler call ensures that the device state
//     // is obtained in a thread-safe manner.
//     hdScheduleSynchronous(copyDeviceDataCallback, this,
//                           HD_MIN_SCHEDULER_PRIORITY);
//     if (HD_DEVICE_ERROR(error = hdGetError())) {
//         yError("*** FakeGeomagic Driver: failed to copy device data (%s)",
//                hdGetErrorString(error.errorCode));
//         return false;
//     }
// 
//     return true;
// }


/*********************************************************************/
// bool FakeGeomagicDriver::setData()
// {
//     HDErrorInfo error;
// 
//     // Perform a synchronous call to copy the motor data force.
//     // This synchronous scheduler call ensures that the device update
//     // is done in a thread-safe manner.
//     hdScheduleSynchronous(updateMotorForceDataCallback, this,
//                           HD_MIN_SCHEDULER_PRIORITY);
//     if (HD_DEVICE_ERROR(error = hdGetError())) {
//         yError("*** FakeGeomagic Driver: failed to update force device data (%s)",
//                hdGetErrorString(error.errorCode));
//         return false;
//     }
// 
//     return true;
// }


/*********************************************************************/
// HDdouble FakeGeomagicDriver::sat(HDdouble value, HDdouble max)
// {
//     if (value>max)
//         value=max;
//     else if (value<-max)
//         value=-max;
//     return value;
// }
