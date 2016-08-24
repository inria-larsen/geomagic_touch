// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later.
 *
 */

#ifndef __FAKE_GEOMAGIC_DRIVER__
#define __FAKE_GEOMAGIC_DRIVER__

#include <yarp/os/Searchable.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include "hapticdevice/IHapticDevice.h"

class FakeGeomagicDriver : public yarp::dev::DeviceDriver,
                       public hapticdevice::IHapticDevice
{
protected:
    bool configured;
    int verbosity;
    std::string name;
    yarp::sig::Matrix T;


public:
    FakeGeomagicDriver();

    // Device Driver
    bool open(yarp::os::Searchable &config);
    bool close();

    // IHapticDevice Interface
    bool getPosition(yarp::sig::Vector &pos);
    bool getOrientation(yarp::sig::Vector &rpy);
    bool getButtons(yarp::sig::Vector &buttons);
    bool isCartesianForceModeEnabled(bool &ret);
    bool setCartesianForceMode(); 
    bool setJointTorqueMode();
    bool getMaxFeedback(yarp::sig::Vector &max);
    bool setFeedback(const yarp::sig::Vector &fdbck);
    bool stopFeedback();
    bool getTransformation(yarp::sig::Matrix &T);
    bool setTransformation(const yarp::sig::Matrix &T);
};

#endif
