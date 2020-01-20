#pragma once
#include "Quaternion.hpp"
#include "MsgReceiver.hpp"
#include "MsgEmitter.hpp"
#include <math.h>
#include "ROSMsg.hpp"
#include "RTKMessagePosition.hpp"
#include "RTKMessageVelocity.hpp"
#include "AttitudeMsg.hpp"
#include "HeadingMsg.hpp"
#include "VelocityMsg.hpp"
#include "Matrix3by3.hpp"
#include "RotationMatrix3by3.hpp"
class EmlidRTK : public msg_receiver , public RotationMatrix3by3 , public msg_emitter {

private:
    const double PI= 3.141592653589793238;
    PositionMsg _bodyPos1,_bodyPos2, _bodyVel1,_bodyVel2;
    const double antenna_height=-0.095;
    Vector3D<double> Attitude;
    RotationMatrix3by3 rotation_matrix;
    PositionMsg _average_position;
    Vector3D<double> New_Attitude;
    Vector3D<double> cartesian_position1;
    Vector3D<double> cartesian_position2;
    Vector3D<double> _diff;
    Vector3D<double> _updated_position;
    VelocityMsg Average_vel;
    

public: 
    void receive_msg_data(DataMessage* t_msg);
    PositionMsg Convert_to_xyz();
    HeadingMsg Calculate_Heading();
    PositionMsg  getPosition();
    HeadingMsg getHeading();
    VelocityMsg AverageVel();
    PositionMsg Transformed();
    PositionMsg Center_pos();
    // float MeasureGPStoMeters(float lat1,float lon1,float lat2,float lon2);
    PositionMsg getRTK1();
    PositionMsg getRTK2();
    Vector3D <double> getAttitude();

    EmlidRTK();
   ~EmlidRTK();
};