#pragma once
#include "Quaternion.hpp"
#include "MsgReceiver.hpp"
#include "MsgEmitter.hpp"
#include <math.h>
#include "ROSMsg.hpp"
#include "RTKMessagePosition.hpp"
#include "RTKMessageVelocity.hpp"
#include "AttitudeMsg.hpp"
#include "Matrix3by3.hpp"
#include "RotationMatrix3by3.hpp"


#define PI 3.14159265


class EmlidRTK : public msg_receiver , public RotationMatrix3by3 , public msg_emitter {

private:
    Vector3D <double> _bodyPos1,_bodyPos2, _bodyVel1,_bodyVel2;
    double antenna_height=-0.095;
    Vector3D<double> Attitude;
    RotationMatrix3by3 rotation_matrix;
    Vector3D <double>_average_position;
    Vector3D<double> New_Attitude;
    Vector3D <double> cartesian_position1;
    Vector3D <double> cartesian_position2;
    Vector3D<double> _diff;
    Vector3D<double> _updated_position;
    

public: 
    void receive_msg_data(DataMessage* t_msg);
    Vector3D <double> Convert_to_xyz();
    double Calculate_Heading();
    Vector3D <double> getPosition();
    double getHeading();
    Vector3D<double> Transformed();
    Vector3D <double> Center_pos();
    float MeasureGPStoMeters(float lat1,float lon1,float lat2,float lon2);

    EmlidRTK();
   ~EmlidRTK();
};