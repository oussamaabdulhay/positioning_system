#include "EmlidRTK.hpp"
#include <iostream>

EmlidRTK::EmlidRTK(){
}

EmlidRTK::~EmlidRTK(){}

// float EmlidRTK::MeasureGPStoMeters(float lat1,float lon1,float lat2,float lon2)
// {  
//     float R    = 6378.137; // Radius of earth in km
//     float dLat = lat2*(PI/180) - lat1*(PI/180);
//     float dLon = lon2*(PI/180) - lon1*(PI/180);
//     float a    = sin(dLat/2)*sin(dLat/2) + cos(lat1*PI/180)*cos(lat2*PI/180)*sin(dLon/2)*sin(dLon/2);
//     float c    = 2 * atan2(sqrt(a), sqrt(1-a));
//     float d    = R * c;
//     return d*1000; // meters
// }

PositionMsg EmlidRTK::Convert_to_xyz(){
    
    ///////////////////////////First Method/////////////////////////////////////////////

    //First point:
    // double cosLat1 = cos(_bodyPos1.x * PI / 180.0);
    // double sinLat1 = sin(_bodyPos1.x * PI / 180.0);
    // double cosLon1 = cos(_bodyPos1.y * PI / 180.0);
    // double sinLon1 = sin(_bodyPos1.y * PI / 180.0);
    // double rad = 6378137.0;
    // double f = 1.0 / 298.257224;
    // double C1 = 1.0 / sqrt(cosLat1 * cosLat1 + (1 - f) * (1 - f) * sinLat1 * sinLat1);
    // double S1 = (1.0 - f) * (1.0 - f) * C1;
    // double h1 =0;c
    // cartesian_position1.x = (rad * C1 + h1) * cosLat1 * cosLon1;
    // cartesian_position1.y = (rad * C1 + h1) * cosLat1 * sinLon1;
    // cartesian_position1.z = (rad * S1 + h1) * sinLat1;


    // //Second point:
    // double cosLat2 = cos(_bodyPos2.x * PI / 180.0);
    // double sinLat2 = sin(_bodyPos2.x * PI / 180.0);
    // double cosLon2 = cos(_bodyPos2.y * PI / 180.0);
    // double sinLon2 = sin(_bodyPos2.y * PI / 180.0);
    // double C2 = 1.0 / sqrt(cosLat2 * cosLat2 + (1 - f) * (1 - f) * sinLat2 * sinLat2);
    // double S2 = (1.0 - f) * (1.0 - f) * C2;
    // double h2 = 0;
    // cartesian_position2.x = (rad * C2 + h2) * cosLat2 * cosLon2;
    // cartesian_position2.y = (rad * C2 + h2) * cosLat2 * sinLon2;
    // cartesian_position2.z = (rad * S2 + h2) * sinLat2;
    // return cartesian_position1 ,cartesian_position2;

    ///////////////////////////First Method/////////////////////////////////////////////

    ///////////////////////////Second Method/////////////////////////////////////////////
       double R=6371000.;
        //First point
       cartesian_position1.x = R * cos((_bodyPos1.x)* PI/180.) * cos((_bodyPos1.y)* PI/180.);
       cartesian_position1.y = R * cos((_bodyPos1.x)* PI/180.) * sin((_bodyPos1.y)* PI/180.);
       cartesian_position1.z = R *sin((_bodyPos1.x)* PI/180.);
    //    cout <<"cartesian_position1.x=" << cartesian_position1.x<<"\n";
    //    cout <<"cartesian_position1.y=" << cartesian_position1.y<<"\n";
    //    cout <<"cartesian_position1.z=" << cartesian_position1.z<<"\n";
       //Second Point
       cartesian_position2.x = R * cos((_bodyPos2.x)* PI/180.) * cos((_bodyPos2.y)* PI/180.);
       cartesian_position2.y = R * cos((_bodyPos2.x)* PI/180.) * sin((_bodyPos2.y)* PI/180.);
       cartesian_position2.z = R * sin((_bodyPos2.x)* PI/180.);
    //    cout <<"cartesian_position2.x=" << cartesian_position2.x<<"\n";
    //    cout <<"cartesian_position2.y=" << cartesian_position2.y<<"\n";
    //    cout <<"cartesian_position2.z=" << cartesian_position2.z<<"\n"; 

    ///////////////////////////third Method/////////////////////////////////////////////
    // Vector3D<float> calib_point1, calib_point2;
    // calib_point1.x = 54.3930345;
    // calib_point1.y = 24.4470643;
    // calib_point1.z = 0;

    // calib_point2.x = 54.3969964; 
    // calib_point2.y = 24.4502094;
    // calib_point2.z = 0;

    // Vector3D<float> test_point1, test_point2;
    // test_point1.x = _bodyPos1.x;
    // test_point1.y = _bodyPos1.y;
    // test_point1.z = _bodyPos1.z;

    // test_point2.x = _bodyPos2.x;
    // test_point2.y = _bodyPos2.y;
    // test_point2.z = _bodyPos2.z;

    // Vector3D<float> inertial_vector = calib_point2 - calib_point1;
    // // cout << inertial_vector.x <<endl;
    // // cout << inertial_vector.y <<endl;
    // // cout << inertial_vector.z <<endl;
    
    // double yaw_claib = atan2(inertial_vector.y, inertial_vector.x);
    // Vector3D<float> euler_calib;
    // euler_calib.z = yaw_claib;
    // // cout << yaw_claib <<endl;

    // Vector3D<float> origin = calib_point1;
    // Vector3D<float> calibrated_test_point1 , calibrated_test_point2;
    
    // calibrated_test_point1 = test_point1 - origin;
    // calibrated_test_point2 = test_point2 - origin;
    
    // RotationMatrix3by3 G_I_rot_matrix;
    // // cout <<"==================================="<<endl;
    // // cout << calibrated_test_point.x <<endl;
    // // cout << calibrated_test_point.y <<endl;

    // G_I_rot_matrix.Update(euler_calib);

    // calibrated_test_point1 = G_I_rot_matrix.TransformVector(calibrated_test_point1);
    // calibrated_test_point2 = G_I_rot_matrix.TransformVector(calibrated_test_point2);
    // // cout <<"=================================="<<endl;
    // // cout << calibrated_test_point.x <<endl;
    // // cout << calibrated_test_point.y <<endl;

    // cartesian_position1.x = MeasureGPStoMeters(0,calibrated_test_point1.x,0,0);
    // cartesian_position1.y = MeasureGPStoMeters(0,calibrated_test_point1.y,0,0);
    // cartesian_position1.z = MeasureGPStoMeters(0,calibrated_test_point1.z,0,0);

        
    // cartesian_position2.x = MeasureGPStoMeters(0,calibrated_test_point2.x,0,0);
    // cartesian_position2.y = MeasureGPStoMeters(0,calibrated_test_point2.y,0,0);
    // cartesian_position2.z = MeasureGPStoMeters(0,calibrated_test_point2.z,0,0);
       
}

VelocityMsg EmlidRTK::AverageVel(){
    
    Average_vel.dx=(_bodyVel1.x+_bodyVel2.x)/2;
    Average_vel.dy=(_bodyVel1.y+_bodyVel2.y)/2;
    Average_vel.dz=(_bodyVel1.z+_bodyVel2.z)/2;
    
    this->emit_message((DataMessage*) & Average_vel);
    return Average_vel;
}


HeadingMsg EmlidRTK::getHeading(){
    
    HeadingMsg heading = Calculate_Heading();
    return heading;
}

HeadingMsg EmlidRTK::Calculate_Heading(){
    
    HeadingMsg result;
    Vector3D<double> diff_dual_rtk=cartesian_position2-cartesian_position1;
    result.yaw= atan2(diff_dual_rtk.x,diff_dual_rtk.y) * 180. / PI;
    //double result = atan2 (_updated_position.x,_updated_position.y) * 180 / PI;
    // cout <<"result is =" <<_updated_position.y <<"\n";
    // cout <<"result is =" <<_updated_position.x <<"\n";
    // cout <<"result is =" <<result <<"\n";
    this->emit_message((DataMessage*) & result);
    return result;
}

PositionMsg EmlidRTK::Transformed(){
    cout <<"Attitude.x=" <<Attitude.x << "\n";
    cout <<"Attitude.y=" << Attitude.y<< "\n";
    cout <<"Attitude.z=" << Attitude.z<< "\n";    
    rotation_matrix.Update(Attitude);
    Vector3D <double> V1;
    V1.x=0;
    V1.y=0;
    V1.z=antenna_height;
    Vector3D <double> matrix_transformed = rotation_matrix.TransformVector(V1);
    PositionMsg final_position;
    cout<<"matrix_transformed in x =" <<matrix_transformed.x << "\n";
    cout<<"matrix_transformed in y =" <<matrix_transformed.y << "\n";
    cout<<"matrix_transformed in z =" <<matrix_transformed.z << "\n";
    final_position.x = _average_position.x + matrix_transformed.x ;
    final_position.y = _average_position.y + matrix_transformed.y ;
    final_position.z = _average_position.z + matrix_transformed.z ;
    // cout<<"_average_position in x =" <<_average_position.x << "\n";
    // cout<<"_average_position in y =" <<_average_position.y << "\n";
    // cout<<"_average_position in z =" <<_average_position.z << "\n";
    this->emit_message((DataMessage*) & final_position);  
    return final_position;
         
}

PositionMsg EmlidRTK::getPosition(){

PositionMsg average= Center_pos();
return average;
}

PositionMsg EmlidRTK::getRTK1(){
return _bodyPos1;
}


PositionMsg EmlidRTK::getRTK2(){
return _bodyPos2;
}

Vector3D <double> EmlidRTK::getAttitude(){
return Attitude;
}


PositionMsg EmlidRTK::Center_pos(){
    // _updated_position.x=(_bodyPos1.x-_bodyPos2.x);
    // _updated_position.y=(_bodyPos1.y-_bodyPos2.y);
    // _updated_position.z=(_bodyPos1.z-_bodyPos2.z) - antenna_height;
     Convert_to_xyz();
    _average_position.x=(cartesian_position2.x+cartesian_position1.x)/2.;
    _average_position.y=(cartesian_position2.y+cartesian_position1.y)/2.;
    _average_position.z=(cartesian_position2.z+cartesian_position1.z)/2.;
        // cout <<_updated_position.x << "inside the pos\n";
        // cout <<_updated_position.y<< "inside the pos\n";
        // cout <<_updated_position.z<< "inside the pos\n";
        // cout <<_bodyPos2.x << "inside the function\n";
        // cout <<_bodyPos2.y<< "inside the function\n";
        // cout <<_bodyPos2.z<< "inside the function\n";

    return _average_position;
}

void EmlidRTK::receive_msg_data(DataMessage* t_msg){
    if(t_msg->getType() == msg_type::rtkposition){
        
        RTKMessagePosition* rtk_pos = (RTKMessagePosition*)t_msg;
        
        if (rtk_pos->id==1){
            
        _bodyPos1.x = rtk_pos->position.x;
        _bodyPos1.y= rtk_pos->position.y;      
        _bodyPos1.z= rtk_pos->position.z; 
        
        }

        if (rtk_pos->id==2){
        _bodyPos2.x = rtk_pos->position.x;
        _bodyPos2.y= rtk_pos->position.y;      
        _bodyPos2.z= rtk_pos->position.z; 
        }
    }
    if(t_msg->getType() == msg_type::rtkvelocity){
        RTKMessageVelocity* rtk_vel = (RTKMessageVelocity*)t_msg;
        if (rtk_vel->id=1){
         
        _bodyVel1.x = rtk_vel->velocity.x;
        _bodyVel1.y= rtk_vel->velocity.y;      
        _bodyVel1.z= rtk_vel->velocity.z;
        AverageVel(); 
        }
        
         if (rtk_vel->id=2){
        _bodyVel2.x = rtk_vel->velocity.x;
        _bodyVel2.y= rtk_vel->velocity.y;      
        _bodyVel2.z= rtk_vel->velocity.z;
        AverageVel(); 
        }
    }
    if(t_msg->getType() == msg_type::ATTITUDE){
        AttitudeMsg* attitude_msg = (AttitudeMsg*)t_msg;
        Attitude.x =-attitude_msg->pitch;
        Attitude.y= -attitude_msg->roll;

    }
    if(t_msg->getType() == msg_type::HEADING){
    HeadingMsg* heading_msg = (HeadingMsg*)t_msg;
        Attitude.z=-heading_msg->yaw;        
    }
}
