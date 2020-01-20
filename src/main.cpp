#include <iostream>
#include <vector>
#include "../include/ROSUnit_RTK1.hpp"
#include "../include/ROSUnit_RTK2.hpp"
#include "../include/EmlidRTK.hpp"
#include "../include/ROSUnit_Xsens.hpp"
#include <fstream>

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "testing_node");
    std::ofstream write_data("/home/osama/test//heading.csv");
    std::ofstream write_data2("/home/osama/test//originalposition.csv");
    std::ofstream write_data3("/home/osama/test//transformed.csv");  
    
    ros::NodeHandle nh;
    
    ros::Rate rate(10);
   

    ROSUnit* myROSRTK1 = new ROSUnit_RTK1(nh);
    ROSUnit* myXsens = new ROSUnit_Xsens(nh);
    ROSUnit* myROSRTK2 = new ROSUnit_RTK2(nh);
    EmlidRTK* test = new EmlidRTK;
    myROSRTK1->add_callback_msg_receiver((msg_receiver*) test);
    myROSRTK2->add_callback_msg_receiver((msg_receiver*) test);
    myXsens->add_callback_msg_receiver((msg_receiver*) test);  // PositionMsg RTK1;
  // PositionMsg RTK2;
  // Vector3D <double> heading;st);

    
  PositionMsg pos;
  PositionMsg final_pos;
  Vector3D <double> heading;
  PositionMsg RTK1;
  PositionMsg RTK2;
  // Vector3D <double> heading;

    while(ros::ok()){

        ros::spinOnce();
           RTK1=test->getRTK1();
           RTK2=test->getRTK2();       
        // write_data2 <<setprecision(12)<< pos.x  << ", ";
        // write_data2 <<setprecision(12)<< pos.y  << ", ";
        // write_data2 <<setprecision(12)<< pos.z  << "\n ";
        // write_data3 <<setprecision(12)<< final_pos.x  << ", ";
        // write_data3 <<setprecision(12)<< final_pos.y  << ", ";
        // write_data3 <<setprecision(12)<< final_pos.z  << "\n ";
        //write_data  <<setprecision(12) <<heading.z << " \n ";
        //write_data2 <<setprecision(12)<< RTK1.x  << ", ";
        //write_data2 <<setprecision(12)<< RTK1.y  << " ,";
        // write_data2 <<setprecision(12)<< RTK1.z  << "\n ";
        // write_data3 <<setprecision(12)<< RTK2.x  << " ,";
        // write_data3 <<setprecision(12)<< RTK2.y  << " ,";
        // write_data3 <<setprecision(12)<< RTK2.z  << "\n ";
        pos = test->getPosition();
        //heading = test->getHeading();
        final_pos=test->Transformed();
        // cout<<setprecision(12)<<"Original_pos.x ="<<pos.x<< "\n";
        // cout<<setprecision(12)<<"Original_pos.y ="<<pos.y<< "\n";
        // cout<<setprecision(12)<<"Original_pos.z ="<<pos.z << "\n";
        // cout<<setprecision(12)<<"transformed_pos.x ="<< final_pos.x << "\n";
        // cout<<setprecision(12)<<"transformed_pos.y ="<<  final_pos.y << "\n";
        // cout<<setprecision(12)<<"transformed_pos.z ="<<  final_pos.x << "\n" ;
        //cout << " Heading=" <<heading.yaw << "\n";
        //write_data <<heading.yaw << "\n ";
        write_data2 <<setprecision(12)<< pos.x  << ", ";
        write_data2 <<setprecision(12)<< pos.y  << ", ";
        write_data2 <<setprecision(12)<< pos.z  << "\n ";
        write_data3 <<setprecision(12)<< final_pos.x  << ", ";
        write_data3 <<setprecision(12)<< final_pos.y  << ", ";
        write_data3 <<setprecision(12)<< final_pos.z  << "\n ";


        rate.sleep();
    }
    
    



}
