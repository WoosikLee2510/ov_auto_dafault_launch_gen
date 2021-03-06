#include <iostream>
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>


using namespace std;

struct OptionInfo
{
    vector<string> type;
    vector<string> name;
    vector<string> value;
    vector<bool> rosparam;
};

OptionInfo op;

void read_file(const string& path, const string& name){
    ifstream inFile (path);
    char oneline[256];

    int count = 0;
    while (inFile)
    {
        inFile.getline(oneline, 256);
        string line(oneline); // convert to string

        std::vector<std::string> parts;
        boost::split(parts, line, boost::is_any_of(" =;{}\""));

        bool save = false;
        for (auto i : parts) {
            if(name == "imu_"){
                if(i == "sigma_w" || i == "sigma_wb"  || i == "sigma_a" || i == "sigma_ab")
                    save = true;

            } else {
                if(i == "bool" || i == "string" || i == "Eigen::Vector3d" || i == "int" || i =="Eigen::VectorXd" ||
                   i == "double" || i == "LandmarkRepresentation::Representation"|| i == "std::map<size_t,bool>" ||
                   i == "std::map<size_t,Eigen::VectorXd>" || i== "std::vector<double>" || i == "Eigen::Matrix<double,3,1>" ||
                   i == "std::map<size_t,std::pair<int,int>>")
                    save = true;
            }
        }
        if(save){
            vector<string> tmp_data;
            for (int i = 0; i < parts.size(); i++) {
                if(parts.at(i).length() != 0){
                    // check if we have space in path
                    if (parts.at(i) == "$(find"){
                        tmp_data.push_back(parts.at(i) + " " + parts.at(i + 1));
                        break;
                    }
                    // special case: LandmarkRepresentation::Representation
                    if (parts.at(i) == "LandmarkRepresentation::Representation"){
                        op.type.push_back("string");
                        op.name.push_back(name + parts.at(i + 1));
                        op.value.push_back("GLOBAL_3D");
                        op.rosparam.push_back(false);
                        count++;
                        break;
                    }

                    // special case: std::map<size_t,bool> fisheye cam
                    if (name == "cam_" && parts.at(i) == "std::map<size_t,bool>" && parts.at(i+1) == "fisheye"){
                        op.type.push_back("bool");
                        op.name.push_back("cam0_is_fisheye");
                        op.value.push_back("true");
                        op.rosparam.push_back(false);
                        count++;
                        op.type.push_back("bool");
                        op.name.push_back("cam1_is_fisheye");
                        op.value.push_back("true");
                        op.rosparam.push_back(false);
                        count++;
                        break;
                    }

                    // special case: std::map<size_t,bool> fisheye cam
                    if (name == "cam_" && parts.at(i) == "std::map<size_t,std::pair<int,int>>" && parts.at(i+1) == "wh"){
                        op.type.push_back("bool");
                        op.name.push_back("cam0_wh");
                        op.value.push_back("[752, 480]");
                        op.rosparam.push_back(true);
                        count++;
                        op.type.push_back("bool");
                        op.name.push_back("cam1_wh");
                        op.value.push_back("[752, 480]");
                        op.rosparam.push_back(true);
                        count++;
                        break;
                    }

                    // special case: std::map<size_t,Eigen::VectorXd> intrinsics cam
                    if (name == "cam_" && parts.at(i) == "std::map<size_t,Eigen::VectorXd>" && parts.at(i+1) == "intrinsics"){
                        op.type.push_back("bool");
                        op.name.push_back("cam0_k");
                        op.value.push_back("[476.4850207717928, 478.0310503180516, 314.5836189313042, 240.16980920673427]");
                        op.rosparam.push_back(true);
                        count++;
                        op.type.push_back("bool");
                        op.name.push_back("cam0_d");
                        op.value.push_back("[-0.03149689493503132, 0.07696336480701078, -0.06608854732019281, 0.019667561645120218]");
                        op.rosparam.push_back(true);
                        count++;
                        op.type.push_back("bool");
                        op.name.push_back("cam1_k");
                        op.value.push_back("[476.4850207717928, 478.0310503180516, 314.5836189313042, 240.16980920673427]");
                        op.rosparam.push_back(true);
                        count++;
                        op.type.push_back("bool");
                        op.name.push_back("cam1_d");
                        op.value.push_back("[-0.03149689493503132, 0.07696336480701078, -0.06608854732019281, 0.019667561645120218]");
                        op.rosparam.push_back(true);
                        count++;
                        break;
                    }

                    // special case: std::map<size_t,Eigen::VectorXd> extrinsics cam
                    if (name == "cam_" && parts.at(i) == "std::map<size_t,Eigen::VectorXd>" && parts.at(i+1) == "extrinsics"){
                        op.type.push_back("bool");
                        op.name.push_back("T_C0toI");
                        op.value.push_back("[1, 0, 0, 0.1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]");
                        op.rosparam.push_back(true);
                        count++;
                        op.type.push_back("bool");
                        op.name.push_back("T_C1toI");
                        op.value.push_back("[1, 0, 0, -0.1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]");
                        op.rosparam.push_back(true);
                        count++;
                        break;
                    }

                    // special case: std::map<size_t,Eigen::VectorXd> cam settings
                    if (name == "cam_" && parts.at(i) == "max_features"){
                        op.type.push_back("int");
                        op.name.push_back(name + "msckf_max_features");
                        op.value.push_back("200");
                        op.rosparam.push_back(false);
                        count++;
                        op.type.push_back("int");
                        op.name.push_back(name + "slam_max_features");
                        op.value.push_back("200");
                        op.rosparam.push_back(false);
                        count++;
                        op.type.push_back("int");
                        op.name.push_back(name + "aruco_max_features");
                        op.value.push_back("200");
                        op.rosparam.push_back(false);
                        count++;
                        break;
                    }

                    // special case: std::map<size_t,Eigen::VectorXd> cam settings
                    if (name == "cam_" && parts.at(i) == "max_in_update"){
                        op.type.push_back("int");
                        op.name.push_back(name + "msckf_max_in_update");
                        op.value.push_back("9999");
                        op.rosparam.push_back(false);
                        count++;
                        op.type.push_back("int");
                        op.name.push_back(name + "slam_max_in_update");
                        op.value.push_back("9999");
                        op.rosparam.push_back(false);
                        count++;
                        op.type.push_back("int");
                        op.name.push_back(name + "aruco_max_in_update");
                        op.value.push_back("9999");
                        op.rosparam.push_back(false);
                        count++;
                        break;
                    }

                    // special case: std::map<size_t,Eigen::VectorXd> cam settings
                    if (name == "cam_" && parts.at(i) == "chi2_multipler"){
                        op.type.push_back("int");
                        op.name.push_back(name + "msckf_chi2_multipler");
                        op.value.push_back("1");
                        op.rosparam.push_back(false);
                        count++;
                        op.type.push_back("int");
                        op.name.push_back(name + "slam_chi2_multipler");
                        op.value.push_back("1");
                        op.rosparam.push_back(false);
                        count++;
                        op.type.push_back("int");
                        op.name.push_back(name + "aruco_chi2_multipler");
                        op.value.push_back("1");
                        op.rosparam.push_back(false);
                        count++;
                        break;
                    }

                    // special case: std::map<size_t,Eigen::VectorXd> cam settings
                    if (name == "cam_" && parts.at(i) == "sigma_pix"){
                        op.type.push_back("int");
                        op.name.push_back(name + "msckf_sigma_pix");
                        op.value.push_back("1");
                        op.rosparam.push_back(false);
                        count++;
                        op.type.push_back("int");
                        op.name.push_back(name + "slam_sigma_pix");
                        op.value.push_back("1");
                        op.rosparam.push_back(false);
                        count++;
                        op.type.push_back("int");
                        op.name.push_back(name + "aruco_sigma_pix");
                        op.value.push_back("1");
                        op.rosparam.push_back(false);
                        count++;
                        break;
                    }

                    // special case: std::map<size_t,Eigen::VectorXd> cam settings
                    if (name == "cam_" && parts.at(i) == "sigma_pix_sq"){
                        op.type.push_back("int");
                        op.name.push_back(name + "msckf_sigma_pix_sq");
                        op.value.push_back("1");
                        op.rosparam.push_back(false);
                        count++;
                        op.type.push_back("int");
                        op.name.push_back(name + "slam_sigma_pix_sq");
                        op.value.push_back("1");
                        op.rosparam.push_back(false);
                        count++;
                        op.type.push_back("int");
                        op.name.push_back(name + "aruco_sigma_pix_sq");
                        op.value.push_back("1");
                        op.rosparam.push_back(false);
                        count++;
                        break;
                    }

                    // special case: std::map<size_t,Eigen::VectorXd> cam settings
                    if (name == "topic_" && parts.at(i) == "Eigen::Matrix<double,3,1>"){
                        op.type.push_back("int");
                        op.name.push_back(name + "trans_gt_pos");
                        op.value.push_back("0.0, 0.0, 0.0");
                        op.rosparam.push_back(true);
                        count++;
                        break;
                    }

                    // special case: Eigen::VectorXd extrinsics lidar
                    if (name == "lidar_" && parts.at(i) == "Eigen::VectorXd" && parts.at(i+1) == "extrinsics"){
                        op.type.push_back("bool");
                        op.name.push_back("T_LtoI");
                        op.value.push_back("[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]");
                        op.rosparam.push_back(true);
                        count++;
                        break;
                    }

                    // special case: Eigen::VectorXd extrinsics relpos
                    if (name == "relpos_" && parts.at(i) == "Eigen::VectorXd" && parts.at(i+1) == "extrinsics"){
                        op.type.push_back("bool");
                        op.name.push_back("T_XtoI");
                        op.value.push_back("[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]");
                        op.rosparam.push_back(true);
                        count++;
                        break;
                    }

                    // special case: Eigen::VectorXd extrinsics wheel
                    if (name == "wheel_" && parts.at(i) == "Eigen::VectorXd" && parts.at(i+1) == "extrinsics"){
                        op.type.push_back("bool");
                        op.name.push_back("T_WtoI");
                        op.value.push_back("[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]");
                        op.rosparam.push_back(true);
                        count++;
                        break;
                    }

                    // special case: std::map<size_t,Eigen::VectorXd> extrinsics gps
                    if (name == "gps_" && parts.at(i) == "Eigen::VectorXd" && parts.at(i+1) == "p_GPS_in_I"){
                        op.type.push_back("bool");
                        op.name.push_back("p_GPS_in_I");
                        op.value.push_back("[0, 0, 0]");
                        op.rosparam.push_back(true);
                        count++;
                        break;
                    }

                    // special case: std::map<size_t,Eigen::VectorXd> extrinsics cam
                    if (name == "sim_" && parts.at(i) == "Eigen::VectorXd" && parts.at(i+1) == "VIOtoENU_trans"){
                        op.type.push_back("bool");
                        op.name.push_back("T_ENUtoVIO");
                        op.value.push_back("[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]");
                        op.rosparam.push_back(true);
                        count++;
                        break;
                    }


                    // normal
                    tmp_data.push_back(parts.at(i));
                }
            }

            if(tmp_data.size() != 3){
//                cout << "check the data: ";
//                for (auto i : tmp_data)  cout << i << " ";
//                cout << endl;
                continue;
            }

            string structname = "";

            op.type.push_back(tmp_data.at(0));
            op.name.push_back(name + structname + tmp_data.at(1));
            op.value.push_back(tmp_data.at(2));
            op.rosparam.push_back(false);
            count++;
        }
    }

    cout << "The number of " << name << "variable: " << count << endl;
    inFile.close();
}

void write_launch_file(string path){
    ofstream f;
    f.open (path, std::ofstream::out | std::ofstream::trunc);

    // pre writing
    f << "<launch>\n\n";
    f <<"\t"<<R"(<arg name="name" default="ov_auto"/>)" << endl;
    f <<"\t"<<R"(<arg name="debug" default="false"/>)" << endl;
    f <<"\t"<<R"(<arg name="run" default="simulation"/>)" << endl;
    f <<"\t"<<R"(<arg name="mute" default="false"/>)" << endl;

    // main 1 writing
    for (int i = 0; i < op.type.size(); i++){
        if(op.type.at(i) == "Eigen::Vector3d" || op.type.at(i) == "std::vector<double>" ){
            f << "\t<arg name=\"" << op.name.at(i) << "\"\tdefault=\"[" << op.value.at(i) << "]\"/>" << endl;
            if(op.type.at(i) == "std::vector<double>"){ // add special case for lidar lidar_elevation_angles
                f << "\t<!--Oster OS1-->\n"
                     "\t<!--<arg name=\"lidar_elevation_angles\"\tdefault=\"[-22.5,-21.8,-21.1,-20.4,-19.7,-19,-18.3,-17.6,-16.9,-16.2,-15.5,-14.8,-14.1,-13.4,-12.7,-12,-11.3,-10.6,-9.9,-9.2,-8.5,-7.8,-7.1,-6.4,-5.7,-5,-4.3,-3.6,-2.9,-2.2,-1.5,-0.8,-0.1,0.6,1.3,2,2.7,3.4,4.1,4.8,5.5,6.2,6.9,7.6,8.3,9,9.7,10.4,11.1,11.8,12.5,13.2,13.9,14.6,15.3,16,16.7,17.4,18.1,18.8,19.5,20.2,20.9,21.6]\"/>-->\n";
                f << "\t<!--Uni deg-->\n"
                     "\t<!--<arg name=\"lidar_elevation_angles\"\tdefault=\"[";
                int num_ang = 60;
                double ang = 180/num_ang;
                for (int j = 0; j < num_ang; j++){
                    f << j * ang -90;
                    if (j != num_ang - 1)
                        f << ",";
                }
                f << "]\"/>-->\n";
            }
        }
        else
            f << "\t<arg name=\"" << op.name.at(i) << "\"\tdefault=\"" << op.value.at(i) << "\"/>" << endl;
    }

    // main 2 writing
    f << endl;
    for (int i = 0; i < op.type.size(); i++){
        if (op.rosparam.at(i) || op.type.at(i) == "Eigen::Vector3d" || op.type.at(i) == "std::vector<double>" )
            f << "\t<rosparam param=\"$(arg name)/" << op.name.at(i) << "\"\tsubst_value=\"true\">$(arg " << op.name.at(i) << ")</rosparam>" << endl;
        else
            f << "\t<param name=\"$(arg name)/" << op.name.at(i) << "\"\ttype=\"" << op.type.at(i) << "\"\tvalue=\"$(arg " << op.name.at(i) << ")\"/>" << endl;
    }

    //ending line
    f << endl;
    f << "\t<group if=\"$(arg debug)\">\n"
         "\t\t<group if=\"$(arg mute)\">\n"
         "\t\t\t<node if=\"$(eval run == 'simulation')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_simulation\" launch-prefix=\"gdb -ex run --args\" required=\"true\" />\n"
         "\t\t\t<node if=\"$(eval run == 'recorder')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_recorder\" launch-prefix=\"gdb -ex run --args\" required=\"true\" />\n"
         "\t\t\t<node if=\"$(eval run == 'serial')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_serial_msckf\" launch-prefix=\"gdb -ex run --args\" required=\"true\" />\n"
         "\t\t\t<node if=\"$(eval run == 'serial_kaist')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_serial_kaist\" launch-prefix=\"gdb -ex run --args\" required=\"true\" />\n"
         "\t\t\t<node if=\"$(eval run == 'subscriber')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_subscribe_msckf\" launch-prefix=\"gdb -ex run --args\" required=\"true\" />\n"
         "\t\t</group>\n"
         "\t\t<group unless=\"$(arg mute)\">\n"
         "\t\t\t<node if=\"$(eval run == 'simulation')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_simulation\" output=\"screen\" launch-prefix=\"gdb -ex run --args\" required=\"true\" />\n"
         "\t\t\t<node if=\"$(eval run == 'recorder')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_recorder\" output=\"screen\" launch-prefix=\"gdb -ex run --args\" required=\"true\" />\n"
         "\t\t\t<node if=\"$(eval run == 'serial')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_serial_msckf\" output=\"screen\" launch-prefix=\"gdb -ex run --args\" required=\"true\" />\n"
         "\t\t\t<node if=\"$(eval run == 'serial_kaist')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_serial_kaist\" output=\"screen\" launch-prefix=\"gdb -ex run --args\" required=\"true\" />\n"
         "\t\t<node if=\"$(eval run == 'subscriber')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_subscribe_msckf\" output=\"screen\" launch-prefix=\"gdb -ex run --args\" required=\"true\" />\n"
         "\t\t</group>\n"
         "\t</group>\n"
         "\t<group unless=\"$(arg debug)\">\n"
         "\t\t<group if=\"$(arg mute)\">\n"
         "\t\t\t<node if=\"$(eval run == 'simulation')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_simulation\" required=\"true\" />\n"
         "\t\t\t<node if=\"$(eval run == 'recorder')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_recorder\" required=\"true\" />\n"
         "\t\t\t<node if=\"$(eval run == 'serial')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_serial_msckf\" required=\"true\" />\n"
         "\t\t\t<node if=\"$(eval run == 'serial_kaist')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_serial_kaist\" required=\"true\" />\n"
         "\t\t\t<node if=\"$(eval run == 'subscriber')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_subscribe_msckf\" required=\"true\" />\n"
         "\t\t</group>\n"
         "\t\t<group unless=\"$(arg mute)\">\n"
         "\t\t\t<node if=\"$(eval run == 'simulation')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_simulation\" output=\"screen\" required=\"true\" />\n"
         "\t\t\t<node if=\"$(eval run == 'recorder')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_recorder\" output=\"screen\" required=\"true\" />\n"
         "\t\t\t<node if=\"$(eval run == 'serial')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_serial_msckf\" output=\"screen\" required=\"true\" />\n"
         "\t\t\t<node if=\"$(eval run == 'serial_kaist')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_serial_kaist\" output=\"screen\" required=\"true\" />\n"
         "\t\t\t<node if=\"$(eval run == 'subscriber')\" name=\"$(arg name)\" pkg=\"ov_auto\" type=\"run_subscribe_msckf\" output=\"screen\" required=\"true\" />\n"
         "\t\t</group>\n"
         "\t</group>\n"
         "</launch>"<< endl;
    f.close();
}
void write_parse_ros_file(string path){
    ofstream f;
    f.open (path, std::ofstream::out | std::ofstream::trunc);

    // pre writing
    f << "/*\n"
         " * OpenVINS: An Open Platform for Visual-Inertial Research\n"
         " * Copyright (C) 2019 Patrick Geneva\n"
         " * Copyright (C) 2019 Guoquan Huang\n"
         " * Copyright (C) 2019 OpenVINS Contributors\n"
         " *\n"
         " * This program is free software: you can redistribute it and/or modify\n"
         " * it under the terms of the GNU General Public License as published by\n"
         " * the Free Software Foundation, either version 3 of the License, or\n"
         " * (at your option) any later version.\n"
         " *\n"
         " * This program is distributed in the hope that it will be useful,\n"
         " * but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
         " * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
         " * GNU General Public License for more details.\n"
         " *\n"
         " * You should have received a copy of the GNU General Public License\n"
         " * along with this program.  If not, see <https://www.gnu.org/licenses/>.\n"
         " */\n"
         "#ifndef OV_AUTO_PARSE_ROSHANDLER_H\n"
         "#define OV_AUTO_PARSE_ROSHANDLER_H\n"
         "\n"
         "\n"
         "#include <ros/ros.h>\n"
         "\n"
         "#include \"option/SystemOptions.h\"\n"
         "#include \"ov_core/types/LandmarkRepresentation.h\"\n"
         "#include \"utils/nh_wrapper.h\"\n"
         "\n"
         "\n"
         "namespace ov_auto {\n"
         "\n"
         "\n"
         "    /**\n"
         "     * @brief This function will load paramters from the ros node handler / paramter server\n"
         "     * This is the recommended way of loading parameters as compared to the command line version.\n"
         "     * @param nh ROS node handler\n"
         "     * @return A fully loaded SystemOptions object\n"
         "     */\n"
         "    SystemOptions parse_ros_nodehandler(ros::NodeHandle &nh) {\n"
         "\n"
         "        // Our vio manager options with defaults\n"
         "        SystemOptions op;\n"
         "        nh_wrapper cnh;\n";
//
//    // main 1 writing
//    for (int i = 0; i < op.type.size(); i++){
//        if(op.type.at(i) == "Eigen::Vector3d" || op.type.at(i) == "std::vector<double>" )
//            f << "\t<arg name=\"" << op.name.at(i) << "\"\tdefault=\"[" << op.value.at(i) << "]\"/>" << endl;
//        else
//            f << "\t<arg name=\"" << op.name.at(i) << "\"\tdefault=\"" << op.value.at(i) << "\"/>" << endl;
//    }
//
//    // main 2 writing
//    f << endl;
//    for (int i = 0; i < op.type.size(); i++){
//        if (op.rosparam.at(i) || op.type.at(i) == "Eigen::Vector3d" || op.type.at(i) == "std::vector<double>" )
//            f << "\t<rosparam param=\"$(arg name)/" << op.name.at(i) << "\"\tsubst_value=\"true\">$(arg " << op.name.at(i) << ")</rosparam>" << endl;
//        else
//            f << "\t<param name=\"$(arg name)/" << op.name.at(i) << "\"\ttype=\"" << op.type.at(i) << "\"\tvalue=\"$(arg " << op.name.at(i) << ")\"/>" << endl;
//    }

    //ending line
    f << endl;
    f << "\n"
         "        // Success, lets returned the parsed options\n"
         "        return op;\n"
         "\n"
         "    }\n"
         "\n"
         "\n"
         "}\n"
         "\n"
         "\n"
         "#endif //OV_AUTO_PARSE_ROSHANDLER_H";
    f.close();
}

int main() {
    string option_path = "/home/wlee/workspace/sara/catkin_ws/src/estimation/odometry/multisensor_odometry/udel_viwo/src";
    string feat_path = "/home/wlee/workspace/sara/catkin_ws/install/include/ov_core/feat/FeatureInitializerOptions.h";
    read_file(option_path + "/option/SystemOptions.h", "");
    read_file(option_path + "/option/StateOptions.h", "state_");
    read_file(option_path + "/option/SimOptions.h", "sim_");
    read_file(option_path + "/state/Propagator.h", "imu_");
    read_file(option_path + "/option/CamOptions.h", "cam_");
    read_file(feat_path, "cam_fi_");
    read_file(option_path + "/option/WheelOptions.h", "wheel_");
    read_file(option_path + "/option/ZeroVelOptions.h", "zero_");
    read_file(option_path + "/option/LidarOptions.h", "lidar_");
    read_file(option_path + "/option/GPSOptions.h", "gps_");
    read_file(option_path + "/option/RelPosOptions.h", "relpos_");
    read_file(option_path + "/option/TopicOptions.h", "topic_");

    write_launch_file("/home/wlee/workspace/sara/catkin_ws/src/estimation/odometry/multisensor_odometry/udel_viwo/launch/default.launch");
    write_parse_ros_file("/home/wlee/workspace/create_ov_auto_default_launch_file/src/create_default_launch_file/src/parse_ros2.cpp");
    return 0;
}