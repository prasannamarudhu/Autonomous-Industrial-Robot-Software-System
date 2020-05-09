//
// Created by zeid on 2/27/20.
//
#include "sensor.h"


AriacSensorManager::AriacSensorManager():
camera1_part_list{},
camera2_part_list{},
camera3_part_list{},
camera4_part_list{}{
    ROS_INFO_STREAM(">>>>> Subscribing to logical sensors");
    camera_1_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_1", 10,
                                                &AriacSensorManager::LogicalCamera1Callback, this);
    camera_2_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_2", 10,
                                                &AriacSensorManager::LogicalCamera2Callback, this);
    camera_3_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_3", 10,
                                                &AriacSensorManager::LogicalCamera3Callback, this);
    camera_4_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_4", 10,
                                                &AriacSensorManager::LogicalCamera4Callback, this);         // Conveyor belt
    camera_5_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_5", 10,
                                                &AriacSensorManager::LogicalCamera5Callback, this);
    camera_6_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_6", 10,
                                                &AriacSensorManager::LogicalCamera6Callback, this);
    camera_7_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_7", 10,
                                                &AriacSensorManager::LogicalCamera7Callback, this);
    camera_8_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_8", 10,
                                                &AriacSensorManager::LogicalCamera8Callback, this);         // AGV1
    camera_9_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_9", 10,
                                                &AriacSensorManager::LogicalCamera9Callback, this);         // AGV2
    quality_sensor_1_subscriber_ = sensor_nh_.subscribe("/ariac/quality_control_sensor_1", 10,
                                                &AriacSensorManager::QualitySensor1Callback, this);
    quality_sensor_2_subscriber_ = sensor_nh_.subscribe("/ariac/quality_control_sensor_2", 10,
                                                &AriacSensorManager::QualitySensor2Callback, this);

    
    camera1_frame_counter_ = 1;
    camera2_frame_counter_ = 1;
    camera3_frame_counter_ = 1;
    camera4_frame_counter_ = 1;
    camera5_frame_counter_ = 1;
    camera6_frame_counter_ = 1;
    camera7_frame_counter_ = 1;
    camera8_frame_counter_ = 1;
    camera9_frame_counter_ = 1;

    init_ = true;
    init_cam1 = false;
    init_cam2 = false;
    init_cam3 = false;
    init_cam4 = false;
    init_cam5 = false;
    init_cam6 = false;
    init_cam7 = false;
    init_cam8 = false;
    init_cam9 = false;
    

    cam_1_ = false;
    cam_2_ = false;
    cam_3_ = false;
    cam_4_ = false;
    cam_5_ = false;
    cam_6_ = false;
    cam_7_ = false;
    cam_8_ = false;
    cam_9_ = false;

}

AriacSensorManager::~AriacSensorManager() {}


void AriacSensorManager::QualitySensor1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
    ROS_INFO_STREAM_THROTTLE(50,
                             "Quality Control Sensor 1: '" << image_msg->models.size() << "' objects.");


    if (image_msg->models.size() == 0) {
        ROS_INFO_STREAM_THROTTLE(100, "Quality Control Sensor 1 does not see anything");
    }

    quality_current_1_ = *image_msg;
    ROS_INFO_STREAM_THROTTLE(100, "Quality Sensor 1 output:   " << quality_current_1_);
}

void AriacSensorManager::QualitySensor2Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
    ROS_INFO_STREAM_THROTTLE(100,
                             "Quality Control Sensor 2: '" << image_msg->models.size() << "' objects.");

    if (image_msg->models.size() == 0) {
        ROS_INFO_STREAM_THROTTLE(100, "Quality Control Sensor 2 does not see anything");
    }
    quality_current_2_ = *image_msg;
}


bool AriacSensorManager::CheckQualityControl(int camera_id, geometry_msgs::Pose part_pose) {
    osrf_gear::LogicalCameraImage current_parts;
    if(camera_id == 1) current_parts = quality_current_1_;
    else current_parts = quality_current_2_;

    if(current_parts.models.size()==0) {
        ROS_WARN_STREAM("Quality check success, part Quality good..");
        return false;
    }

    if(part_pose.position.x == current_parts.pose.position.x && part_pose.position.y == current_parts.pose.position.y && part_pose.position.z == current_parts.pose.position.z) {
        ROS_WARN_STREAM("Faulty part detected, need to be discarded");
        return true;
    }

}




// void AriacSensorManager::LogicalCamera1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
//     if (init_) return;
//     ROS_INFO_STREAM_THROTTLE(10,
//                              "Logical camera 1: '" << image_msg->models.size() << "' objects.");

//     if (image_msg->models.size() == 0) {
//         ROS_ERROR_STREAM("Logical Camera 1 does not see anything");
//     }

//     current_parts_1_ = *image_msg;
//     this->BuildProductFrames(1);
// //    //--Object which represents a part
// //    AriacPartManager part_manager;
// //    //--Logical camera always represents the first product seen in frame 1
// //
// //    int part_frame{1};
// //
// //    for (auto& msg : image_msg->models) {
// //        geometry_msgs::Pose part_pose;
// //
// //        //--position
// //        part_pose.position.x = msg.pose.position.x;
// //        part_pose.position.y = msg.pose.position.y;
// //        part_pose.position.z = msg.pose.position.z;
// //        //--orientation
// //        part_pose.orientation.x = msg.pose.orientation.x;
// //        part_pose.orientation.y = msg.pose.orientation.y;
// //        part_pose.orientation.z = msg.pose.orientation.z;
// //        part_pose.orientation.w = msg.pose.orientation.w;
// //
// //        part_manager.set_part_frame(part_frame);
// //        part_manager.set_part_type(msg.type);
// //        part_manager.set_part_pose(part_pose);
// //            camera1_part_list.push_back(part_manager);
// //        //--next frame id is incremented by 1
// //        part_frame++;
// //    }
// //
// //    for (auto &part: camera1_part_list)
// //    {
// //        ROS_INFO_STREAM(">>>>> Part type:" << part.get_part_type());
// //        ROS_INFO_STREAM(">>>>> Part Pose x:" << part.get_part_pose().position.x);
// //        ROS_INFO_STREAM(">>>>> Part frame:" << part.get_part_frame());
// //    }
// }
void AriacSensorManager::LogicalCamera1Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_cam1) return;
    ROS_INFO_STREAM_THROTTLE(50,
                             "Logical camera 1: '" << image_msg->models.size() << "' objects.");
    
    // if (image_msg->models.size() == 0)
    //     ROS_ERROR_STREAM("Logical Camera 1 does not see anything");

    current_parts_1_ = *image_msg;
    ROS_INFO_STREAM_THROTTLE(50, "Logical Camera Sensor 1 output:   " << current_parts_2_);
    this->BuildProductFrames(1);
}

void AriacSensorManager::LogicalCamera2Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_cam2) return;
    ROS_INFO_STREAM_THROTTLE(50,
                             "Logical camera 2: '" << image_msg->models.size() << "' objects.");
    
    // if (image_msg->models.size() == 0)
    //     ROS_ERROR_STREAM("Logical Camera 2 does not see anything");

    current_parts_2_ = *image_msg;
    ROS_INFO_STREAM_THROTTLE(50, "Logical Camera Sensor 2 output:   " << current_parts_2_);
    this->BuildProductFrames(2);
}


void AriacSensorManager::LogicalCamera3Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 3: '" << image_msg->models.size() << "' objects.");
    // if (image_msg->models.size() == 0)
    //     ROS_ERROR_STREAM("Logical Camera 3 does not see anything");

    current_parts_3_ = *image_msg;
    this->BuildProductFrames(3);
}

void AriacSensorManager::LogicalCamera4Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_cam4) return;
    ROS_INFO_STREAM_THROTTLE(50, "Logical camera 4: '" << image_msg->models.size() << "' objects.");

    if (image_msg->models.size() == 0){
        ROS_INFO_STREAM_THROTTLE(50, "Logical Camera 4 does not see anything");
        return;}

    current_parts_4_ = *image_msg;
    ROS_INFO_STREAM_THROTTLE(50, "Logical Camera Sensor 4 output:   " << current_parts_4_);
    this->BuildProductFrames(4);
}

void AriacSensorManager::LogicalCamera5Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_cam5) return;
    ROS_INFO_STREAM_THROTTLE(50,
                             "Logical camera 5: '" << image_msg->models.size() << "' objects.");
    
    if (image_msg->models.size() == 0)
        ROS_INFO_STREAM_THROTTLE(50, "Logical Camera 5 does not see anything");

    current_parts_5_ = *image_msg;
    ROS_INFO_STREAM_THROTTLE(50, "Logical Camera Sensor 5 output:   " << current_parts_5_);
    this->BuildProductFrames(5);
}

void AriacSensorManager::LogicalCamera6Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_cam6) return;
    ROS_INFO_STREAM_THROTTLE(50,
                             "Logical camera 6: '" << image_msg->models.size() << "' objects.");
    
    if (image_msg->models.size() == 0)
        ROS_INFO_STREAM_THROTTLE(50, "Logical Camera 6 does not see anything");

    current_parts_6_ = *image_msg;
    ROS_INFO_STREAM_THROTTLE(50, "Logical Camera Sensor 6 output:   " << current_parts_6_);
    this->BuildProductFrames(6);
}

void AriacSensorManager::LogicalCamera7Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_cam7) return;
    ROS_INFO_STREAM_THROTTLE(50, "Logical camera 7: '" << image_msg->models.size() << "' objects.");

    if (image_msg->models.size() == 0){
        ROS_INFO_STREAM_THROTTLE(50, "Logical Camera 7 does not see anything");
        return;}

    current_parts_7_ = *image_msg;
    ROS_INFO_STREAM_THROTTLE(50, "Logical Camera Sensor 7 output:   " << current_parts_7_);
    this->BuildProductFrames(7);
}

void AriacSensorManager::LogicalCamera8Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_cam8) return;
    ROS_INFO_STREAM_THROTTLE(50,
                             "Logical camera 8: '" << image_msg->models.size() << "' objects.");
    
    if (image_msg->models.size() == 0)
        ROS_INFO_STREAM_THROTTLE(50, "Logical Camera 8 does not see anything");

    current_parts_8_ = *image_msg;
    // ROS_INFO_STREAM("Logical Camera Sensor 8 output:   " << current_parts_8_);
    // this->BuildProductFrames(8);
}

void AriacSensorManager::LogicalCamera9Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_cam9) return;
    ROS_INFO_STREAM_THROTTLE(50,
                             "Logical camera 9: '" << image_msg->models.size() << "' objects.");
    
    if (image_msg->models.size() == 0)
        ROS_INFO_STREAM_THROTTLE(50, "Logical Camera 9 does not see anything");

    current_parts_9_ = *image_msg;
    // ROS_INFO_STREAM("Logical Camera Sensor 9 output:   " << current_parts_9_);
    // this->BuildProductFrames(9);
}

osrf_gear::LogicalCameraImage AriacSensorManager::GetAGVPartsPose(int agv_id) {
    if (agv_id==1) return current_parts_8_;
    else return current_parts_9_;
}

void AriacSensorManager::BuildProductFrames(int camera_id){
    if (camera_id == 1) {
        for (auto& msg : current_parts_1_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_1_" + msg.type + "_" +
                                        std::to_string(camera1_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            // if(camera1_frame_counter_< 10)
            //     {camera1_frame_counter_++;}
            camera1_frame_counter_++;
            // std::cout<< "camera frame 1 counter :  "<<camera1_frame_counter_<<'\n';
            
        }
        init_cam1 = true;
    }
    else if (camera_id == 2) {
        for (auto& msg : current_parts_2_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_2_" + msg.type + "_" +
                                        std::to_string(camera2_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            // if(camera2_frame_counter_< 10)
            // 	{camera2_frame_counter_++;}
            camera2_frame_counter_++;
            // std::cout<< "camera frame 2 counter :  "<<camera2_frame_counter_<<'\n';
            
        }
        init_cam2 = true;
    }
    else if (camera_id == 3) {
        for (auto& msg : current_parts_3_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_3_" + msg.type + "_" +
                                        std::to_string(camera3_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            // if(camera3_frame_counter_< 10)
            //     {camera3_frame_counter_++;}
            camera3_frame_counter_++;
            // std::cout<< "camera frame 3 counter :  "<<camera3_frame_counter_<<'\n';
            
        }
        init_cam3 = true;
    }
    else if (camera_id == 5) {
        for (auto& msg : current_parts_5_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_5_" + msg.type + "_" +
                                        std::to_string(camera5_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            // if(camera5_frame_counter_< 10)
                // {camera5_frame_counter_++;}
            camera5_frame_counter_++;
            // std::cout<< "camera frame 5 counter :  "<<camera5_frame_counter_<<'\n';
            
        }
        init_cam5 = true;
    }
    else if (camera_id == 6) {
        for (auto& msg : current_parts_6_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_6_" + msg.type + "_" +
                                        std::to_string(camera6_frame_counter_) + "_frame";
            // std::cout << "Frame name: " << product_frame << std::endl;                                        
            product_frame_list_[msg.type].emplace_back(product_frame);
            // if(camera6_frame_counter_< 10)
            //     {camera6_frame_counter_++;}
            camera6_frame_counter_++;
            // std::cout<< "camera frame 6 counter :  "<<camera6_frame_counter_<<'\n';
        }
        init_cam6 = true;
    }
    else if (camera_id == 7) {
        for (auto& msg : current_parts_7_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_7_" + msg.type + "_" +
                                        std::to_string(camera7_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            // if(camera7_frame_counter_< 10)
            //     {camera7_frame_counter_++;}
            camera7_frame_counter_++;
            // std::cout<< "camera frame 7 counter :  "<<camera7_frame_counter_<<'\n';
        }
        init_cam7 = true;
    }

    else if (camera_id == 4) {
        for (auto& msg : current_parts_4_.models) {

            //--build the frame for each product
            std::string product_frame = "logical_camera_4_" + msg.type + "_" +
                                        std::to_string(camera4_frame_counter_) + "_frame";
            // ROS_INFO_STREAM("Current parts: " << product_frame);
            product_frame_list_[msg.type].emplace_back(product_frame);
            if(camera4_frame_counter_< 3)
                 {camera4_frame_counter_++;
                 }
            if (camera4_frame_counter_==3){init_cam4 = true;}
            //camera4_frame_counter_++;
            // ROS_INFO_STREAM("List size: " << product_frame_list_.size());
            // ROS_INFO_STREAM("Found part on Camera 4...");            
        }
        

    }

    // if (camera_id == 8) {
    //     for(auto& msg : current_parts_8_.models) {

    //     }
    // }
    
}


geometry_msgs::Pose AriacSensorManager::GetPartPose(const std::string& src_frame,
                                        const std::string& target_frame) {
    geometry_msgs::Pose part_pose;

    if (init_) {
        ROS_INFO_STREAM("Getting part pose...");

        camera_tf_listener_.waitForTransform(src_frame, target_frame, ros::Time(0),
                                             ros::Duration(150.0));
        ROS_WARN_STREAM("Flag 1");
        camera_tf_listener_.lookupTransform(src_frame, target_frame, ros::Time(0),
                                            camera_tf_transform_);
        ROS_WARN_STREAM("Flag 2");
        part_pose.position.x = camera_tf_transform_.getOrigin().x();
        part_pose.position.y = camera_tf_transform_.getOrigin().y();
        part_pose.position.z = camera_tf_transform_.getOrigin().z();
        ROS_WARN_STREAM("Flag 3");

    } 
    else {
        ROS_INFO_STREAM("Else...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(1);
        this->BuildProductFrames(2);
        this->BuildProductFrames(3);
        this->BuildProductFrames(4);
        this->BuildProductFrames(5);
        this->BuildProductFrames(6);
        this->BuildProductFrames(7);
        ros::spinOnce();
        ros::Duration(1.0).sleep();

        part_pose = this->GetPartPose(src_frame, target_frame);
    }

    return part_pose;
}

