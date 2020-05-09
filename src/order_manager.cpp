//
// Created by zeid on 2/27/20.
//

#include "order_manager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <algorithm>
#include <deque>

int disk_count=0;
int OnConveyor;

AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
// AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}
{
    order_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/orders", 10,
            &AriacOrderManager::OrderCallback, this);
}

AriacOrderManager::~AriacOrderManager(){}


void AriacOrderManager::OrderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
    ROS_WARN(">>>>> OrderCallback");
    received_orders_.push_back(*order_msg);
    // CheckOrderUpdate(*order_msg);
}

bool AriacOrderManager::CheckIfPartNeeded(osrf_gear::Product part,std::vector<osrf_gear::Product> product_list) {
    for(auto new_order: product_list) {
        if(part.type == new_order.type) return true;
    }

    return false;
}

bool AriacOrderManager::CanPlaceOnAGV(osrf_gear::Product part, std::deque <osrf_gear::Product> kit) {
    // ROS_INFO_STREAM("Product in hand: "<< part);
    for(auto part_on_agv : kit) {
        // ROS_INFO_STREAM("Comparing with : " << part_on_agv);
        if(part.pose.position.x == part_on_agv.pose.position.x && part.pose.position.y == part_on_agv.pose.position.y &&
            part.pose.position.z == part_on_agv.pose.position.z) {
            ROS_WARN_STREAM("Pose collition found...");
            return false;    
        }
    }
    
    return true;
}

std::vector<osrf_gear::Product> AriacOrderManager::RemovePartFromList(osrf_gear::Product part, std::vector<osrf_gear::Product> product_list) {
    for(auto it = product_list.begin(); it<product_list.end(); it++) {
        if(part.type == it->type) {
            if(part.pose.position.x == it->pose.position.x && part.pose.position.y == it->pose.position.y && 
                part.pose.position.z == it->pose.position.z) {
                // ROS_WARN_STREAM("Removing from the list...");
                std::swap(*it, product_list.back());
                product_list.pop_back();
                return product_list;
            }
        }
    }
}   

osrf_gear::Product AriacOrderManager::GetUpdatedPose(osrf_gear::Product part, std::vector<osrf_gear::Product> product_list) {
    for(auto product : product_list) {
        if(part.type == product.type) return product;
    }
}


bool AriacOrderManager::DiscardPartOnAGV(geometry_msgs::Pose part_pose, int agv_id) {

    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;
    if(agv_id== 2) {
        StampedPose_in.header.frame_id = "/kit_tray_2";
        StampedPose_in.pose = part_pose;
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");
        bool success = arm2_.DiscardPart(StampedPose_out.pose);       
    }
    else {
        StampedPose_in.header.frame_id = "/kit_tray_1";
        StampedPose_in.pose = part_pose;
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");
        bool success = arm1_.DiscardPart(StampedPose_out.pose);
    }
    return true;
}


bool AriacOrderManager::PickPartAtAGV(osrf_gear::Product final_product, geometry_msgs::Pose pickup, int agv_id) {
    geometry_msgs::PoseStamped pick_StampedPose_in,pick_StampedPose_out, place_StampedPose_in, place_StampedPose_out;
    if(agv_id== 2) {
        pick_StampedPose_in.header.frame_id = "/kit_tray_2";
        pick_StampedPose_in.pose = pickup;
        part_tf_listener_.transformPose("/world",pick_StampedPose_in,pick_StampedPose_out);
        ROS_INFO_STREAM("Pick StampedPose_int (" << pick_StampedPose_in.pose.position.x <<","<< pick_StampedPose_in.pose.position.y << "," << pick_StampedPose_in.pose.position.z<<")");
        ROS_INFO_STREAM("Pick StampedPose_out (" << pick_StampedPose_out.pose.position.x <<","<< pick_StampedPose_out.pose.position.y << "," << pick_StampedPose_out.pose.position.z<<")");
        place_StampedPose_in.header.frame_id = "/kit_tray_2";
        place_StampedPose_in.pose = final_product.pose;
        part_tf_listener_.transformPose("/world",place_StampedPose_in,place_StampedPose_out);
        ROS_INFO_STREAM("Place StampedPose_int (" << place_StampedPose_in.pose.position.x <<","<< place_StampedPose_in.pose.position.y << "," << place_StampedPose_in.pose.position.z<<")");
        ROS_INFO_STREAM("Place StampedPose_out (" << place_StampedPose_out.pose.position.x <<","<< place_StampedPose_out.pose.position.y << "," << place_StampedPose_out.pose.position.z<<")");
        arm2_.PickAndPlaceUpdated(pick_StampedPose_out.pose, place_StampedPose_out.pose);
    }
    else {
        pick_StampedPose_in.header.frame_id = "/kit_tray_1";
        pick_StampedPose_in.pose = pickup;
        part_tf_listener_.transformPose("/world",pick_StampedPose_in,pick_StampedPose_out);
        ROS_INFO_STREAM("Pick StampedPose_int (" << pick_StampedPose_in.pose.position.x <<","<< pick_StampedPose_in.pose.position.y << "," << pick_StampedPose_in.pose.position.z<<")");
        ROS_INFO_STREAM("Pick StampedPose_out (" << pick_StampedPose_out.pose.position.x <<","<< pick_StampedPose_out.pose.position.y << "," << pick_StampedPose_out.pose.position.z<<")");
        place_StampedPose_in.header.frame_id = "/kit_tray_1";
        place_StampedPose_in.pose = final_product.pose;
        part_tf_listener_.transformPose("/world",place_StampedPose_in,place_StampedPose_out);
        ROS_INFO_STREAM("Place StampedPose_int (" << place_StampedPose_in.pose.position.x <<","<< place_StampedPose_in.pose.position.y << "," << place_StampedPose_in.pose.position.z<<")");
        ROS_INFO_STREAM("Place StampedPose_out (" << place_StampedPose_out.pose.position.x <<","<< place_StampedPose_out.pose.position.y << "," << place_StampedPose_out.pose.position.z<<")");
        arm1_.PickAndPlaceUpdated(pick_StampedPose_out.pose, place_StampedPose_out.pose);
    }
    return true;

}

void AriacOrderManager::CheckPartDrop(int agv_id, std::vector< std::pair<std::string,geometry_msgs::Pose>> fulfilled_products, std::string product_type) {
    auto order_products = received_orders_[0].shipments[0].products;
    osrf_gear::LogicalCameraImage cam_image_agv;
    if(agv_id==1) cam_image_agv = camera_.GetAGVPartsPose(agv_id);
    else cam_image_agv = camera_.GetAGVPartsPose(agv_id);
    //std::vector<osrf_gear::Product> fulfilled_products;
    

    // for(int i=0; i<=count; i++) {
    //     fulfilled_products.push_back(order_products[i]);
    // }

    auto parts_on_agv = cam_image_agv.models;
    // for(auto x : parts_on_agv) {
    //     ROS_INFO_STREAM("Parts on AGV: " << x );
    // }

    // for(auto x : fulfilled_products) {
        
    //     ROS_INFO_STREAM("Fulfilled products: " <<x.first<<" , "<<x.second<<"\n");
    // }

    for(int i=0; i<parts_on_agv.size(); i++) {
        for(int j=0; j<fulfilled_products.size(); j++) {
            if(fulfilled_products[j].first == parts_on_agv[i].type && parts_on_agv[i].type==product_type) {
                geometry_msgs::PoseStamped StampedPose_in_right,StampedPose_out_right, StampedPose_in_wrong,StampedPose_out_wrong;
                if(agv_id==1) {
                    StampedPose_in_right.header.frame_id = "/kit_tray_1";
                    StampedPose_in_wrong.header.frame_id = "/logical_camera_8_frame";
                }
                else {
                    StampedPose_in_right.header.frame_id = "/kit_tray_2";
                    StampedPose_in_wrong.header.frame_id = "/logical_camera_9_frame";
                }
                StampedPose_in_wrong.pose = parts_on_agv[i].pose;
                StampedPose_out_right.pose = fulfilled_products.back().second;
                //part_tf_listener_.transformPose("/world", StampedPose_in_right, StampedPose_out_right);
                part_tf_listener_.transformPose("/world", StampedPose_in_wrong, StampedPose_out_wrong);
                ROS_WARN_STREAM("Product Type: " << product_type);
                ROS_WARN_STREAM("Part -" << fulfilled_products[j].first  << "- should be at position: " << StampedPose_out_right.pose.position);
                //if (fulfilled_products[j].first=="disk_part"){StampedPose_out_wrong.pose.position.y=-StampedPose_out_wrong.pose.position.y;}
                if(StampedPose_out_right.pose.position.x != StampedPose_out_wrong.pose.position.x || StampedPose_out_right.pose.position.y != StampedPose_out_wrong.pose.position.y) {
                    ROS_WARN_STREAM("Part -" << fulfilled_products[j].first  << "- is at position: \n" << StampedPose_out_wrong.pose);
                    if(agv_id==1) {
                        //ROS_WARN_STREAM("Exec Arm 1 Drop");
                        auto temp_pose = StampedPose_out_right.pose;
                        temp_pose.position.z += 0.2;
                        arm1_.GoToTarget(temp_pose);
                        arm1_.PickPart(StampedPose_out_wrong.pose, fulfilled_products[j].first,OnConveyor=0);
                        StampedPose_out_right.pose.position.z += 0.2;
                        arm1_.DropPart(StampedPose_out_right.pose);
                    }
                    else {
                        //ROS_WARN_STREAM("Exec Arm 2 Drop");
                        auto temp_pose = StampedPose_out_wrong.pose;
                        temp_pose.position.z += 0.2;
                        arm1_.GoToTarget(temp_pose);
                        ros::Duration(1.0).sleep();
                        arm2_.PickPart(StampedPose_out_wrong.pose, fulfilled_products[j].first,OnConveyor=0);
                        ros::Duration(1.0).sleep();
                        StampedPose_out_right.pose.position.z += 0.2;
                        arm2_.DropPart(StampedPose_out_right.pose);    
                    }
                }
            }
        }
    }

}

bool AriacOrderManager::CheckOrderUpdate(int count, int agv_id, std::string conveyor_part) {
    if(received_orders_.size() < 2) return false;

    ROS_WARN_STREAM("Order update found!!");
    ROS_INFO_STREAM("Updating order...");

    auto prev__products = received_orders_[0].shipments[0].products;
    auto updated_products = received_orders_[1].shipments[0].products;

    std::deque <osrf_gear::Product> old_kit;
   
    for(int i=0; i<count;i++){                  // Pushing orders fulfilled in a vector and updating it
        old_kit.push_back(prev__products[i]);
    }

   

    ROS_WARN_STREAM("Updating the kit...");

    osrf_gear::Product temp_product;
    temp_product.pose.position.x = 0.1;
    temp_product.pose.position.y = 0.28;
    temp_product.pose.position.z = 0;
    temp_product.pose.orientation.w = 1;
    osrf_gear::Product part;
    

    while(!old_kit.empty()) {
        part = old_kit.front();
        old_kit.pop_front();
        bool needed = CheckIfPartNeeded(part, updated_products);
        if(!needed) {
            ROS_INFO_STREAM("Part " << part.type << " is not needed, discarding...");
            DiscardPartOnAGV(part.pose, agv_id);  //
        }
        else{
            osrf_gear::Product final_product = GetUpdatedPose(part, updated_products);
            bool can_place = CanPlaceOnAGV(final_product, old_kit); 
            if(can_place) {
                if(part.pose.position.x == final_product.pose.position.x && part.pose.position.y == final_product.pose.position.y 
                    && part.pose.position.z == final_product.pose.position.z) {
                    ROS_INFO_STREAM("Part " << final_product.type << " already at updated position");
                }
                else {
                ROS_INFO_STREAM("Placing part " << final_product.type << " in new position: " << final_product.pose << " from pose :" << part.pose);
                PickPartAtAGV(final_product, part.pose, agv_id); //
                }
                updated_products = RemovePartFromList(final_product, updated_products);
            }
            else {
                ROS_INFO_STREAM("Placing part at temp position " << temp_product.pose.position);
                PickPartAtAGV(temp_product, part.pose, agv_id);
                part.pose.position.x = temp_product.pose.position.x;
                part.pose.position.y = temp_product.pose.position.y;
                part.pose.position.z = temp_product.pose.position.z;
                temp_product.pose.position.x += 0.1;
                old_kit.push_back(part);
            }
        }
    }

    ROS_INFO_STREAM("Parts yet to be fulfiled: ");
    for(auto part: updated_products) {
        ROS_INFO_STREAM("Part " << part);
        std::pair<std::string,geometry_msgs::Pose> product_type_pose (part.type, part.pose);
        PickAndPlace(product_type_pose, agv_id, count, conveyor_part);       
    }


    ros::Duration(2.0).sleep();
    return true;
}

/**
 * @brief Get the product frame for a product type
 * @param product_type
 * @return
 */
std::string AriacOrderManager::GetProductFrame(std::string product_type) {
    //--Grab the last one from the list then remove it
    std::string product_frame;
    //product_frame_list_ = camera_.get_product_frame_list();
    std::cout<<"The content of the list is: \n";
    for (auto p: product_frame_list_){
        for(auto d: p.second){
        std::cout<<d<<'\n';}
    }

    label:
    //if ((!product_frame_list_.empty())&&(product_frame_list_.size()>=5)) {
    if ((!product_frame_list_.empty())&&(product_frame_list_.size()>=5)) {
        product_frame = product_frame_list_[product_type].back();
        product_frame_list_[product_type].pop_back();
        return product_frame;
        
    } 
    else {
        ROS_WARN_STREAM("Waiting for conveyor belt part : " << product_type);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        product_frame_list_ = camera_.get_product_frame_list();
        //ROS_WARN_STREAM("SIZE: " << product_frame_list_.size());
        goto label;
        //this->GetProductFrame(product_type);
        
    }
    
}

bool AriacOrderManager::PickPartExchange(geometry_msgs::Pose part_pose, std::string product_type, std::string pick_arm, std::string place_arm) {
    bool failed_pick;
    geometry_msgs::Pose exchange_pose;
    exchange_pose.position.x = 0.3;
    exchange_pose.position.y = 0;
    exchange_pose.position.z = 1.0;

    if(place_arm == "arm1")    {
        failed_pick = arm2_.PickPart(part_pose, product_type,OnConveyor=0);
        std::vector<double> exchange_joint_pose = {0, 1.5, -1.25, 2.4, 3.6, -1.51, 0};    //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}
        arm2_.SendRobotToJointValues(exchange_joint_pose);
        auto temp_pose = exchange_pose;
        temp_pose.position.z += 0.2;
        arm2_.GoToTarget(temp_pose);
        arm2_.DropPartExchange(exchange_pose);
        exchange_joint_pose = {-0.25, 4.6, -1.25, 2.4, 3.6, -1.51, 0};    //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}
        arm1_.SendRobotToJointValues(exchange_joint_pose);
        exchange_pose.position.z -= 0.05;
        arm1_.GoToTarget(temp_pose);
        failed_pick = arm1_.PickPart(exchange_pose, product_type,OnConveyor=0);
    }
    else {
        failed_pick = arm1_.PickPart(part_pose, product_type,OnConveyor=0);
        std::vector<double> exchange_joint_pose = {0, 4.6, -1.25, 2.4, 3.6, -1.51, 0};    //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}
        arm1_.SendRobotToJointValues(exchange_joint_pose);
        if(product_type=="pulley_part"){exchange_pose.position.z+=0.08;}
        auto temp_pose = exchange_pose;
        temp_pose.position.z += 0.2;
        arm1_.GoToTarget(temp_pose);
        arm1_.DropPartExchange(exchange_pose);
        exchange_joint_pose = {0.25, 1.5, -1.25, 2.4, 3.6, -1.51, 0};    //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}
        arm2_.SendRobotToJointValues(exchange_joint_pose);
        exchange_pose.position.z -= 0.05;
        //ROS_INFO_STREAM("Exchange pose: " << exchange_pose.position.y);
        //ROS_INFO_STREAM("Exchange pose: " << exchange_pose.position.z);
        if(product_type=="pulley_part"){exchange_pose.position.x-=0.04;}
        arm2_.GoToTarget(temp_pose);
        failed_pick = arm2_.PickPart(exchange_pose, product_type,OnConveyor=0);   
    }
    return failed_pick;
}

bool AriacOrderManager::PickPartFlip(geometry_msgs::Pose part_pose, std::string product_type, std::string pick_arm) {
    bool failed_pick;

    if(pick_arm == "arm1") {
        part_pose.orientation.x = 0.707;
        part_pose.orientation.w = 0.707;
        arm1_.DropPartFlipped(part_pose);
    }   
    else {
        part_pose.orientation.x = 0.707;
        part_pose.orientation.w = 0.707;
        arm2_.DropPartFlipped(part_pose);
    } 
}



bool AriacOrderManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id, int order_count, std::string conveyor_part) 
{
    std::string product_type = product_type_pose.first;
    geometry_msgs::Pose conv_pick_pose;
    geometry_msgs::Pose part_pose;

    bool failed_pick;

    geometry_msgs::Pose drop_pose = product_type_pose.second;
    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

    // ROS_WARN_STREAM("Initial Drop Pose >>>> " << drop_pose);

    ROS_WARN_STREAM("Product type >>>> " << product_type);
    std::string product_frame = this->GetProductFrame(product_type);
    ROS_WARN_STREAM("Product frame >>>> " << product_frame);

    if (product_type!=conveyor_part)
    {
        //Verifying the products under the camera
        // ros::spinOnce();
        // ros::Duration(1.0).sleep();
        // product_frame_list_ = camera_.get_product_frame_list();
        // std::cout<<"The content of the camera list is: \n";
        // for (auto p: product_frame_list_)
        // {
        //    for(auto d: p.second){
        //      std::cout<<d<<'\n';}
        // }
        //ROS_WARN_STREAM("Double checking Product type >>>> " << product_type);
        auto part_pose = camera_.GetPartPose("world",product_frame);
        //ROS_WARN_STREAM("Reached Here 1");
        //ROS_WARN_STREAM("Part pose obtained >>>> " << part_pose);

        if(product_type == "pulley_part"){
                part_pose.position.z += 0.08;}
            //--task the robot to pick up this part

            
        if(agv_id==1) 
        {
            if(part_pose.position.y > 0) 
            {

                ROS_WARN_STREAM("Arm 1 picking up the part...");
                if(product_type == "pulley_part") failed_pick = PickPartFlip(part_pose, product_type, "arm1");
                failed_pick = arm1_.PickPart(part_pose,product_type,OnConveyor=0);

            }
            else 
            {
                ROS_WARN_STREAM("Arm 2 picking the part and exchanging....");
                if(product_type == "pulley_part") failed_pick = PickPartFlip(part_pose, product_type, "arm2");
                failed_pick = PickPartExchange(part_pose, product_type, "arm2", "arm1");
            }
        }
        else 
        {
            if((part_pose.position.y < 0)&&(part_pose.position.x < 0.6)) 
            {
                ROS_WARN_STREAM("Arm 2 picking up the part...");
                if(product_type == "pulley_part") failed_pick = PickPartFlip(part_pose, product_type, "arm2");
                failed_pick = arm2_.PickPart(part_pose, product_type,OnConveyor=0);
            }
            else 
            {
                ROS_WARN_STREAM("Arm 1 picking the part and exchanging...");
                if(product_type == "pulley_part") failed_pick = PickPartFlip(part_pose, product_type, "arm1");
                failed_pick = PickPartExchange(part_pose, product_type, "arm1", "arm2");
            }
        }

        if(agv_id==1)
        {
            StampedPose_in.header.frame_id = "/kit_tray_1";
            StampedPose_in.pose = drop_pose;
            //ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
            part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
            StampedPose_out.pose.position.z += 0.1;
            //ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");
            //Dropping the part :
            auto result = arm1_.DropPart(StampedPose_out.pose);
            finished_tray_parts.first = product_type;
            finished_tray_parts.second = StampedPose_out.pose;
            Finished_Tray1_Vect.push_back(finished_tray_parts);
            bool discard_flag = camera_.CheckQualityControl(agv_id, StampedPose_out.pose);
            ROS_WARN_STREAM("Dropped a part on AGV tray!");

            if(result) 
            {
                CheckPartDrop(agv_id, Finished_Tray1_Vect, product_type);
                return !result;
            }

            if(discard_flag) 
            {
                ROS_WARN_STREAM("Bad part, discarding....");
                StampedPose_out.pose.position.z -= 0.1;
                arm1_.DiscardPart(StampedPose_out.pose);
                return discard_flag;
            }
            // std::cout<<"THE COMPLETED TRAY PARTS ARE: "<<"\n";
            // for(auto s: Finished_Tray1_Vect)
            // {
            //     std::cout<<s.first<<" , "<<s.second<<"\n";
            // }
            return result;

            
        }
        else{
            StampedPose_in.header.frame_id = "/kit_tray_2";
            StampedPose_in.pose = drop_pose;
            //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
            part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
            StampedPose_out.pose.position.z += 0.1;
            ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
            //Dropping the part :
            auto result = arm2_.DropPart(StampedPose_out.pose);
            finished_tray_parts.first = product_type;
            finished_tray_parts.second = StampedPose_out.pose;

            Finished_Tray2_Vect.push_back(finished_tray_parts);
            bool discard_flag = camera_.CheckQualityControl(agv_id, StampedPose_out.pose);
            ROS_WARN_STREAM("Dropped a part on AGV tray!");

            if(result) 
            {

                CheckPartDrop(agv_id, Finished_Tray2_Vect, product_type);
                return !result;
            }

            if(discard_flag) 
            {
                ROS_WARN_STREAM("Bad part, discarding....");
                StampedPose_out.pose.position.z -= 0.1;
                arm2_.DiscardPart(StampedPose_out.pose);
                return discard_flag;
            }
            return result;
        }


        

    }

    else 
    {
        //Coneyor Pickup pose for reaching the belt part once the belt part cross the camera on the belt
        conv_pick_pose.position.x=1.21984;
        conv_pick_pose.position.y=2.3718;
        conv_pick_pose.position.z=0.9491;
        
    
        if((product_type == conveyor_part)&&(disk_count!=1))
        {
            //ROS_WARN_STREAM("Reached Here 3");
            failed_pick = arm1_.PickPart(conv_pick_pose, product_type,OnConveyor=1);
            conv_pick_pose.position.y -= 1.0;
            //ROS_WARN_STREAM("Reached Here 4");
            ros::Duration(1.0).sleep();
            failed_pick = arm2_.PickPart(conv_pick_pose, product_type,OnConveyor=1);
            disk_count=1;
            
            //ROS_WARN_STREAM("Reached Here 5");
            arm1_.SendRobotHome();
            StampedPose_in.header.frame_id = "/kit_tray_1";
            // ROS_WARN_STREAM("Initial-1 Drop Pose >>>> " << drop_pose);
            StampedPose_in.pose = drop_pose;
            //ROS_WARN_STREAM("Drop1 Stamp In pose obtained >>>> " << StampedPose_in.pose);
            part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
            StampedPose_out.pose.position.z += 0.1;
            //ROS_WARN_STREAM("Drop Stamp Out pose obtained >>>> " << StampedPose_out.pose);
            auto result = arm1_.DropPart(StampedPose_out.pose);
            finished_tray_parts.first = product_type;
            finished_tray_parts.second = StampedPose_out.pose;
            Finished_Tray1_Vect.push_back(finished_tray_parts);
            bool discard_flag = camera_.CheckQualityControl(agv_id, StampedPose_out.pose);
            ROS_WARN_STREAM("Dropped a part on AGV tray!");

            if(result) 
            {
                ROS_WARN_STREAM("Checking Part Drop Function");
                CheckPartDrop(agv_id, Finished_Tray1_Vect, product_type);
                return !result;
            }

            if(discard_flag) 
            {
                ROS_WARN_STREAM("Bad part, discarding....");
                StampedPose_out.pose.position.z -= 0.1;
                arm1_.DiscardPart(StampedPose_out.pose);
                return discard_flag;
            }


            ROS_WARN_STREAM("Second Conveyor Part Execution");
            arm2_.SendRobotHome();
            StampedPose_in.header.frame_id = "/kit_tray_2";
            // ROS_WARN_STREAM("Initial-2 Drop Pose >>>> " << drop_pose);
            StampedPose_in.pose = drop_pose;
            //ROS_WARN_STREAM("Drop2 Stamp In pose obtained >>>> " << StampedPose_in.pose);
            part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
            StampedPose_out.pose.position.z += 0.1;
            //ros::Duration(1.0).sleep();
            //ROS_WARN_STREAM("Drop Stamp Out pose obtained >>>> " << StampedPose_out.pose);
            result = arm2_.DropPart(StampedPose_out.pose);
            finished_tray_parts.first = product_type;
            finished_tray_parts.second = StampedPose_out.pose;
            //ROS_WARN_STREAM("Stamped World pose"<<StampedPose_out.pose);
            Finished_Tray2_Vect.push_back(finished_tray_parts);
            discard_flag = camera_.CheckQualityControl(agv_id, StampedPose_out.pose);
            ROS_WARN_STREAM("Dropped a part on AGV tray!");
            //ROS_WARN_STREAM("RESULT VALUE: "<<result);
            if(result) 
            {

                ROS_WARN_STREAM("Checking Part Drop Function");
                CheckPartDrop(agv_id=2, Finished_Tray2_Vect, product_type);
                return !result;
            }

            if(discard_flag) 
            {
                ROS_WARN_STREAM("Bad part, discarding....");
                StampedPose_out.pose.position.z -= 0.1;
                arm2_.DiscardPart(StampedPose_out.pose);
                return discard_flag;
            }
            //ros::Duration(1.0).sleep();
            //ROS_WARN_STREAM("Reached Here 7");

            return result;
        }
    }
    
}


void AriacOrderManager::ExecuteOrder() 
{
    ROS_WARN(">>>>>> Executing order...");
 
    std::list<std::string> order_type_list;   //finding the part on belt
    std::list<std::string> camera_type_list;   //finding the part on belt
    //-- used to check if pick and place was successful
    bool pick_n_place_success{false};

    std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts;

    ros::spinOnce();
    ros::Duration(1.0).sleep();
    product_frame_list_ = camera_.get_product_frame_list();
    std::cout<<"The List of camera frame parts : \n";
    for (auto s: product_frame_list_)
    {
        std::cout<<s.first<<'\n';
        camera_type_list.push_back(s.first);
    }
    printf("\n");
    // std::cout<<"The content of the list is: \n";
    // for (auto p: product_frame_list_){
    //     for(auto d: p.second){
    //     std::cout<<d<<'\n';}
    // }
    bool update_check_success{false};
    int count=0;
    std::string order_completed;
    int conv_part_flag=0;
    for (const auto &order:received_orders_)
    {
        // ROS_INFO_STREAM("Here 1");   
        auto order_id = order.order_id;
        auto shipments = order.shipments;
        for (const auto &shipment: shipments)
        {
            auto shipment_type = shipment.shipment_type;
            auto agv = shipment.agv_id.back();//--this returns a char
            //-- if agv is any then we use AGV1, else we convert agv id to int
            //--agv-'0' converts '1' to 1 and '2' to 2
            int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';
            // ROS_INFO_STREAM("Here 2");
            auto products = shipment.products;

            std::list<std::string>::iterator it;
            //extern auto val = products[0].type;
            
            int i=0;
            if(conv_part_flag==0)
            {
                for (const auto &product: products)
                {   
                    //finding the part on belt
                    auto val = product.type;
                    // Fetch the iterator of element with value 'val'
                    it = std::find(camera_type_list.begin(), camera_type_list.end(), product.type);
         
                    // Check if iterator points to end or not
                    if(it != camera_type_list.end())
                    {
                        // It does not point to end, it means element exists in list
                        std::cout<<val <<" exists in bin "<<'\n';
                    }
                    else
                    {
                        // It points to end, it means element does not exists in list
                        std::cout<<val<<" does not exists in bin"<<std::endl;
                        product_type_pose_.first = product.type;
                        product_type_pose_.second = product.pose;
                        conveyor_part = val;
                        std::cout<<"Working on the "<<val<< " order"<<'\n';
                        PickAndPlace(product_type_pose_, agv_id, count, conveyor_part);
                    }
                }
            }
            conv_part_flag = 1; 

            printf("\n");
            ROS_WARN_STREAM("Completed the Belt Order First");
            ROS_WARN_STREAM("Now Executing the Bin Orders");
            printf("\n");
            //executed the part on conveyor belt-----------------------------------------------------
            
            ROS_INFO_STREAM("Order ID: " << order_id);
            ROS_INFO_STREAM("Shipment Type: " << shipment_type);
            ROS_INFO_STREAM("AGV ID: " << agv_id);
            for (const auto &product: products){
                ros::spinOnce();
                // ROS_INFO_STREAM("Here 3");
                if(product.type == conveyor_part){
                    goto Order_done;
                }
                product_type_pose_.first = product.type;
                product_type_pose_.second = product.pose;
                discard_part_jump:    
                pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id, count, conveyor_part);    
                if(pick_n_place_success) {goto discard_part_jump;}
                Order_done:
                count++;
                update_check_success =  CheckOrderUpdate(count, agv_id, conveyor_part);
                if (update_check_success==true){
                    goto update_check_jump;
                }
            }
            //ros::Duration(20.0).sleep();  //Waiting for 20 seconds to check for any new order update
            ros::Duration(1.0).sleep();
            ros::spinOnce();
            update_check_success =  CheckOrderUpdate(count, agv_id, conveyor_part);
            // do {
            //     update_check_success =  CheckOrderUpdate();
            // } 
            // while(!update_check_success);

            update_check_jump:
            int finish=1;
            ros::Duration(1.0).sleep();
            ROS_WARN("KIT COMPLETE");
            SubmitAGV(agv_id);
            ROS_INFO_STREAM("Submitting the AGV to deliver the parts");
            
        }
        //ros::Duration(20.0).sleep();  //Waiting for 20 seconds to check for any new order update
        //SubmitAGV(agv_id);
        //ROS_INFO_STREAM("Working on the second KIT");
    }
}

void AriacOrderManager::SubmitAGV(int num) {
    std::string s = std::to_string(num);
    ros::ServiceClient start_client =
            order_manager_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv"+s);
    if (!start_client.exists()) {
        ROS_INFO("Waiting for the client to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Service started.");
    }

    osrf_gear::AGVControl srv;
    // srv.request.kit_type = "order_0_kit_0";
    start_client.call(srv);

    if (!srv.response.success) {
        ROS_ERROR_STREAM("Service failed!");
    } else
        ROS_INFO("Service succeeded.");
}