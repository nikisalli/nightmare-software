#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

void Callback(const octomap_msgs::OctomapConstPtr& msg){
    octomap::ColorOcTree* map = NULL;
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    if (tree){
        map = dynamic_cast<octomap::ColorOcTree*>(tree);
        if(!map){
            ROS_ERROR("Wrong octomap type. Use a different display type.");
        }
    } else {
        ROS_ERROR( "Failed to deserialize octree message.");
        return;
    }
    ROS_INFO("leaves: %d", map->getNumLeafNodes());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "bridge");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/rtabmap/octomap_binary", 1000, Callback);
    ros::spin();

    return 0;
}
