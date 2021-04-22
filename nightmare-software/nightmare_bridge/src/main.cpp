#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <time.h>

static octomap::ColorOcTree* map = NULL;
static octomap::ColorOcTree* old_map = NULL;

unsigned long micros(){
    struct timeval tp;
    gettimeofday(&tp, NULL);
    return tp.tv_usec;
}

void Callback(const octomap_msgs::OctomapConstPtr& msg){
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);

    // check if tree is valid
    if (tree){
        map = dynamic_cast<octomap::ColorOcTree*>(tree);
        if(!map){
            ROS_ERROR("Wrong octomap type. Use a different display type.");
        }
    } else {
        ROS_ERROR("Failed to deserialize octree message.");
        return;
    }

    unsigned long t = micros();

    int diff = 0;

    if(old_map != NULL){
        for(octomap::ColorOcTree::iterator it = map->begin_leafs(), end=map->end_leafs(); it!= end; ++it) {
            octomap::OcTreeNode* n = old_map->search(it.getKey());
            if (!n){
                diff++;
            }
        }
        for(octomap::ColorOcTree::iterator it = old_map->begin_leafs(), end=old_map->end_leafs(); it!= end; ++it) {
            octomap::OcTreeNode* n = map->search(it.getKey());
            if (!n){
                diff--;
            }
        }
    }
    
    ROS_INFO("total leaves: %d diff: %d time: %fms", map->getNumLeafNodes(), diff, (micros() - t)/1000.0);
    old_map = map;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "bridge");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/rtabmap/octomap_binary", 1000, Callback);
    ros::spin();

    return 0;
}
