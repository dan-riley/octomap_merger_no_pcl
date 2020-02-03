#ifndef OCTOMAP_MERGER_H_
#define OCTOMAP_MERGER_H_

#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <std_msgs/Float64.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <list>
#include <cmath>
#include "octomap_merger/OctomapArray.h"

using std::cout;
using std::endl;
using namespace octomap;
using namespace octomath;

#define MAXITER 500

double merge_maps(OcTree *tree1, OcTree *tree2);

class OctomapMerger {
  public:
    // Constructor
    OctomapMerger(ros::NodeHandle* nodehandle);
    // Destructor
    ~OctomapMerger();
    // Callbacks
    void callback_myMap(const octomap_msgs::Octomap::ConstPtr& msg);
    void callback_neighborMaps(const octomap_merger::OctomapArrayConstPtr &msg);
    // Public Methods
    void merge();
    // Variables
    bool myMapNew;
    bool otherMapsNew;
    std::string id;
    std::string type;
    int merger;
    int octo_type;
    double resolution;
    int map_thresh;
    std::string map_topic;
    std::string neighbors_topic;
    std::string merged_topic;
    std::string merged_size_topic;

  /* Private Variables and Methods */
  private:
    ros::NodeHandle nh_;

    octomap_msgs::Octomap myMap;
    octomap_merger::OctomapArray neighbors;
    octomap::OcTree *treem;
    octomap::OcTree *tree1;
    octomap::OcTree *tree2;
    double treem_size;
    size_t tree1_last_size;
    std::map<std::string, double> treen_last_size;

    ros::Subscriber sub_mymap;
    ros::Subscriber sub_neighbors;

    ros::Publisher pub_merged;
    ros::Publisher pub_size;

    void initializeSubscribers();
    void initializePublishers();
};

#endif
