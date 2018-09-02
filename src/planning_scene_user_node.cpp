#include <ros/ros.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>


/**
 * @class PlannigSceneUserNode
 *
 * A node with a planning scene monitor that prints out infos about the stored
 * planning scene when a service is
 * triggered.
 */
class PlannigSceneUserNode {
public:
  bool init();

protected:
  boost::shared_ptr<tf::TransformListener> _transformListenerPtr;
  planning_scene_monitor::PlanningSceneMonitorPtr _planningSceneMonitorPtr;
};

bool PlannigSceneUserNode::init() {

  ros::NodeHandle nh("~");

  _transformListenerPtr = boost::make_shared<tf::TransformListener>(ros::Duration(10.0));
  _planningSceneMonitorPtr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      "robot_description", _transformListenerPtr);

  // guard against empty planning scene
  if (_planningSceneMonitorPtr->getRobotDescription().empty()) {
    return false;
  }

  // initialize monitor
  _planningSceneMonitorPtr->startSceneMonitor();
  _planningSceneMonitorPtr->startWorldGeometryMonitor();
  _planningSceneMonitorPtr->startStateMonitor();

  // initialize publishing of monitor state
  _planningSceneMonitorPtr->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
    "/monitored_planning_scene");

  return true;
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "palnning scene user");
  PlannigSceneUserNode node;
  if (!node.init()) {
    return EXIT_FAILURE;
  }

  ros::spin();
}
