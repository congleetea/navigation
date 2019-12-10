/****************************************************************************
 *
 *   Copyright (c) 2018 MuYuan Development Team. All rights reserved.
 *   @author Li Xuancong 
 *   @created Monday, December 9 08:32:47 CST 2019
 *
 ****************************************************************************/

#ifndef COSTMAP_2D_VIRTUAL_LAYER_H_
#define COSTMAP_2D_VIRTUAL_LAYER_H_

#include <ros/ros.h>
#include <vector>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/VirtualWall.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <message_filters/subscriber.h>

namespace costmap_2d
{

class VirtualLayer : public CostmapLayer
{
public:
  VirtualLayer();
  virtual ~VirtualLayer();
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void matchSize();

private:
  /**
   * @brief  Callback to update the costmap's map from the map_server
   * @param new_map The map to put into the costmap. The origin of the new
   * map along with its size will determine what parts of the costmap's
   * static map are overwritten.
   */
  void IncommingVirtualWall(const costmap_2d::VirtualWallConstPtr& new_map);
  void incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update);
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  unsigned char interpretValue(unsigned char value);
  void DrawLine(const costmap_2d::VirtualWall& msg, std::vector<uint32_t>& virtual_points);
  uint32_t GetIndex(uint32_t width, uint32_t mx, uint32_t my);
  
  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string map_frame_;  /// @brief frame that map is located in
  bool subscribe_to_updates_;
  bool map_received_;
  bool has_updated_data_;
  unsigned int x_, y_, width_, height_;
  bool track_unknown_space_;
  bool use_maximum_;
  bool first_map_only_;      ///< @brief Store the first static map and reuse it on reinitializing
  bool trinary_costmap_;
  ros::Subscriber virtual_wall_sub_, map_update_sub_;

  unsigned char lethal_threshold_, unknown_cost_value_;

  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_VIRTUAL_LAYER_H_
