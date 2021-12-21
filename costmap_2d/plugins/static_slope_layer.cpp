/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/

#include <costmap_2d/static_slope_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

PLUGINLIB_EXPORT_CLASS(costmap_2d::StaticSlopeLayer, costmap_2d::Layer)

// 255
using costmap_2d::NO_INFORMATION;
// 254
using costmap_2d::LETHAL_OBSTACLE;
// 0
using costmap_2d::FREE_SPACE;

// INSCRIBED_INFLATED_OBSTACLE = 253;

namespace costmap_2d
{

StaticSlopeLayer::StaticSlopeLayer() : dsrv_(NULL) {}

StaticSlopeLayer::~StaticSlopeLayer()
{
  if (dsrv_)
    delete dsrv_;
}

void StaticSlopeLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string map_topic;
  // もらうマップのトピック名
  nh.param("map_topic", map_topic, std::string("point_cloud_map"));
  // trueの場合,マップトピックの最初のメッセージのみをサブスクライブし、後続すべてのメッセージを無視する
  nh.param("first_map_only", first_map_only_, false);
  // trueの場合,map_updatesもサブスクライブする
  nh.param("subscribe_to_updates", subscribe_to_updates_, false);
  // 未観測スペースの扱い方
  // trueの場合, 未観測スペース値はレイヤーに直接変換される falseの場合,free_spaceとして変換される
  // globalで未観測スペースを扱うと、そこに行くパスが生成できないためfalseがよい
  nh.param("track_unknown_space", track_unknown_space_, true);
  // trueの場合、最大値のみがコストマップに書き込まれる（static_layerが最下層のときのみ重要）
  nh.param("use_maximum", use_maximum_, false);

  int temp_lethal_threshold, temp_unknown_cost_value;
  // map_serverからマップを読み込むとき、致命的コストとみなす閾値
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  // map_serverからマップを読み込むとき, 未知スペースとしてみなす値
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
  // trueの場合, すべてのマップメッセージ値をno_infomation,free_space, lethal_obstacleに変換する
  nh.param("trinary_costmap", trinary_costmap_, true);

  //tomikawa add
  if(!nh.getParam("frame_id", frame_id))
    frame_id = "map";
  if(!nh.getParam("width", map_size_x))
    map_size_x = 384;
  if(!nh.getParam("height", map_size_y))
    map_size_y = 384;
  if(!nh.getParam("resolution", map_resolution))
    map_resolution = 0.05;
  if(!nh.getParam("origin_x", origin_x))
    origin_x = -10.0;
  if(!nh.getParam("origin_y", origin_y))
    origin_y = -10.0;

  ROS_INFO("frame_id:=%s", frame_id.c_str());
  ROS_INFO("width:=%3d", map_size_x);
  ROS_INFO("height:=%3d", map_size_y);
  ROS_INFO("resolution:=%3f", map_resolution);
  ROS_INFO("origin_x:=%3f", origin_x);
  ROS_INFO("origin_y:=%3f", origin_y);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;
  // Only resubscribe if topic has changed
  if (point_cloud_map_sub_.getTopic() != ros::names::resolve(map_topic))
  {
    // we'll subscribe to the latched topic that the map server uses
    ROS_INFO("Requesting the map...");
    ROS_INFO("map_topic:=%s", map_topic.c_str());
    point_cloud_map_sub_ = g_nh.subscribe(map_topic, 1, &StaticSlopeLayer::incomingPointCloudMap, this);
    map_received_ = false;
    has_updated_data_ = false;

    ros::Rate r(10);
    while (!map_received_ && g_nh.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());

    if (subscribe_to_updates_)
    {
      ROS_INFO("Subscribing to updates");
      map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &StaticSlopeLayer::incomingUpdate, this);

    }
  }
  else
  {
    has_updated_data_ = true;
  }

  if (dsrv_)
  {
    delete dsrv_;
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &StaticSlopeLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void StaticSlopeLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if (config.enabled != enabled_)
  {
    enabled_ = config.enabled;
    has_updated_data_ = true;
    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
  }
}

void StaticSlopeLayer::matchSize()
{
  // std::cout << "2 matchSize()" << std::endl;
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap
  if (!layered_costmap_->isRolling())
  {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }
}

unsigned char StaticSlopeLayer::interpretValue(unsigned char value)
{
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION; // unknown
  else if (!track_unknown_space_ && value == unknown_cost_value_)
    return FREE_SPACE; // free
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE; // obstacle cell
  else if (trinary_costmap_)
    return FREE_SPACE;

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

double StaticSlopeLayer::translate(unsigned int i, int leftmin, int leftmax, double rightmin, double rightmax)
{
  // Figure out how 'wide' each range is
  double leftSpan, rightSpan, valueScaled;
  leftSpan = leftmax - leftmin;
  rightSpan = rightmax - rightmin;

  // Convert the left range into a 0-1 range (float)
  valueScaled = float(i - leftmin) / float(leftSpan);

  // Convert the 0-1 range into a value in the right range.
  return rightmin + (valueScaled * rightSpan);
}

// ここで作成したマップをもらっている→ここを2.5DMapに変更する
// 斜面コストマップを作成する
// 具体的には,スロープ部分のコスト値が付加されたコストマップを作成する
// void StaticSlopeLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
void StaticSlopeLayer::incomingPointCloudMap(const sensor_msgs::PointCloud2ConstPtr& new_map)
{
  // sensor_msgs::PointCloud2 -> pcl::PointCloud<pcl::PointXYZRGB>
  pcl::PointCloud<pcl::PointXYZRGB> cloud_;
  pcl::fromROSMsg(*new_map, cloud_);

  // tomikawa add
  std::string frame_id_ = frame_id;
  unsigned int size_x = map_size_x;
  unsigned int size_y = map_size_y;
  double resolution = map_resolution;
  double origin_position_x = origin_x;
  double origin_position_y = origin_y;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, resolution);

  // resize costmap if size, resolution or origin do not match
  // layered_costmap = LayeredCostmapのポインタオブジェクト
  Costmap2D* master = layered_costmap_->getCostmap();
  if (!layered_costmap_->isRolling() &&
      (master->getSizeInCellsX() != size_x ||
       master->getSizeInCellsY() != size_y ||
       master->getResolution() != resolution ||
       master->getOriginX() != origin_position_x ||
       master->getOriginY() != origin_position_y))
  {
    // Update the size of the layered costmap (and all layers, including this one)
    // resize of layered_costmap2d.h -> resize of layered_costmap2d.cpp -> resize of costmap2d.h -> resize of costmap2d.cpp
    ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, resolution);
    layered_costmap_->resizeMap(size_x, size_y, resolution, origin_position_x,
                                origin_position_y,
                                true /* set size_locked to true, prevents reconfigureCb from overriding map size*/);
  }
  else if (size_x_ != size_x || size_y_ != size_y ||
           resolution_ != resolution ||
           origin_x_ != origin_position_x ||
           origin_y_ != origin_position_y)
  {
    // only update the size of the costmap stored locally in this layer
    ROS_INFO("Resizing static Slope layer to %d X %d at %f m/pix", size_x, size_y, resolution);
    resizeMap(size_x, size_y, resolution, origin_position_x, origin_position_y);
  }

  unsigned int index = 0;

  unsigned char value; 
  // mapping.pyの方を調整してみること
  // initialize the costmap with static data
  for (unsigned int i = 0; i < size_x; i++)
  {
    for (unsigned int j = 0; j < size_y; j++)
    { 
      // rgb値によって値を変化させる
      // 高さの値によってコスト値を変化させる 0 = 黒 255= 白 155 = 灰色
      if (cloud_.points[MAP_IDX(size_x, i, j)].r == 0 && cloud_.points[MAP_IDX(size_x, i, j)].g == 0 && cloud_.points[MAP_IDX(size_x, i, j)].b == 0 )
        value = 100;
      else if (cloud_.points[MAP_IDX(size_x, i, j)].r == 255 && cloud_.points[MAP_IDX(size_x, i, j)].g == 255 && cloud_.points[MAP_IDX(size_x, i, j)].b == 255)
        value = 0;
      else if(cloud_.points[MAP_IDX(size_x, i, j)].r == 155 && cloud_.points[MAP_IDX(size_x, i, j)].g == 155 && cloud_.points[MAP_IDX(size_x, i, j)].b == 155)    
        value = -1;
      
      // unsigned char value = new_map->data[index];
      // 占有情報をコスト値に変換し,コストマップに付加している costmap_ = Costmap2Dのオブジェクト
      costmap_[index] = interpretValue(value);
      
      // tomikawa add
      // 高さ情報をコスト値に変換し、コストマップに付加する 
      if (cloud_.points[MAP_IDX(size_x, i, j)].z > 0.04 && cloud_.points[MAP_IDX(size_x, i, j)].r == 255 && cloud_.points[MAP_IDX(size_x, i, j)].g == 255 && cloud_.points[MAP_IDX(size_x, i, j)].b == 255)
        costmap_[index] = cloud_.points[MAP_IDX(size_x, i, j)].z  * 750;
      ++index;
    }
  }

  // なぜか90度反対
  // for (unsigned int i = 0; i < cloud_.points.size(); ++i)
  // {
  //   unsigned char value;  
  //   // rgb値によって値を変化させる
  //   // 高さの値によってコスト値を変化させる
  //   if (cloud_.points[i].r == 0 && cloud_.points[i].g == 0 && cloud_.points[i].b == 0 && cloud_.points[i].a == 255)
  //     value = 0;
  //   else if (cloud_.points[i].r == 255 && cloud_.points[i].g == 255 && cloud_.points[i].b == 255 && cloud_.points[i].a == 255)
  //     value = 100;
  //   else if(cloud_.points[i].r == 155 && cloud_.points[i].g == 155 && cloud_.points[i].b == 155 && cloud_.points[i].a == 255)    
  //     value = -1;
    
  //   costmap_[i] = interpretValue(value);
  //   // ++index;
  // }
  
  
  map_frame_ = frame_id_;

  // we have a new map, update full size of map
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;

  // shutdown the map subscrber if firt_map_only_ flag is on
  if (first_map_only_)
  {
    ROS_INFO("Shutting down the map subscriber. first_map_only flag is on");
    map_sub_.shutdown();
  }
}

void StaticSlopeLayer::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update)
{
  unsigned int di = 0;
  for (unsigned int y = 0; y < update->height ; y++)
  {
    unsigned int index_base = (update->y + y) * size_x_;
    for (unsigned int x = 0; x < update->width ; x++)
    {
      unsigned int index = index_base + x + update->x;
      costmap_[index] = interpretValue(update->data[di++]);
    }
  }
  x_ = update->x;
  y_ = update->y;
  width_ = update->width;
  height_ = update->height;
  has_updated_data_ = true;
}

void StaticSlopeLayer::activate()
{
  onInitialize();
}

void StaticSlopeLayer::deactivate()
{
  map_sub_.shutdown();
  if (subscribe_to_updates_)
    map_update_sub_.shutdown();
}

void StaticSlopeLayer::reset()
{
  if (first_map_only_)
  {
    has_updated_data_ = true;
  }
  else
  {
    onInitialize();
  }
}

void StaticSlopeLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{

  if( !layered_costmap_->isRolling() ){
    if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
      return;
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  double wx, wy;

  mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);

  has_updated_data_ = false;
}

void StaticSlopeLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!map_received_)
    return;

  if (!enabled_)
    return;

  if (!layered_costmap_->isRolling())
  {
    // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
    if (!use_maximum_)
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    else
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }
  else
  {
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer
    unsigned int mx, my;
    double wx, wy;
    // Might even be in a different frame
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0));
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
    // Copy map data given proper transformations
    tf2::Transform tf2_transform;
    tf2::convert(transform.transform, tf2_transform);
    for (unsigned int i = min_i; i < max_i; ++i)
    {
      for (unsigned int j = min_j; j < max_j; ++j)
      {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        // Transform from global_frame_ to map_frame_
        tf2::Vector3 p(wx, wy, 0);
        p = tf2_transform*p;
        // Set master_grid with cell from map
        if (worldToMap(p.x(), p.y(), mx, my))
        {
          if (!use_maximum_)
            master_grid.setCost(i, j, getCost(mx, my));
          else
            master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
        }
      }
    }
  }
}

}  // namespace costmap_2d
