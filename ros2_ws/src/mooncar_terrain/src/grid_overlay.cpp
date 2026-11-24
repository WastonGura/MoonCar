#include "mooncar_terrain/grid_overlay.hpp"
#include <nav2_costmap_2d/static_layer.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mooncar_terrain {
class RiskOverlay : public nav2_costmap_2d::StaticLayer {
 public:
  void onInitialize() override {
    StaticLayer::onInitialize();
    // Jazzy StaticLayer reads these at the costmap namespace. Each sparse
    // overlay needs its own interpretation, independently of the SLAM map.
    declareParameter("trinary_costmap", rclcpp::ParameterValue(true));
    declareParameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
    auto node=node_.lock();
    if (!node) throw std::runtime_error("RiskOverlay node unavailable");
    node->get_parameter(name_+".trinary_costmap",trinary_costmap_);
    int threshold=100;
    node->get_parameter(name_+".lethal_cost_threshold",threshold);
    if (threshold<1 || threshold>100) throw std::runtime_error("Invalid risk threshold");
    lethal_threshold_=static_cast<unsigned char>(threshold);
  }
  void updateBounds(double x,double y,double yaw,double *minx,double *miny,double *maxx,double *maxy) override {
    StaticLayer::updateBounds(x,y,yaw,minx,miny,maxx,maxy);
    if (enabled_ && map_received_ && layered_costmap_->isRolling()) {
      // Bounds are in the master frame, even when evidence arrives in map.
      const auto *master=layered_costmap_->getCostmap();
      *minx=std::min(*minx,master->getOriginX());
      *miny=std::min(*miny,master->getOriginY());
      *maxx=std::max(*maxx,master->getOriginX()+master->getSizeInMetersX());
      *maxy=std::max(*maxy,master->getOriginY()+master->getSizeInMetersY());
    }
  }
  void updateCosts(nav2_costmap_2d::Costmap2D &master,int minx,int miny,int maxx,int maxy) override {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
    if (!enabled_ || !map_received_in_update_bounds_) return;
    tf2::Transform transform;
    transform.setIdentity();
    try {
      if (map_frame_!=global_frame_) tf2::fromMsg(tf_->lookupTransform(
          map_frame_,global_frame_,tf2::TimePointZero,transform_tolerance_).transform,transform);
    } catch (const tf2::TransformException &error) {
      current_=false;
      RCLCPP_ERROR(logger_,"RiskOverlay: %s",error.what());
      return;
    }
    for (int x=minx;x<maxx;++x) for (int y=miny;y<maxy;++y) {
      double wx,wy;unsigned int mx,my;
      master.mapToWorld(x,y,wx,wy);
      const auto p=transform*tf2::Vector3(wx,wy,0);
      if (worldToMap(p.x(),p.y(),mx,my)) master.setCost(x,y,mergeRisk(master.getCost(x,y),getCost(mx,my)));
    }
    current_=true;
  }
};
}
PLUGINLIB_EXPORT_CLASS(mooncar_terrain::RiskOverlay,nav2_costmap_2d::Layer)
