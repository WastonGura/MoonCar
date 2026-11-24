#include "mooncar_terrain/grid_overlay.hpp"
#include <gtest/gtest.h>
#include "../src/grid_overlay.cpp"
using mooncar_terrain::mergeRisk;
TEST(RiskOverlay, MissingOrFreeEvidencePreservesHazardsAndUnknown) {
  for (auto cost : {0,80,253,254,255}) {
    EXPECT_EQ(mergeRisk(cost,255),cost);
    EXPECT_EQ(mergeRisk(cost,0),cost);
  }
}
TEST(RiskOverlay, PositiveRiskOverridesUnknownAndNeverClearsHigherRisk) {
  EXPECT_EQ(mergeRisk(255,254),254);
  EXPECT_EQ(mergeRisk(255,80),80);
  EXPECT_EQ(mergeRisk(254,80),254);
  EXPECT_EQ(mergeRisk(80,254),254);
}
class NativeOverlay : public mooncar_terrain::RiskOverlay {
 public:
  NativeOverlay() {
    enabled_=true; map_received_in_update_bounds_=true;
    map_frame_="map";global_frame_="map";
    resizeMap(2,2,.2,1.,2.);
    for (unsigned int x=0;x<2;++x) for (unsigned int y=0;y<2;++y) setCost(x,y,255);
  }
};
TEST(RiskOverlay, NativeGridResamplingPreservesAnotherLayerAndUnknown) {
  NativeOverlay overlay;
  nav2_costmap_2d::Costmap2D master(8,8,.1,.8,1.8,255);
  overlay.setCost(0,0,254);
  master.setCost(4,2,254);  // unknown evidence must preserve this obstacle
  overlay.setCost(0,1,0);  // sparse free evidence must leave unknown space alone
  overlay.updateCosts(master,0,0,8,8);
  EXPECT_EQ(master.getCost(2,2),254);
  EXPECT_EQ(master.getCost(3,3),254);
  EXPECT_EQ(master.getCost(4,2),254);
  EXPECT_EQ(master.getCost(2,4),255);
  EXPECT_EQ(master.getCost(0,0),255);
}
