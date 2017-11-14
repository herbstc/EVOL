/* Copyright (C) Ignacio Alzugaray, 2017 <aignacio at student dot ethz dot ch>
 * ETH Zurich - V4RL
 */
#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

#include <cmath>
#include <unordered_map>
#include <stack>
#include <vector>
#include <mutex>
//#include <opencv2/core/core.hpp>

#include "../event_tracker/common.hpp" //#include "event_tracker/common.hpp"
#include "../event_tracker/line_feature.hpp" //#include "event_tracker/line_feature.hpp"


namespace event_tracker {




template <uint16_t ROWS, uint16_t COLS>
class LineTracker {
 public:
  const size_t MAX_CLUSTERS = 20000;

  LineTracker();

  inline bool AddEvent(const uint16_t& x,
                               const uint16_t& y,
                               const EventTime& timestamp,
                               const bool &polarity
                               ); // External interface

  void Render(int delay=0);
  void RenderLines(int delay=0);
  void LogClusters(int delay, std::ofstream& cluster_log);
  void FreeEmptyFeature();
  inline void UpdateLineParams();


  std::stack<EventTree*> feature_stack_;
  std::vector<FeatureParams> active_feature_params_;

 protected:
  std::mutex mutex_feature_stack_;
  Eigen::Array<EventNode*, ROWS, COLS> event_node_array_;

  std::vector<EventTree*> active_feature_; // Active features;
  // cv::Mat cluster_img_; // Visualization / debug
  cv::Mat line_img_;



  inline void AssignEvent(EventNode* const &event_node,
                          EventTree* feature);
  inline void RemoveEvent(EventNode* const &event_node);

  inline EventTree* InitializeFeature();
  inline void FreeFeature(EventTree* const &feature);

//  inline void ForceRemoveEvent(EventInfo<ROWS*COLS>* event_info);




};
} // namesapce event_tracker
#endif // LINE_TRACKER_H
