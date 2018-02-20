/* Copyright (C) Ignacio Alzugaray, 2017 <aignacio at student dot ethz dot ch>
 * ETH Zurich - V4RL
 */
#ifndef ELISED_H
#define ELISED_H

#include "../event_tracker/line_tracker.hpp" // #include <event_tracker/line_tracker.hpp>
#include "../event_tracker/common.hpp" // #include <event_tracker/common.hpp> // OrientationVector definition
#include <list>

namespace event_tracker {
enum SimilarityStatus {kNotSimilar,kNodeSimilar,kTreeSimilar};

template <uint16_t ROWS, uint16_t COLS>
class Elised : public LineTracker<ROWS,COLS> {
 public:



  Elised();
  inline bool AddEvent(const uint16_t& x, const uint16_t& y,
                       const uint64_t& timestamp, const bool &polarity);

 protected:
  inline void UpdateGlobalBuffer( EventNode* const &event_node);

  inline bool SetEventOrientation(EventNode* const &event_node);

  inline void SimilarNeighbours(EventNode* const &event_node,
        std::list<EventNode*>& candidate_list);

  inline SimilarityStatus isSimilar(const EventNode* const &event_node_root,
                          const EventNode* const &event_node_neigh);

  inline bool MergeCandidates(std::list<EventNode*>& candidate_list);

  inline void Process(EventNode* const &event_node_root);


 private:
  Eigen::Array<EventTime, Eigen::Dynamic, Eigen::Dynamic> time_array_;
  std::list<EventNode*> global_buffer_;

  // Parameters
  //TODO(ialzugaray): this can be straightforwardly templated
  const static int MASK_SIZE = 3;
  Eigen::Array<EventTime, MASK_SIZE, MASK_SIZE> saliency_mask_x_;
  Eigen::Array<EventTime, MASK_SIZE, MASK_SIZE> saliency_mask_y_;

  double_t similarity_threshold_;
  double_t min_candidate_cluster_;
  size_t max_buffer_;
  size_t buffer_size_;
  float ratio_limit_;

};


} // namespace event_tracker
#endif // ELISED_H
