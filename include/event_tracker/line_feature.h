/* Copyright (C) Ignacio Alzugaray, 2017 <aignacio at student dot ethz dot ch>
 * ETH Zurich - V4RL
 */
#ifndef LINE_FEATURE_H
#define LINE_FEATURE_H

#include <tuple>
#include <bitset>
#include <unordered_set>
#include <iomanip>
//#include "../event_tracker/common.hpp" //#include <event_tracker/common.hpp>
#include "../event_tracker/event_logger.hpp" //#include <event_tracker/event_logger.hpp>

namespace event_tracker {

// Custom typedef
class EventTree;

struct EventData {
  EventData(uint16_t _x, uint16_t _y): x(_x), y(_y){
    ResetAssociation();
    timestamp = INVALID_TIME;
    orientation.clear();
  }

  inline void ResetAssociation() { tree = nullptr;}

  uint16_t x, y;
  EventTime timestamp;
  EventTree* tree;
  OrientationVector orientation;
};

struct FeatureParams {
  // FeatureParams(float _l, std::vector<int>& _pos, std::vector<float>& _dir) {}
  float x,y;
  float angle;
  float l;
  std::tuple<double,double,double> color;
  FeatureParams() : x (-1),y (-1),angle (0),l (0), color (std::tuple<double,double,double>{(double)0,(double)0,(double)0}) {}
}; //ToDo(herbstc): finish this

//template <typename DataType>
class EventNode {
  typedef typename std::list<EventNode*>::iterator ChildIter;
 public:
  EventNode(uint16_t x, uint16_t y);

  inline void push_front(EventNode* node);

  inline EventNode* pull_through();

  inline void Remove();
  inline void RemoveHard();
  inline void ShredTreeDownstream();

  inline void TransferChildrenToNewParent(EventNode* new_parent_node);

  inline ChildIter find_newest_children();


  // Sanity checks
  inline bool is_single_branch_downstream();
  inline size_t get_size_downstream();
  inline bool is_ordered_downstream();
  inline size_t get_depth_downstream();
  inline void get_depth_downstream(size_t& depth_level);
  inline void plot_tree_downstream(size_t depth = 1);
  inline size_t get_num_branch();
  inline void get_num_branch(size_t& num_branch);

  size_t children_list_size() {return children_node_list_.size();}
  void clear_children_list() {this->children_node_list_.clear();}
  EventNode* parent_node() {return parent_node_;}
//  std::list<EventNode*> children_node_list() {return children_node_list_;}

  // Boolean checkers
  inline bool is_tail(){ return children_node_list_.empty();} // TODO(ialzugaray): beaware list::size() is not const complexity in GCC - instead O(N); Hopefully it wont be critical due to the low number of children
  inline bool is_head(){ assert(is_associated()); return parent_node_ == nullptr;}
  inline bool is_isolated(){ return (parent_node_ == nullptr) && children_node_list_.empty();}
  inline bool is_associated() const {
    return data_.tree!=nullptr;
  }
  // Comparators
  inline static bool is_newer(const EventNode* const &lhs,
                              const EventNode* const &rhs) {
    return lhs->data_.timestamp > rhs->data_.timestamp;
  }
  inline static bool is_older(const EventNode* const &lhs,
                              const EventNode* const &rhs) {
    return lhs->data_.timestamp < rhs->data_.timestamp;
  }
  inline static bool is_newer_eq(const EventNode* const &lhs,
                              const EventNode* const &rhs) {
    return lhs->data_.timestamp >= rhs->data_.timestamp;
  }


  // Getters
  EventData data_;

 private:
  EventNode* parent_node_;
  std::list<EventNode*> children_node_list_;
};


typedef uint64_t TreeId;
TreeId IdGenerator = 0; // Convert into static member


class EventTree {
 public:
  EventTree();
  inline void AddEvent(EventNode* const &node);
  inline void RemoveEventData(EventNode* const &node);
  inline void Merge(EventTree* const &merged_tree);

  // Boolean checkers
  inline bool is_top_tree() const {return (this==parent_tree_);}
  inline bool is_removable() {
//    assert (isChildData()); // You may still have orientation
    return size_==0 && children_tree_set_.empty();}

  inline bool isChildData();

  inline void Clear();
  inline void ClearChildData();

  inline void ClearAssociation();

  // This method is public so that top-level managers can decide a better timing
  // for refreshing the data instead of updating it at every change.
  //  inline std::list<EventKey> ForceThinning();
  inline void ForceThinning(const float& max_minor,const size_t& min_node=0);


  // Setters
  inline void SetNewHeadNode(EventNode* const &node) {assert(this->is_top_tree()); this->head_node_ = node; assert(this->head_node_==nullptr || this->head_node_->data_.tree->parent_tree_ == this); }
  inline void set_age(EventTime timestamp) {assert(this->is_top_tree()); age_ = timestamp;}

  // Getters
  inline TreeId id() {return id_;}
  inline std::tuple<double,double,double> color() {return color_;}
  inline size_t size() {return size_;}
  inline EventTime age() const { assert(this->is_top_tree()); return age_;}
  inline OrientationVector& orientation() {return orientation_;}
  inline EventTime newest_timestamp() {assert(head_node_ != nullptr);
                                       assert(is_top_tree());
                                       return head_node_->data_.timestamp;}
  inline EventNode* head_node() {assert(this->is_top_tree()); return head_node_;}
//  inline bool isEmpty() {return ((size()==0 && children_set_.empty()) ? true : false);}
  inline ImageMoment moment() {assert(this->is_top_tree()); return moment_;} 
  inline EventTree* parent_tree() {return parent_tree_;}
  inline const size_t& last_thinning_size() {return last_thinning_size_;}
  inline static bool older_weak_order(const EventTree * const & lhs,
                                 const EventTree * const & rhs) {
    return (lhs->age() <= rhs->age());
  }

private:
  TreeId id_;
  EventTime age_;
  // Node Data related
  size_t size_;
  OrientationVector orientation_;
  // Tree related
  ImageMoment moment_;
  EventTree* parent_tree_;
  std::unordered_set<EventTree*> children_tree_set_;
  EventNode* head_node_;
  size_t last_thinning_size_;



  // Visualization
  std::tuple<double,double,double> color_ ;

  // Methods
  inline void AddEventData(EventNode* const &node);
  inline void AddAngle(const OrientationVector& orientation,
                       const size_t& weight = 1 );
  inline void PushNewHeadNode(EventNode* const &node);
  inline void AddTreeData(const EventTree* const &tree);
  inline void RemoveTreeData(const EventTree* const &tree);
  inline void TransferChildrenToNewParent(EventTree* const &new_parent);
};



} // namespace event_tracker
#endif // LINE_FEATURE_H



