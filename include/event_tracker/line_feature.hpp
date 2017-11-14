/* Copyright (C) Ignacio Alzugaray, 2017 <aignacio at student dot ethz dot ch>
 * ETH Zurich - V4RL
 */
#ifndef LINE_FEATURE_HPP
#define LINE_FEATURE_HPP

#include <algorithm>
#include "../event_tracker/line_feature.h" //#include <event_tracker/line_feature.h>

namespace event_tracker {

// =============== Node =====================
EventNode::EventNode(uint16_t x, uint16_t y) : data_(x,y), parent_node_(nullptr) {
  children_node_list_.clear(); // Todo(ialzugaray): this should be unncessary
}

inline void EventNode::push_front(EventNode* node){
  //assert(node->is_isolated()); // It should be cleared beforehand // Not. We can pushback nodes with children and therefore no isolated
  assert(node->is_head()); // It should be head of any tree
  assert(this->is_head()); // Only can be pushed in front of head nodes
  assert(is_newer_eq(node, this)); // The tree maintain the order when setting a new head (Or equal)
  node->children_node_list_.push_front(this);
  this->parent_node_ = node;
}

inline EventNode* EventNode::pull_through() {
  /*Returns newest child node of this node*/
  assert(this->is_associated()); // This operation only applies to associated nodes
  if(is_tail()) {
    return nullptr;
  } else {
    if (this->children_node_list_.size()==1) { // Todo(ialzugaray): this may be more expensive and not a shortcut if size() is not const complexity (as in GCC C++11)
      return this->children_node_list_.front();
    } else {
      // Pull newest child as a new independent node
      ChildIter newest_child_it = find_newest_children();
      EventNode* newest_child = *newest_child_it;
      // Temporary remove connection current -> newest_child (Allow block move)
      this->children_node_list_.erase(newest_child_it); // Todo(ialzugaray):delete and then insert is not a wise movement
      // Transfer current's children to newest child;
      this->TransferChildrenToNewParent(newest_child);
      // Establish back connection current-> newest_child
      this->children_node_list_.push_back(newest_child);
//      newest_child->parent_node = this; // This is already set
      return newest_child;
    }

  }
}


inline void EventNode::Remove() {
  // Remove node from tree and amend parent - children connection
  assert(this->is_associated()); // This operation only applies to associated nodes
  bool tail = is_tail(), head = is_head();

  if (!head) {
    // Remove this node from parent's children list (parent->current)
    ChildIter find_it = std::find(parent_node_->children_node_list_.begin(),
                                  parent_node_->children_node_list_.end(),
                                  this); // Find to break as soon as it finds it -> it should be unique
    assert(find_it != parent_node_->children_node_list_.end()); // The element must exist in the list
    parent_node_->children_node_list_.erase(find_it);

    if (!tail) {// We need to transfer currents children to parent childen (Remove current -> childrent, Add parent->children)
      this->TransferChildrenToNewParent(this->parent_node_);
    }
    // Remove parent <- current
    this->parent_node_ = nullptr;

  }else if (head && !tail) { // Todo(ialzugaray): Most probably fuse into if (head)
    EventNode* newest_child =  this->pull_through(); // Todo(ialzugaray): pull_through establish connection current->new_node which we have to remove now, maybe consider especific function
    assert(this->children_node_list_.size() == 1); // Only newest child
    assert(this->children_node_list_.front() == newest_child); // Only newest child
    this->children_node_list_.clear();
    assert(newest_child->parent_node_ == this); // This newest child is supposed to point to this node so far
    newest_child->parent_node_ = nullptr;

    this->data_.tree->parent_tree()->SetNewHeadNode(newest_child);
    assert(newest_child->is_head());

  }else if (head && tail ) { // equivalent to is_isolated() -> This is, is the only node in the tree
    assert(this->children_node_list_.empty());
    this->data_.tree->parent_tree()->SetNewHeadNode(nullptr);
  } else {
    assert(0);
  }
  this->data_.tree->RemoveEventData(this);
  this->data_.orientation.clear();
  this->data_.ResetAssociation();
  assert(this->is_isolated());
  assert(!this->is_associated());
  return;
}

inline void EventNode::RemoveHard()  {
  // Destroy node without fixing connections in the tree -> complete destruction of connectivity
  this->parent_node_ = nullptr;
  this->data_.tree->RemoveEventData(this); // Todo(ialzugaray): this is inneficient as each node has to access memory
  this->data_.ResetAssociation();
  this->data_.orientation.clear();
  this->children_node_list_.clear();
}

inline void EventNode::ShredTreeDownstream() {
  for (auto& child_node : this->children_node_list_) {
    child_node->ShredTreeDownstream();
  }
  this->RemoveHard();
}

inline void EventNode::TransferChildrenToNewParent(EventNode* new_parent_node) {
  for (auto& child_node : this->children_node_list_) {
    assert(child_node->parent_node_==this);// It must currently point to the current node
    child_node->parent_node_= new_parent_node; // Baypass into the new parent
  }
  // Transfer the childrens from current to parent node
  new_parent_node->children_node_list_.splice(
        new_parent_node->children_node_list_.begin(),
        this->children_node_list_);
  assert(this->children_node_list_.size()==0); // It should have emptied the list
}

inline EventNode::ChildIter EventNode::find_newest_children() {
  return  std::min_element(children_node_list_.begin(),
                           children_node_list_.end(),
                           is_newer);
}


inline bool EventNode::is_single_branch_downstream() {
  if (this->is_tail()){
    return true;
  } else if (this->children_node_list_.size() == 1) {
    return this->children_node_list_.front()->is_single_branch_downstream();
  } else {
    return false;
  }
}

inline size_t EventNode::get_size_downstream() {
  size_t acc_size = 1;
  for (EventNode* child_node : this->children_node_list_) {
    acc_size += child_node->get_size_downstream();
  }
  return acc_size;
}


inline bool EventNode::is_ordered_downstream() {
  for(EventNode* child_node : this->children_node_list_) {
    if(!(is_newer_eq(this, child_node) && child_node->is_ordered_downstream())){
      return false;
    }
  }
  return true;
}

inline size_t EventNode::get_depth_downstream() {
  size_t depth_level = 0;
  get_depth_downstream(depth_level);
  return depth_level;
}

inline void EventNode::get_depth_downstream(size_t& depth_level) {
  for(EventNode* child_node : this->children_node_list_) {
    child_node->get_depth_downstream(depth_level);
  }
  if (this->children_node_list_.size()>1) {
    depth_level++;
  }
}

inline void EventNode::plot_tree_downstream(size_t depth) {
  int char_per_level = 3;
  std::string tab_level(char_per_level*(depth-1),' ');
  std::string child_symbol = "|-";
  tab_level = tab_level.append(child_symbol);
  std::cout << std::setfill('0') << std::setw(2) <<this->data_.tree->id() <<"-";

  std::list<EventNode*> children_node_list_ordered;

  int i_branch=0;
  while (children_node_list_ordered.size() != this->children_node_list_.size()) {
    for (EventNode* child_node : this->children_node_list_) {
      if(child_node->get_num_branch() == i_branch) {
        children_node_list_ordered.push_back(child_node);
      }
    }
    i_branch++;
  }


  bool is_first_child = true;
  for(EventNode* child_node :children_node_list_ordered) {
    if (is_first_child) {
      is_first_child = false;
    } else {
      std::cout << tab_level ;
    }
     child_node->plot_tree_downstream(depth+1);
  }
  if (children_node_list_ordered.empty()) {
    std::cout << std::endl;
  }
}

inline size_t EventNode::get_num_branch() {
  size_t num_branch = 0;
  this->get_num_branch(num_branch);
  return num_branch;
}

inline void EventNode::get_num_branch(size_t& num_branch) {
  for(EventNode* child_node : this->children_node_list_) {
     child_node->get_num_branch(num_branch);
  }
  if (this->children_node_list_.size()>1) {
    num_branch++;
  }
}


// =============== Tree =====================
EventTree::EventTree()
 {
  /*Constructor for EventTree. Creates an EventTree with ascending id_, and random color*/
  id_ = IdGenerator++;
  color_ = std::tuple<double,double,double> {(double)rand() / RAND_MAX,
        (double)rand() / RAND_MAX,(double)rand() / RAND_MAX};

  Clear();
//  children_set_.reserve(20); // It is to be expected no more than X children
//  clear();
}

inline void EventTree::AddEvent(EventNode* const &node) {
  /*Adds event nodes to the tree. Sets its parent_tree_ as the tree associated with the node and calls AddEventData*/
  // Associate node to current tree
  assert(!node->is_associated()); // It should be already associated
  assert(parent_tree_!=nullptr);
  node->data_.tree = parent_tree_;

  // All new modifications is done in the parent tree;
  // Modify data
  this->AddEventData(node);

  // Mutate tree
  parent_tree_->PushNewHeadNode(node);
}

inline void EventTree::AddEventData(EventNode* const &node) {
  /*Adds event data to tree */
  // All new modifications is done in the parent tree;
  // Modify data
  parent_tree_->AddAngle(node->data_.orientation);
  parent_tree_->size_++;
  parent_tree_->moment_.Add(node->data_.x, node->data_.y);
}

inline void EventTree::RemoveEventData(EventNode* const &node) {
  parent_tree_->moment_.Substract(node->data_.x, node->data_.y);
  parent_tree_->size_--;
  if (!is_top_tree()) {
    this->size_--;
  }
}

inline void EventTree::PushNewHeadNode(EventNode* const &node) {
  assert(this->is_top_tree()); // Only makes sense to add in the top level tree
  if (head_node_!=nullptr) { // Todo(ialzugaray): This will usually happens but at initialization and therefore we could get a specific function for it
    assert(head_node_->is_head());
    head_node_->push_front(node);
  }
  head_node_ = node;
}

inline void EventTree::AddAngle(const OrientationVector& orientation_add,
                                      const size_t& weight) {
  assert(this->is_top_tree()); // Only add angle in top level
  int n = this->size_;
  float n_inv = 1.0/(n+weight); // WARNING: Assume the size of the cluster size is incremented AFTER this call

//  if(orientation_.Similarity(orientation_add)<0) { // Assume at this point the orientation is above threshold so only requires correction if flipped
//    orientation_add.Flip();
//  }
    if(orientation_.Similarity(orientation_add)>=0) { // Assume at this point the orientation is above threshold so only requires correction if flipped
      orientation_.SetComponents((orientation_.dx*n + orientation_add.dx*weight)*n_inv,
                                 (orientation_.dy*n + orientation_add.dy*weight)*n_inv);
    } else {
      orientation_.SetComponents((orientation_.dx*n - orientation_add.dx*weight)*n_inv,
                                 (orientation_.dy*n - orientation_add.dy*weight)*n_inv);
    }


  assert(orientation_.isNormBounded());
  assert(orientation_.isValid());
}

inline void EventTree::AddTreeData(const EventTree* const &tree) {
  assert(this->is_top_tree());
  size_t tree_size = tree->size_;
  this->AddAngle(tree->orientation_, tree_size);
  this->size_ += tree_size;
  this->moment_.Add(tree->moment_);
}

inline void EventTree::RemoveTreeData(const EventTree* const &tree) {
  assert(this->is_top_tree());
//  this->SubstractAngle(tree->orientation_, tree_size);
  this->size_ -= tree->size_;
  this->moment_.Substract(tree->moment_); // TODO(ialzugaray): If this is only applicable during merging, moment is no longer relevant
}

inline void EventTree::Merge(EventTree* const &merged_tree) {
  // We only merge to the top levels clusters
  assert (this->is_top_tree());
  assert (merged_tree->is_top_tree());

  assert (this->head_node_!=nullptr);
  assert (merged_tree->head_node_!=nullptr);

//  if (this->size()<merged_tree->size()) {
//    std::swap(this->color_,merged_tree->color_);
//  }

  // Reconnect branches accordingly (In a new head)
  // Select the newest head between the two trees and pushfront in the other
  if (EventNode::is_newer(this->head_node_, merged_tree->head_node_)) {
    merged_tree->head_node_->push_front(this->head_node_);
  } else { // Merged tree has a newer head
    this->head_node_->push_front(merged_tree->head_node_);
    this->head_node_ = merged_tree->head_node_;
  }


  // Add data associated to node distribution
  this->AddTreeData(merged_tree);

  // Transfer merged tree children to new parent (this)
  merged_tree->TransferChildrenToNewParent(this);
  this->children_tree_set_.insert(merged_tree); // Add also merged tree as a child

  assert (this->head_node_!=nullptr);

  // Point from merged tree to new parent this
  merged_tree->parent_tree_ = this;

  assert (this->head_node_!=nullptr);
  merged_tree->ClearChildData();

  //TODO(ialzugaray): maybe it would be a good idea to transfer the last size in thinning from merged to new parent

  assert (this->head_node_!=nullptr);
}

inline void EventTree::ClearChildData() {
  // Clear other volatile data no longer relevant as a child
  this->age_ = INVALID_TIME;
  this->head_node_ = nullptr;
  this->moment_.Reset();
  this->orientation_.clear();
  this->last_thinning_size_=0;
}

inline bool EventTree::isChildData() {
  // Clear other volatile data no longer relevant as a child
  return (this->head_node_ == nullptr &&
  this->age_ == INVALID_TIME &&
  this->orientation_.empty() &&
  this->moment_.empty()) &&
  this->last_thinning_size_==0;
}

inline void EventTree::TransferChildrenToNewParent(EventTree* const &new_parent) {
  // Only transfer between top trees
  assert(new_parent->is_top_tree());
  assert(this->is_top_tree());

  // Remove current<-children link && Add new <- children link
  for(auto& child_tree : this->children_tree_set_) {
    assert(this == child_tree->parent_tree_); // All should be pointing to the current object as parent
    child_tree->parent_tree_ = new_parent;
    this->RemoveTreeData(child_tree);
  }
  // Remove current->children link && Add new->children
  //TODO(ialzugaray): this should be done by moving the objects instead of copying&delete
  new_parent->children_tree_set_.insert(this->children_tree_set_.begin(),
                                        this->children_tree_set_.end());
  this->children_tree_set_.clear();
}

inline void EventTree::Clear() {
  age_ = INVALID_TIME;
//  assert(this->size_==0); // It should be empty
  size_ = 0; // Should be already 0 if procedural deleting
  moment_.Reset(); // Should be already 0 if procedural deleting
  orientation_.clear();

  // Tree conectivity
  parent_tree_ = this;
//  assert(children_tree_set_.empty());
  children_tree_set_.clear(); // Should have been transfered already
//  assert(head_node_== nullptr)
  head_node_ = nullptr;

  this->last_thinning_size_=0;
}

inline void EventTree::ClearAssociation() {
  assert(is_removable());
  parent_tree_->children_tree_set_.erase(this);
  parent_tree_ = this;
}



inline void EventTree::ForceThinning(const float& max_minor,const size_t& min_node) {
  assert(this->is_top_tree()); // Only applicable to top levels

  EventNode* node = this->head_node_;
  assert(node!= nullptr);
  // if(node == nullptr) {std::cout<<"There comes a segfault: this->head_node_ is nullptr"; abort();}
  if (node==nullptr){
      this->last_thinning_size_ = this->size_;
      return;
  }
  ImageMoment temp_moment;
  size_t temp_size = 0;


//  float max_minor = 2;
  double ratio_theoretical;

// for min_node times, add node data of newest children to temp_moment
  for (temp_size = 0; temp_size<min_node; ++temp_size) {
    temp_moment.Add(node->data_.x, node->data_.y);
    node = node->pull_through(); // the function returns newest child of node, the statement therfore sets node as its newest child
    if (node==nullptr){
      this->last_thinning_size_ = this->size_;
      return;
    }
  }

  do {
    temp_size++;
    temp_moment.Add(node->data_.x, node->data_.y);
    ratio_theoretical = max_minor/(temp_size/2.0);

    if (ImageMoment::GetAxisRatio(temp_moment, temp_size) > ratio_theoretical && node->parent_node() != nullptr) {
    // if ((ImageMoment::GetAxisRatio(temp_moment, temp_size) > ratio_theoretical || abs(ImageMoment::GetAngle(temp_moment, temp_size) - atan2(node->data_.orientation.dy,node->data_.orientation.dx) - (3.14/4)) > (3.14/20)) && node->parent_node() != nullptr) { // Test for new line thinning metric
      // std::cout<<"thinning"<<std::endl;
      // if(node->parent_node() == nullptr) {std::cout<<"SegFault on the way: parent_node is a nullptr"<<std::endl; abort();}
      temp_size--;
      temp_moment.Substract(node->data_.x, node->data_.y);
      node->parent_node()->clear_children_list(); // Break last parent->node connection
      node->ShredTreeDownstream();
      this->last_thinning_size_ = this->size_;
      return;
    }

    if (node!=nullptr) { //ToDO: unnecessary change
      node = node->pull_through();
    } else {
      break;
    }
    // if (node==nullptr) {std::cout<<"SegFault on the way: node is a nullptr"<<std::endl; abort();}
  } while (node!=nullptr);

  this->last_thinning_size_ = this->size_;
}

#undef TEMPT
} // namespace event_tracker
#endif // LINE_FEATURE_HPP
