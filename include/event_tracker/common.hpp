#ifndef COMMON_H
#define COMMON_H

#include<map>
#include<list>
#include <algorithm>
//#include<event_tracker/line_feature.hpp>
#include <algorithm>
#include <math.h>
namespace event_tracker {
typedef int64_t EventTime;
typedef size_t EventKey;
const EventTime INVALID_TIME = 0; // Assume no timestamp will be 0 -> origin of time;

struct OrientationVector  {
  float dx, dy;

  inline bool isNormBounded() const {
    return  std::sqrt((dx*dx + dy*dy))<=1.001;
  }

  inline bool isValid() const {
    return ( (dx!=0 || dy!=0) );
  }


  inline void clear() {
    dx = 0; dy = 0;
  }

  inline bool empty() {
    return (!dx && !dy);
  }

  inline bool SetComponents(float _dx, float _dy){
    // Todo(ialzugaray): I think this method could be boosted
    float norm = std::sqrt(_dx*_dx + _dy*_dy);
    if (norm!=0) {
      dx = _dx/norm;
      dy = _dy/norm;
      return true;
    } else {
      return false;
    }
  }
  inline float Similarity(OrientationVector* orientation_vector) const {
    // Return 0 (no similarity) if compared to uninitialized orientation vector
    return (orientation_vector->dx*this->dx + orientation_vector->dy*this->dy);
  }
  inline float Similarity(const OrientationVector& orientation_vector) const{
    // Return 0 (no similarity) if compared to uninitialized orientation vector
    return (orientation_vector.dx*this->dx + orientation_vector.dy*this->dy);
  }
  inline void Flip(){
    dx = -dx; dy = -dy;
  }
};


struct ImageMoment {
  // Assume ImageMoment moment_ initializes to 0
  uint x, y, xy, xx, yy; // Todo(ialzugarary): check for overflow
  float cx, cy, l, angle;

  ImageMoment() {
    Reset();
  }

  inline void Add(const uint16_t& _x, const uint16_t& _y){
    x += _x;
    y += _y;
    xy += _x * _y;
    xx += _x * _x;
    yy += _y * _y;
  }

  inline void Substract(const uint16_t& _x, const uint16_t& _y){
    x -= _x;
    y -= _y;
    xy -= _x * _y;
    xx -= _x * _x;
    yy -= _y * _y;
  }

  //  ImageMoment(const double& _x, const double& _y) {
  //    x = _x;
  //    y = _y;
  //    xy = _x * _y;
  //    xx = _x * _x;
  //    yy = _y * _y;
  //  }

  inline void Add(const ImageMoment& obj){
    x += obj.x;
    y += obj.y;
    xy += obj.xy;
    xx += obj.xx;
    yy += obj.yy;
  }

  inline void Substract(const ImageMoment& obj){
    x -= obj.x;
    y -= obj.y;
    xy -= obj.xy;
    xx -= obj.xx;
    yy -= obj.yy;
  }

  inline void Reset(){
    x = 0; y=0; xy = 0 ; xx = 0; yy = 0; cx = 0; cy = 0; l = 0; angle = 0;
  }

  static float GetMajorAxis(const ImageMoment& moment, uint64_t size) {
    float size_f = (float) size;
    float x_center = moment.x / size_f;
    float y_center = moment.y / size_f;

    float a = (moment.xx / size_f) - (x_center * x_center);
    float b = 2.0f * ((moment.xy / size_f) - (x_center * y_center));
    float c = (moment.yy / size_f) - (y_center * y_center);


    float major_axis = sqrt(6.0f * (a + c + sqrt(b * b + (a - c) * (a - c))));
    return major_axis;
  }

  static float GetAngle(const ImageMoment& moment, uint64_t size) {
    float size_f = (float) size;
    float x_center = moment.x / size_f;
    float y_center = moment.y / size_f;

    float a = (moment.xx / size_f) - (x_center * x_center);
    float b = 2.0f * ((moment.xy / size_f) - (x_center * y_center));
    float c = (moment.yy / size_f) - (y_center * y_center);


    float angle = 0.5 * atan2(b,(a-c));
    return angle;
  }

  static float GetX(const ImageMoment& moment, const size_t& size) {return moment.x/size;}

  static float GetY(const ImageMoment& moment, const size_t& size) {return moment.y/size;}


  static float GetAxisRatio(const ImageMoment& moment, const size_t& size) {

    float size_f = (float) size;
    float x_center = moment.x / size_f;
    float y_center = moment.y / size_f;

    float a = (moment.xx / size_f) - (x_center * x_center);
    float b = 2.0f * ((moment.xy / size_f) - (x_center * y_center));
    float c = (moment.yy / size_f) - (y_center * y_center);


//    float major_axis = sqrt(6.0f * (a + c + sqrt(b * b + (a - c) * (a - c))));
//    float minor_axis = sqrt(6.0f * (a + c - sqrt(b * b + (a - c) * (a - c))));

    float major_axis_par= a + c + sqrt(b * b + (a - c) * (a - c));
    float minor_axis_par = a + c - sqrt(b * b + (a - c) * (a - c));

//    std::cout << "{" << a + c  << " , " << sqrt(b * b + (a - c) * (a - c)) << " , " <<  a + c  - sqrt(b * b + (a - c) * (a - c)) << "}" << std::endl;
//    std::cout << "x: " << moment.x << " \ty: " << moment.y << " \txx: " << moment.xx << " \tyy: " << moment.yy  << std::endl;
//    std::cout << "a: " << a << " \tb: " << b << " \tc: " << c << std::endl;
//    std::cout << minor_axis_par << " | " << major_axis_par << std::endl;
    if (minor_axis_par<0) {
      return 0;
    } else {
      return (major_axis_par > 0) ?  sqrt(minor_axis_par /major_axis_par): 0;
    }

  }

  bool operator==(const ImageMoment &rhs) const {
    if (this->x == rhs.x && this->y == rhs.y &&
        this->xy == rhs.xy && this->xx == rhs.xx && this->yy == rhs.yy) {
      return true;
    } else {
      return false;
    }
  }

  inline bool empty() {
    return (!x && !y && !xy && !xx && !yy);
  }
};




class TimeSortedTreeNode {
public:
  TimeSortedTreeNode(EventKey key, ImageMoment moment) :
    parent_(nullptr), key_(key), moment_(moment) {
    timestamp_ = INVALID_TIME;
    children_list_.clear();
    assert(this->is_uninitialized());
  }

  TimeSortedTreeNode* push_front(TimeSortedTreeNode* new_node){
    assert(this->is_head()); // It only makes sense in the head (Always add newer events)
    assert(is_newer(new_node, this) || is_equal(new_node, this)); // Ensure new node is newer or equal in time
    this->parent_ = new_node;
    new_node->children_list_.push_back(this);
    return new_node;
  }

  TimeSortedTreeNode* remove(){
    TimeSortedTreeNode* new_parent = nullptr;
    if (is_head() && is_tail()) { // Unique element in the tree
      return nullptr;
    } else if (is_head()){ // On the top of the tree
      // New parent is the the newest children
      ChildIter it_newest_child = find_newest_children();
      assert(it_newest_child != children_list_.end());
      this->children_list_.erase(it_newest_child); // Delete from children list
      new_parent = *it_newest_child;
      new_parent->parent_ = nullptr;
    } else if (!is_head()){ // Inside the tree -> reconnect parent to children
      new_parent = this->parent_;
      // Erase from parent's children list
      ChildIter find_it = std::find(new_parent->children_list_.begin(),
                                    new_parent->children_list_.end(), this);
      assert(find_it != new_parent->children_list_.end()); // The element must exist in the list
      new_parent->children_list_.erase(find_it);
      this->parent_ = nullptr;
    }
    this->TransferChildrenToNewParent(new_parent);
    assert(this->is_uninitialized());
    return new_parent;
  }

  static TimeSortedTreeNode* merge(TimeSortedTreeNode* lhs,
                                   TimeSortedTreeNode* rhs) {
    assert(lhs->is_head() && rhs->is_head());// Only merge from heads
    TimeSortedTreeNode *new_head, *old_head;

    if (is_newer(lhs, rhs)) { new_head = lhs; old_head = rhs;}
    else { new_head = rhs; old_head = lhs;}

    old_head->parent_ = new_head;
    new_head->children_list_.push_back(old_head);
    return new_head;
  }
  TimeSortedTreeNode* pull_through() {
    if (this->is_tail()) {return nullptr;}

    // Boosting if a single children (Direct chain link)
    // Todo(ialzugaray): verify that this actually boost anything
    if (this->children_list_.size()==1) {return this->children_list_.front();}

    // Delete from children list
    ChildIter it_newest_child = this->find_newest_children();
    this->children_list_.erase(it_newest_child);
    // Move children to the next node
    TimeSortedTreeNode* next_node = *it_newest_child;
    this->TransferChildrenToNewParent(next_node);
    this->children_list_.push_back(next_node); // Todo(ialzugaray): this is redundant as the link to this node was already there
    assert(this->children_list_.size()==1);
    return next_node;
  }

  void shred_tree_downstream(std::list<EventKey>& removed_leaf_key_list) {
    // Recursive algorithm to destroy nodes below current node to become the new tail
    for (TimeSortedTreeNode* leaf_node: this->children_list_) { // Todo(ialzugaray): this could be boosted so the pointer is not copied - Maybe worth it
      leaf_node->parent_ = nullptr;
      removed_leaf_key_list.push_back(leaf_node->key_);
      leaf_node->shred_tree_downstream(removed_leaf_key_list);
    }
    this->children_list_.clear();

    assert(this->is_tail());
  }

  bool is_head(){return (parent_ == nullptr);}
  bool is_tail(){return (children_list_.empty());}
  bool is_uninitialized(){return (parent_ == nullptr && children_list_.empty());}

  EventTime timestamp_;
  EventKey key_;
  ImageMoment moment_;


private :
  typedef std::list<TimeSortedTreeNode*> ChildrenList;
  typedef ChildrenList::iterator ChildIter;

  static bool is_newer(TimeSortedTreeNode* lhs, TimeSortedTreeNode* rhs) {
    return (lhs->timestamp_ > rhs->timestamp_); // Is LHS strictly newer than RHS
  }
  static bool is_equal(TimeSortedTreeNode* lhs, TimeSortedTreeNode* rhs) {
    return (lhs->timestamp_ == rhs->timestamp_); // Is LHS strictly newer than RHS
  }


   ChildIter find_newest_children() {
    return  std::max_element(children_list_.begin(),children_list_.end(),
                             is_newer);
  }

  void TransferChildrenToNewParent(TimeSortedTreeNode* new_parent) {
    for (auto& child : this->children_list_) {
      assert(child->parent_==this);// It must currently point to the current node
      child->parent_= new_parent; // Baypass into the new parent
    }
    // Transfer the childrens from current to parent node
    new_parent->children_list_.splice( new_parent->children_list_.begin(),
                                    this->children_list_);
    assert(this->children_list_.size()==0); // It should have emptied the list
  }


  TimeSortedTreeNode* parent_;
  ChildrenList children_list_;
};






} // namespace event_tracker

#endif // COMMON_H
