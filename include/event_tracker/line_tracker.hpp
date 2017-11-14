/* Copyright (C) Ignacio Alzugaray, 2017 <aignacio at student dot ethz dot ch>
 * ETH Zurich - V4RL
 */
#ifndef LINE_TRACKER_HPP
#define LINE_TRACKER_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>


#include "../event_tracker/line_tracker.h" // #include <event_tracker/line_tracker.h>


namespace event_tracker {
#define TEMPT template <uint16_t ROWS, uint16_t COLS>

TEMPT
LineTracker<ROWS,COLS>::LineTracker() {
  std::cout << "LineTracker Constructor" << std::endl;
  // Initialize Feature Info Array
  for (size_t x = 0; x < ROWS; x++) {
    for (size_t y = 0; y < COLS; y++){
      event_node_array_(x,y) = new EventNode(x,y);    
    }
  }

  // Initialize empty clusters;
  for (size_t i = 0; i<MAX_CLUSTERS; i++) {
    feature_stack_.push(new EventTree());
    active_feature_params_.push_back(FeatureParams());
  }
  active_feature_.resize(MAX_CLUSTERS, nullptr);
  // std::cout<<"Initializing active_feature_params_"<<std::endl;
  // active_feature_params_ = std::vector<FeatureParams>(MAX_CLUSTERS);

  // Rendering options
  cv::namedWindow("Lines", cv::WINDOW_NORMAL);
  line_img_ = cv::Mat(ROWS,COLS,CV_32FC3);
}

TEMPT
inline void LineTracker<ROWS,COLS>::AssignEvent(EventNode* const &event_node,
                                                EventTree* feature) {

//  std::cout << "AssignEvent: " << "(" << event_node->data_.x <<","<< event_node->data_.y << ")" << std::endl;
  assert(!event_node->is_associated()); // At this point the event has no association
  assert(feature->is_top_tree()); // Although this is managed internally and therefore it might not be necessary
  feature->AddEvent(event_node);
  assert(event_node->is_associated()); // At this point the event has rassociation
}


TEMPT
inline void LineTracker<ROWS,COLS>::RemoveEvent(EventNode* const &event_node) {

//  std::cout << "RemoveEvent: " << "(" << event_node->data_.x <<","<< event_node->data_.y << ")"<< std::endl;
//  EventTree* const feature = event_node->data_.tree;
  event_node->Remove();

//  if (feature->size()==0){
//    if (!(feature->is_top_tree())) { // Cascade structure to avoid unnecesary allocation of variables
//      EventTree* const parent_tree = feature->parent_tree();
//      if (parent_tree->size()==0) {
//        FreeFeature(parent_tree);
//      }
//    }
//    FreeFeature(feature);
//  }

  assert(event_node->is_isolated());
  assert(!event_node->is_associated());
}

TEMPT
inline EventTree* LineTracker<ROWS,COLS>::InitializeFeature() {
  // Extract last unused feature from stack
  assert(feature_stack_.size()); // Stack of unused features cannot be empty
  EventTree* feature = feature_stack_.top();
  // Add to active feature vector and remove from stack
  active_feature_[feature->id()] = feature;
  feature_stack_.pop();
  assert(feature->is_top_tree());
  assert(feature->head_node()==nullptr);
  return feature;

}

TEMPT
inline void LineTracker<ROWS,COLS>::FreeFeature(EventTree * const &feature) {

  feature->ClearAssociation();
  feature->Clear();    // Maybe unnecesary if procedural deleting
//  std::cout << "Is top: "  << feature->is_top_tree() << std::endl;
  active_feature_[feature->id()] = nullptr;
//  std::cout << "Is top: "  << feature->is_top_tree() << std::endl;
  feature_stack_.push(feature);

  assert(feature->is_top_tree());
  assert(feature->head_node()==nullptr);

}


TEMPT
inline void LineTracker<ROWS,COLS>::FreeEmptyFeature() {
  mutex_feature_stack_.lock(); // TODO(ialzugaray): this lock doesnt guard properly-> it shouldnt cover the complete set of features but the last in the stack
  for (EventTree * feature : active_feature_) {
    if (feature!=nullptr ) {
      if(feature->is_removable()) {
        FreeFeature(feature);
      }
    }
  }
  mutex_feature_stack_.unlock();
}


TEMPT
inline void LineTracker<ROWS,COLS>::Render(int delay) {
  const int min_cluster_size_render = 10;
  line_img_ = cv::Scalar(0.0,0.0,0.0);
  for (size_t x = 0; x < ROWS; x++) {
    for (size_t y = 0; y < COLS; y++){
      EventNode* node = event_node_array_(x,y);
      if (node->is_associated() && node->data_.tree->parent_tree()->size()> min_cluster_size_render) {
//        std::cout << "[" << node->data_.tree->parent_tree()  << "]" << node->data_.tree->parent_tree()->size()<< std::endl;
        assert(node->data_.tree->parent_tree() !=nullptr);
        line_img_.at<cv::Point3f>(y,x) = cv::Point3f(std::get<0>(node->data_.tree->parent_tree()->color()),
                                                       std::get<1>(node->data_.tree->parent_tree()->color()),
                                                       std::get<2>(node->data_.tree->parent_tree()->color()));
      }
    }
  }

  cv::imshow("Clusters", line_img_);
  cv::waitKey(delay);
}

TEMPT
inline void LineTracker<ROWS,COLS>::LogClusters(int delay, std::ofstream& cluster_log) {
  uint64_t id;
  const int min_cluster_size_log = 5;
  cluster_log << -1  << ", " << -1 << ", " << -1 << ", " << -1 << ", " << -1 << ", " << -1 <<std::endl; 
  for (size_t x = 0; x < ROWS; x++) {
    for (size_t y = 0; y < COLS; y++){
      EventNode* node = event_node_array_(x,y);
      if (node->is_associated() && node->data_.tree->parent_tree()->size()> min_cluster_size_log){
//        std::cout << "[" << node->data_.tree->parent_tree()  << "]" << node->data_.tree->parent_tree()->size()<< std::endl;
        assert(node->data_.tree->parent_tree() !=nullptr);
        // cluster_img_.at<cv::Point3f>(y,x) = cv::Point3f(std::get<0>(node->data_.tree->parent_tree()->color()),
        //                                                std::get<1>(node->data_.tree->parent_tree()->color()),
        //                                                std::get<2>(node->data_.tree->parent_tree()->color()));
        id = node->data_.tree->parent_tree()->id();
        cluster_log << id  << ", " << x << ", " << y << ", " << node->data_.orientation.dx << ", " << node->data_.orientation.dy << ", " << node->data_.timestamp <<std::endl;
      }
    }
  }
}

TEMPT
inline void LineTracker<ROWS,COLS>::RenderLines(int delay) {
  cv::Mat image = cv::Mat::zeros( ROWS, COLS, CV_8UC3 );
  cv::Point start, dir, end;
  float dx, dy;
  ImageMoment moment;
  FeatureParams line;
  float test;
  uint64_t n;
  uint64_t id;
  float ratio_limit_ = 2.0;

  for (EventTree* treeIt: active_feature_) {
    // std::cout<<"size " << treeIt->size() <<std::endl;
    if (treeIt != nullptr && treeIt->is_top_tree() && treeIt->size() > 6){

        if (treeIt->last_thinning_size() > treeIt->size()) {
          this->mutex_feature_stack_.lock();
          // std::cout<<"Force Thinning"<<std::endl;
        //   std::cout<<"size " << treeIt->size() << ", last_thinning_size: " << treeIt->last_thinning_size() << std::endl;
          treeIt->ForceThinning(ratio_limit_, treeIt->last_thinning_size()*0.75); //ToDo(herbstc): produces segfault after a while. why?
          this->mutex_feature_stack_.unlock();
        }
        // if (treeIt == nullptr) {std::cout<<"here you go, a segfault"<<std::endl; abort();}

        
        n = treeIt->size();
          moment = treeIt->parent_tree()->moment();
          // if (treeIt->last_thinning_size() > 0) treeIt->ForceThinning(ratio_limit_, (3.0 * treeIt->last_thinning_size())/4.0);
          // std::cout<<"Updating line params"<<std::endl;
          line.angle = ImageMoment::GetAngle(moment, n);
          line.l = ImageMoment::GetMajorAxis(moment, n);
          line.x = moment.x / n;
          line.y = moment.y / n;
          line.color = treeIt->color();
          dx = 0.5 * line.l * cos(line.angle);
          dy = 0.5 * line.l * sin(line.angle);
          start.x = (int) round(line.x - dx);
          start.y = (int) round(line.y - dy);
          end.x = (int)round(line.x + dx);
          end.y = (int) round(line.y + dy);
          // std::cout << start.x << " to " <<  end.x << std::endl;
          // std::cout << start.y << " to " <<  end.y << std::endl;
          // std::cout << l << std::endl;
          cv::line(image,start,end, cv::Scalar(100,100,100));
        
      }
      // n = treeIt->last_thinning_size();
      // if (n > 10){
      //   moment = treeIt->moment();
      //   // if (treeIt->last_thinning_size() > 0) treeIt->ForceThinning(ratio_limit_, (3.0 * treeIt->last_thinning_size())/4.0);
      //   // std::cout<<"Updating line params"<<std::endl;
      //   line.angle = ImageMoment::GetAngle(moment, n);
      //   line.l = ImageMoment::GetMajorAxis(moment, n);
      //   line.x = moment.x / n;
      //   line.y = moment.y / n;
      //   line.color = treeIt->color();
      //   dx = 0.5 * line.l * cos(line.angle);
      //   dy = 0.5 * line.l * sin(line.angle);
      //   start.x = (int) round(line.x - dx);
      //   start.y = (int) round(line.y - dy);
      //   end.x = (int)round(line.x + dx);
      //   end.y = (int) round(line.y + dy);
      //   // std::cout << start.x << " to " <<  end.x << std::endl;
      //   // std::cout << start.y << " to " <<  end.y << std::endl;
      //   // std::cout << l << std::endl;
      //   cv::line(image,start,end, cv::Scalar(100,100,100));
      // }
    }
   

  cv::imshow("Lines", image);
  cv::waitKey(delay);
}

TEMPT
inline void LineTracker<ROWS, COLS>::UpdateLineParams() {
  // For each feature, get line parameters and save them in the vector active_feature_params
  assert(active_feature_ != nullptr);
  ImageMoment moment;
  FeatureParams line;
  float test;
  uint64_t n;
  uint64_t id;
  for (EventTree* treeIt: active_feature_) {
    if (treeIt != nullptr){
      n = treeIt->size();
      if (n > 10){
        moment = treeIt->moment();
        // std::cout<<"Updating line params"<<std::endl;
        line.angle = ImageMoment::GetAngle(moment, n);
        line.l = ImageMoment::GetMajorAxis(moment, n);
        line.x = moment.x / n;
        line.y = moment.y / n;
        line.color = treeIt->color();
        active_feature_params_[id] = line;
      }
    }

  }
} 

#undef TEMPT

} // namesapce event_tracker
#endif // LINE_TRACKER_HPP
