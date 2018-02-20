/* Copyright (C) Ignacio Alzugaray, 2017 <aignacio at student dot ethz dot ch>
 * ETH Zurich - V4RL
 */

#ifndef ELISED_HPP
#define ELISED_HPP

#include <unordered_set>
#include <list>
#include <map>


#include "../event_tracker/elised.h" //#include <event_tracker/elised.h>
#include "../aux/tic_toc.hpp"

#define TIMING

namespace event_tracker {

#define TEMPT template <uint16_t ROWS, uint16_t COLS>

    TEMPT
    Elised<ROWS,COLS>::Elised() {
        std::cout << "Elised Constructor" << std::endl;

        time_array_.resize(ROWS, COLS);
        // Parameters
        // TODO(ialzugaray): get rid of hardcoded values
        similarity_threshold_ = 0.9;
        min_candidate_cluster_ = 3;
        max_buffer_ = 4000;
        buffer_size_ = 0;
        ratio_limit_ = 2.0; //2

        // Saliency detectors mask: In this case sobel
        if(MASK_SIZE == 3)  {
            saliency_mask_x_ << 1, 0,-1, 2, 0,-2, 1, 0,-1;
            saliency_mask_y_ << 1, 2, 1, 0, 0, 0, -1, -2, -1;
        } else if (MASK_SIZE == 5) {
            saliency_mask_x_ << 1, 2, 0, -2, -1, 4, 8, 0, -8, -4, 6, 12, 0, -12, -6, 4, 8, 0, -8, -4, 1, 2, 0, -2, -1;
            saliency_mask_y_ << 1, 4, 6, 4, 1, 2, 8, 12, 8, 2, 0, 0, 0, 0, 0, -2, -8, -12, -8, -2, -1, -4, -6, -4, -1;
        } else if (MASK_SIZE == 9) {
            saliency_mask_x_ << 4, 3, 2, 1, 0, -1, -2, -3, -4,
                    5, 4, 3, 2, 0, -2, -3, -4, -5,
                    6, 5, 4, 3, 0, -3, -4, -5, -6,
                    7, 6, 5, 4, 0, -4, -5, -6, -7,
                    8, 7, 6, 5, 0, -5, -6, -7, -8,
                    7, 6, 5, 4, 0, -4, -5, -6, -7,
                    6, 5, 4, 3, 0, -3, -4, -5, -6,
                    5, 4, 3, 2, 0, -2, -3, -4, -5,
                    4, 3, 2, 1, 0, -1, -2, -3, -4;
            saliency_mask_y_ << 4, 5, 6, 7, 8, 7, 6, 5, 4, 3, 4, 5, 6, 7, 6, 5, 4, 3, 2, 3, 4, 5, 6, 5, 4, 3,
                    2, 1, 2, 3, 4, 5, 4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, -1, -2, -3, -4, -5, -4 ,-3,
                    -2, -1, -2, -3, -4, -5, -6, -5, -4, -3, -2, -3, -4, -5, -6, -7, -6, -5, -4, -3,
                    -4, -5, -6, -7, -8, -7, -6, -5, -4;

        } else {
            std::cout<<"\033[1;31m Error: Mask size not appropiated (3 or 5)\n \033[0m\n ";
            throw std::exception();
        }
    }

    TEMPT
    inline bool Elised<ROWS,COLS>::AddEvent(const uint16_t& x, const uint16_t& y,
                                            const uint64_t& timestamp, const bool &polarity) {


        // Strategy implementation
//  std::cout << "-------Processing: " << "(" << x << "," << y << ")" << std::endl;

//  uint64_t time_event_info = 0, time_remove = 0, time_buffer=0,
//      time_timestamp = 0, time_orientation = 0, time_similar = 0, time_merge = 0;

//  tic("nsec");

        // Discard event processing if too close to image borders
        if(x<((MASK_SIZE-1)/2) || y<((MASK_SIZE-1)/2) || x>(ROWS-1-(MASK_SIZE-1)/2) || y>(COLS-1-(MASK_SIZE-1)/2)) {
            return false;
        }
        time_array_(x,y) = timestamp; // TODO(ialzugaray): duplicated so we can apply enhanced operations

        // Remove Event association with clusters

        EventNode* event_node = this->event_node_array_(x,y);
//  time_event_info = toc("muted"); tic("nsec");

        if (event_node->is_associated()) {
            assert(event_node->data_.tree->parent_tree()!=nullptr);
            this->mutex_feature_stack_.lock(); // Todo(ialzugaray): this lock should guard only feature instead of the complete set
            this->RemoveEvent(event_node);
            this->mutex_feature_stack_.unlock();
        }

//  time_remove = toc("muted"); tic("nsec");

        event_node->data_.timestamp = timestamp;

//  time_timestamp = toc("muted"); tic("nsec");

        // Delete oldest event if global buffer is full
        UpdateGlobalBuffer(event_node);
//  time_buffer = toc("muted"); tic("nsec");



        // Set orientation according to time surface
        if (!SetEventOrientation(event_node)) {
            // Event is isolated in the time surface - No neighbors time information
            return false;
        }
        this->mutex_feature_stack_.lock(); // Todo(ialzugaray): this lock should guard only feature instead of the complete set
        Process(event_node);
        this->mutex_feature_stack_.unlock();
//    time_orientation = toc("muted"); tic("nsec");
        // Get candidates
//  std::list<EventNode*> candidate_list;
//  SimilarNeighbours(event_node, candidate_list);



//time_similar = toc("muted"); tic("nsec");
//  bool is_merged = MergeCandidates(candidate_list);
//time_merge = toc("muted"); tic("nsec");


//std::cout  << "EventInfo: " << time_event_info
//           << " \tRemove: " << time_remove
//           << " \tTimestamp: " << time_timestamp
//           << " \tBuffer: " << time_buffer
//           << " \tOrientation: " << time_orientation
//           << " \tSimilar: " << time_similar
//            << " \tMerge: " << time_merge
//            << " \tTotal: "<< time_event_info + time_remove + time_timestamp + time_buffer + time_orientation + time_similar + time_merge<<std::endl;

//  std::cout << size;
//  if (is_merged) {std::cout << "*" ;}
//  std::cout << std::endl;
        return true;

    }

    TEMPT
    inline void Elised<ROWS,COLS>::Process(EventNode* const &node_root) {
//  std::cout << "Event" << std::endl;
        uint64_t time_vec_push=0, time_similarity=0, time_initialization = 0, time_load = 0;

//tic("nsec");
        // Load data
        EventNode * const node_neigh_00 = this->event_node_array_(node_root->data_.x-1, node_root->data_.y-1);
        EventNode * const node_neigh_10 = this->event_node_array_(node_root->data_.x,   node_root->data_.y-1);
        EventNode * const node_neigh_20 = this->event_node_array_(node_root->data_.x+1, node_root->data_.y-1);
        EventNode * const node_neigh_01 = this->event_node_array_(node_root->data_.x-1, node_root->data_.y);
        EventNode * const node_neigh_21 = this->event_node_array_(node_root->data_.x+1, node_root->data_.y);
        EventNode * const node_neigh_02 = this->event_node_array_(node_root->data_.x-1, node_root->data_.y+1);
        EventNode * const node_neigh_12 = this->event_node_array_(node_root->data_.x,   node_root->data_.y+1);
        EventNode * const node_neigh_22 = this->event_node_array_(node_root->data_.x+1, node_root->data_.y+1);
        /*8-neighborhood of root_node */


//  time_load = toc("muted"); tic("nsec");
        static std::vector<EventNode *> node_vec(9); node_vec.clear(); // Requires clear if it is static
        node_vec.push_back(node_root);
        // put root_node at the end of a vector of nodes containing its neighborhood
        static std::vector<EventTree *> tree_vec(9); tree_vec.clear(); // Requires clear if it is static

//time_initialization = toc("muted"); tic("nsec");

//  tic("nsec");
        const SimilarityStatus node_neigh_00_similarity = isSimilar(node_root, node_neigh_00);
        const SimilarityStatus node_neigh_10_similarity = isSimilar(node_root, node_neigh_10);
        const SimilarityStatus node_neigh_20_similarity = isSimilar(node_root, node_neigh_20);
        const SimilarityStatus node_neigh_01_similarity = isSimilar(node_root, node_neigh_01);
//  const SimilarityStatus node_neigh_11_similarity = isSimilar(node_root, node_neigh_11);
        const SimilarityStatus node_neigh_21_similarity = isSimilar(node_root, node_neigh_21);
        const SimilarityStatus node_neigh_02_similarity = isSimilar(node_root, node_neigh_02);
        const SimilarityStatus node_neigh_12_similarity = isSimilar(node_root, node_neigh_12);
        const SimilarityStatus node_neigh_22_similarity = isSimilar(node_root, node_neigh_22);
// SimilarityStatus is given value kNodeSimililar if orientation similar to a neighbor node, or kTreeSimilar if similar to neighboring tree

//time_similarity = toc("muted"); tic("nsec");


        if (node_neigh_00_similarity==kNodeSimilar){
            node_vec.push_back(node_neigh_00);
        } else if (node_neigh_00_similarity==kTreeSimilar){
            tree_vec.push_back(node_neigh_00->data_.tree->parent_tree());
        }
        if (node_neigh_10_similarity==kNodeSimilar){
            node_vec.push_back(node_neigh_10);
        } else if (node_neigh_10_similarity==kTreeSimilar){
            tree_vec.push_back(node_neigh_10->data_.tree->parent_tree());
        }
        if (node_neigh_20_similarity==kNodeSimilar){
            node_vec.push_back(node_neigh_20);
        } else if (node_neigh_20_similarity==kTreeSimilar){
            tree_vec.push_back(node_neigh_20->data_.tree->parent_tree());
        }
        if (node_neigh_01_similarity==kNodeSimilar){
            node_vec.push_back(node_neigh_01);
        } else if (node_neigh_01_similarity==kTreeSimilar){
            tree_vec.push_back(node_neigh_01->data_.tree->parent_tree());
        }
        ////  if (node_neigh_11_similarity==kNodeSimilar){
        ////    node_vec.push_back(node_neigh_11);
        ////  } else if (node_neigh_11_similarity==kTreeSimilar){
        ////    tree_map.emplace(node_neigh_11->data_.tree);
        ////  }
        if (node_neigh_21_similarity==kNodeSimilar){
            node_vec.push_back(node_neigh_21);
        } else if (node_neigh_21_similarity==kTreeSimilar){
            tree_vec.push_back(node_neigh_21->data_.tree->parent_tree());
        }
        if (node_neigh_02_similarity==kNodeSimilar){
            node_vec.push_back(node_neigh_02);
        } else if (node_neigh_02_similarity==kTreeSimilar){
            tree_vec.push_back(node_neigh_02->data_.tree->parent_tree());
        }

        if (node_neigh_12_similarity==kNodeSimilar){
            node_vec.push_back(node_neigh_12);
        } else if (node_neigh_12_similarity==kTreeSimilar){
            tree_vec.push_back(node_neigh_12->data_.tree->parent_tree());
        }

        if (node_neigh_22_similarity==kNodeSimilar){
            node_vec.push_back(node_neigh_22);
        } else if (node_neigh_22_similarity==kTreeSimilar){
            tree_vec.push_back(node_neigh_22->data_.tree->parent_tree());
        }


//    time_vec_push = toc("muted"); tic("nsec");

//  if (tree_vec.size() > 4 ) {
//    std::cout << std::endl;
//    std::cout << "Vec " << "(" << tree_vec.size() << "):"  <<std::endl;;
//    for (std::vector<EventTree*>::iterator it = tree_vec.begin(); it != it_new_end; it++) {
//      std::cout << " " << *it << "(" << (*it)->age() <<")" <<std::endl;
//    }
//    std::cout << std::endl;
//  }
//  std::cout << "Load: " << time_load
//            << "\t Initialization: "<< time_initialization
//            << "\t Similarity: " <<time_similarity
//            << "\t Vec: " << time_vec_push << std::endl;


//    std::unordered_set<EventTree*> feature_set;
//    std::list<EventNode*>::iterator it = candidate_list.begin();
//    // Todo(ialzugaray): this could be done during the similarity check -> we are repeating processing
//    while(it!=candidate_list.end()) {
//      if ((*it)->is_associated()) {
//        assert((*it)->data_.tree->parent_tree()->is_top_tree());
//        feature_set.insert((*it)->data_.tree->parent_tree()); // Only use top level features (parents)
//        it = candidate_list.erase(it);
//      } else {
//        it++;
//      }
//    }

        //  time_feature_set = toc("muted"); tic("nsec");
        if(&(node_vec[0]->data_.timestamp) == nullptr) {
            std::cout << "data field pointer of node_vec not pointing anywhere" << std::endl;
            abort();
        }
        // Sort node_vec according to timestamp, ascending
        std::sort(node_vec.begin(),node_vec.end(),EventNode::is_older);
        // Try to initialize a new feature if no neighbor features and enough candidates
        if (tree_vec.size()==0) {
            if (node_vec.size()>=min_candidate_cluster_) {
                // Create a new cluster with candidates

                EventTree* feature = this->InitializeFeature();
                EventTime new_timestamp = node_root->data_.timestamp; // Todo(ialzugaray): Quick hack to prevent pre-ordering of the vector -> might be not a good idea; Loses individual timestamp info
                for (EventNode* node: node_vec) {
                    node->data_.timestamp = new_timestamp;
                    this->AssignEvent(node,feature);
                }

                feature->set_age(new_timestamp);
            }
            return;
        }

        // Merge to the oldest cluster


//    std::vector<EventTree*>::iterator it_new_end = std::unique(tree_vec.begin(),tree_vec.end());
//    tree_vec.resize( std::distance(tree_vec.begin(),it_new_end));


        std::vector<EventTree*>::iterator it_in, it_ext;
        it_ext= tree_vec.begin();
        // Remove duplicate trees in tree_vec
        while(it_ext != tree_vec.end()) { // TODO(ialzugaray): super inneficient code to remove duplicates
            it_in = it_ext+1;
            while(it_in != tree_vec.end()) {
                if (*it_in == *it_ext) {
                    it_in = tree_vec.erase(it_in);
                } else {
                    it_in++;
                }
            }
            it_ext++;
        }
        //  Sort trees according to their age
        std::sort(tree_vec.begin(),tree_vec.end(),EventTree::older_weak_order);
        EventTree* oldest_feature = tree_vec.front();



//    std::cout << "Merge: " << oldest_feature << " \tAge: " << oldest_feature->age() << std::endl;
        // Get oldest cluster
        for (std::vector<EventTree*>::iterator it = tree_vec.begin()+1; it!=tree_vec.end(); it++) {
//      std::cout << " - " << (*it) << " \tAge: " << (*it)->age() << std::endl;
            assert(oldest_feature->is_top_tree());
            assert((*it)->is_top_tree());
//      if (oldest_feature!=(*it)) {
            assert(oldest_feature!=(*it));
            oldest_feature->Merge((*it));
//      } else {
//        for (std::vector<EventTree*>::iterator it = tree_vec.begin(); it!=tree_vec.end(); it++) {
//          std::cout << " " << *it << " "  << std::endl;;
//        }

//        std::cout << std::endl;
//      }

            assert(oldest_feature->head_node()!=nullptr);
        }

        EventTime oldest_feature_time = oldest_feature->newest_timestamp();
        // Merge candidates to oldest cluster
        for (EventNode* node: node_vec) {
            // node->data_.timestamp = oldest_feature_time; // ToDo(fake as all the candidates has new timestamp according to the newest added event)
            this->AssignEvent(node, oldest_feature);
        }

        //    time_assign = toc("muted"); tic("nsec");
        //    size_t old_size, new_size;
        //    old_size = oldest_feature->size();
        oldest_feature->ForceThinning(ratio_limit_, (3.0 * oldest_feature->last_thinning_size())/4.0);
        //  new_size = oldest_feature->size();
        //  time_thinning = toc("muted"); tic("nsec");
        //  std::cout << "Feature: " << time_feature_set << " \tMerge:" << time_merge<< " \tAssign:" << time_assign << " \tThinning: "<< "("<< old_size<<","<< new_size << ") " << time_thinning
        //            << " \tTotal: " << time_feature_set + time_merge + time_assign + time_thinning << std::endl;


    }

    TEMPT
    inline void Elised<ROWS,COLS>::UpdateGlobalBuffer(EventNode* const &event_node) {
        global_buffer_.push_back(event_node);
//  while(global_buffer_.size() > max_buffer_) {
//    std::cout << "\t Remove: (" << global_buffer_.front()->data_.x << ","
//              <<global_buffer_.front()->data_.y << ")... ";
//    if (global_buffer_.front()->is_associated()) {
//      this->RemoveEvent(global_buffer_.front());
//      std::cout << "OK"<<std::endl;
//    } else {
//      std::cout << "NOT"<<std::endl;
//    }
//    global_buffer_.front()->data_.orientation.clear();
//    global_buffer_.pop_front();
//  }
//  if(global_buffer_.size() <= max_buffer_) {std::cout << std::endl;}
        if (buffer_size_>max_buffer_) {
            if (global_buffer_.front()->is_associated()) {
                this->RemoveEvent(global_buffer_.front());
            }
            global_buffer_.front()->data_.orientation.clear();
            global_buffer_.pop_front();
        } else {
            buffer_size_++;
        }

    }
    TEMPT
    inline bool Elised<ROWS,COLS>::SetEventOrientation(EventNode* const &event_node) {
        Eigen::Array<EventTime, MASK_SIZE,MASK_SIZE> tmp_block;
        assert(MASK_SIZE==3); // This expression is only valid for 3x3 sobel filter
        tmp_block = ((time_array_).template
                block<MASK_SIZE,MASK_SIZE>(event_node->data_.x-1, //ToDo(herbstc): Add handling of edge cases, when data_.x/y - n < 0
                                           event_node->data_.y-1));

        event_node->data_.orientation.SetComponents(
                (tmp_block.cast<EventTime>()*saliency_mask_x_).sum(),
                (tmp_block.cast<EventTime>()*saliency_mask_y_).sum());

        return event_node->data_.orientation.isValid();
    }


    TEMPT
    inline void Elised<ROWS,COLS>::SimilarNeighbours(EventNode* const &node_root,
                                                     std::list<EventNode*>& candidate_list) {
        assert(node_root->data_.orientation.isValid() &&
               node_root->data_.orientation.isNormBounded()); // The orientation has to be properly configured before manipulate others
        assert(candidate_list.size() == 0); // Candidate list must be empty

//  std::cout <<"Similar neighbors rooted " << node_root->key << std::endl;
        // TODO(ialzugaray): candidate list could have its space preallocated
        candidate_list.push_back(node_root);

        // Unrolled loop
        EventNode* node_neigh =
                this->event_node_array_(node_root->data_.x-1 ,node_root->data_.y-1);
        if (isSimilar(node_root, node_neigh)){
            candidate_list.push_back(node_neigh);
        }

        node_neigh = this->event_node_array_(node_root->data_.x-1,
                                             node_root->data_.y);
        if (isSimilar(node_root, node_neigh)){
            candidate_list.push_back(node_neigh);
        }

        node_neigh = this->event_node_array_(node_root->data_.x-1,
                                             node_root->data_.y+1);
        if (isSimilar(node_root, node_neigh)){
            candidate_list.push_back(node_neigh);
        }

        node_neigh = this->event_node_array_(node_root->data_.x,
                                             node_root->data_.y-1);
        if (isSimilar(node_root, node_neigh)){
            candidate_list.push_back(node_neigh);
        }

        node_neigh = this->event_node_array_(node_root->data_.x,
                                             node_root->data_.y+1);
        if (isSimilar(node_root, node_neigh)){
            candidate_list.push_back(node_neigh);
        }

        node_neigh = this->event_node_array_(node_root->data_.x+1,
                                             node_root->data_.y-1);
        if (isSimilar(node_root, node_neigh)){
            candidate_list.push_back(node_neigh);
        }

        node_neigh = this->event_node_array_(node_root->data_.x+1,
                                             node_root->data_.y);
        if (isSimilar(node_root, node_neigh)){
            candidate_list.push_back(node_neigh);
        }

        node_neigh = this->event_node_array_(node_root->data_.x+1,
                                             node_root->data_.y+1);

        if (isSimilar(node_root, node_neigh)){
            candidate_list.push_back(node_neigh);
        }
    }



    TEMPT
    SimilarityStatus Elised<ROWS,COLS>::isSimilar(const EventNode* const &node_root,
                                                  const EventNode* const &node_neigh) {
        assert(!node_root->is_associated()); // The root should not be associated to any feature

        if (!node_neigh->is_associated()) { // Not associated to any feature
            // Use own orientation
//    assert(event_info_neigh->orientation->isValid()); // It can be invalid, e.g. not yet initialized
            assert(node_neigh->data_.orientation.isNormBounded());
            if (std::abs(node_root->data_.orientation.Similarity(node_neigh->data_.orientation))>=similarity_threshold_) {
                return kNodeSimilar;
            } else {
                return kNotSimilar;
            }
        } else {
            // Use cluster orientation
//    OrientationVector orientation = event_info_neigh->feature->orientation;
//    std::cout << "Is similar to ? "<< event_info_neigh->feature->id() << "("<<event_info_neigh->feature->orientation().dx << ", " << event_info_neigh->feature->orientation().dy << ")"<< std::endl;
            assert(node_neigh->data_.tree->parent_tree()->orientation().isValid());
            assert(node_neigh->data_.tree->parent_tree()->orientation().isNormBounded());
            if (std::abs(node_root->data_.orientation.Similarity(node_neigh->data_.tree->parent_tree()->orientation()))>=similarity_threshold_) {
                return kTreeSimilar;
            } else {
                return kNotSimilar;
            }
        }

    }


    TEMPT
    inline bool Elised<ROWS,COLS>::MergeCandidates(std::list<EventNode*>& candidate_list) {

//  uint64_t time_feature_set = 0, time_initialize=0,
//      time_merge = 0, time_assign = 0, time_thinning = 0;
//  tic("nsec");
        // Remove candidates belonging to initialized features
        std::unordered_set<EventTree*> feature_set;
        std::list<EventNode*>::iterator it = candidate_list.begin();
        // Todo(ialzugaray): this could be done during the similarity check -> we are repeating processing
        while(it!=candidate_list.end()) {
            if ((*it)->is_associated()) {
                assert((*it)->data_.tree->parent_tree()->is_top_tree());
                feature_set.insert((*it)->data_.tree->parent_tree()); // Only use top level features (parents)
                it = candidate_list.erase(it);
            } else {
                it++;
            }
        }

//  time_feature_set = toc("muted"); tic("nsec");

        // Try to initialize a new feature if no neighbor features and enough candidates
        if (feature_set.size()==0) {
            if (candidate_list.size()>=min_candidate_cluster_) {
                candidate_list.sort(EventNode::is_older);
                EventTime oldest_timestamp = candidate_list.front()->data_.timestamp;
                // Create a new cluster with candidates
                this->mutex_feature_stack_.lock(); // Todo(ialzugaray): this lock should guard only feature instead of the complete set
                EventTree* feature = this->InitializeFeature();
                for (EventNode* candidate_node: candidate_list) {

                    this->AssignEvent(candidate_node, feature);
                    if (candidate_node->data_.timestamp < oldest_timestamp ){
                        oldest_timestamp  = candidate_node->data_.timestamp;
                    }
//        std::cout << candidate_node->data_.tree << " | "<< feature <<std::endl;
                }
                this->mutex_feature_stack_.unlock();
                feature->set_age(oldest_timestamp);


//      time_initialize = toc("muted"); tic("nsec");
//      std::cout << "Feature: " << time_feature_set << " \tInitialize:" << time_initialize
//                << " \tTotal: " << time_feature_set + time_initialize << std::endl;
//      feature->RefreshLineData();
                return false;
            } else {
                // Not enough candidates to intialize a cluster
                return false;
            }
        }

        // Merge to the oldest cluster


        EventTree* oldest_feature = nullptr;
        EventTime oldest_feature_time;

        // Get oldest cluster
        for (EventTree* feature: feature_set) { // Todo(ialzugaray): the loop should be change so it doesnt copy the ptr
            if (feature->age()<oldest_feature_time || oldest_feature == nullptr) {
                oldest_feature = feature;
                oldest_feature_time = feature->age();
            }
        }
        int deleted_features = feature_set.erase(oldest_feature);
        assert(deleted_features ==1); // Assert the oldest feature is actually erased
        assert (oldest_feature != nullptr); // oldest feature is non existent

//  time_merge = toc("muted"); tic("nsec");

        // Merge others cluster to oldest cluster
        for (EventTree* feature: feature_set) { // Todo(ialzugaray): the loop should be change so it doesnt copy the ptr
            assert(feature->is_top_tree());
            oldest_feature->Merge(feature);

//    if(feature->is_removable()) { // Merged feature is no longer relevant (empty)
//      this->FreeFeature(feature);
//    }

        }


        // Merge candidates to oldest cluster
        candidate_list.sort(EventNode::is_newer);
        for (EventNode* candidate_info: candidate_list) {
            candidate_info->data_.timestamp =  oldest_feature->newest_timestamp(); // TOdo(fake as all the candidates has new timestamp according to the newest added event)
            this->AssignEvent(candidate_info, oldest_feature);
        }

//    time_assign = toc("muted"); tic("nsec");



//    size_t old_size, new_size;
//    old_size = oldest_feature->size();
        oldest_feature->ForceThinning(ratio_limit_, (3.0 * oldest_feature->last_thinning_size())/4.0);
//  new_size = oldest_feature->size();
//  time_thinning = toc("muted"); tic("nsec");


//  std::cout << "Feature: " << time_feature_set << " \tMerge:" << time_merge<< " \tAssign:" << time_assign << " \tThinning: "<< "("<< old_size<<","<< new_size << ") " << time_thinning
//            << " \tTotal: " << time_feature_set + time_merge + time_assign + time_thinning << std::endl;
        return true;
    }



} // namespace event_tracker
#endif // ELISED_HPP
