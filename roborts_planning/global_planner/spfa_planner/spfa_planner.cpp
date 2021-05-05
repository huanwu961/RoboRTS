#include "spfa_planner.h"

namespace roborts_global_planner{

	using roborts_common::ErrorCode;
	using roborts_common::ErrorInfo;

	SPFAPlanner::SPFAPlanner(CostmapPtr costmap_ptr) :
		GlobalPlannerBase::GlobalPlannerBase(costmap_ptr),
    	gridmap_width_(costmap_ptr_->GetCostMap()->GetSizeXCell()),
    	gridmap_height_(costmap_ptr_->GetCostMap()->GetSizeYCell()),
    	cost_(costmap_ptr_->GetCostMap()->GetCharMap()) {
    	//initialize path to get configuration
		SPFAPlannerConfig spfa_planner_config;
  		std::string full_path = ros::package::getPath("roborts_planning") + "/global_planner/spfa_planner/"\
      		"config/spfa_planner_config.prototxt";
			ROS_INFO("spfa_planner.cpp, line 17");
  		if (!roborts_common::ReadProtoFromTextFile(full_path.c_str(),
                                           &spfa_planner_config)) {
    		ROS_ERROR("Cannot load spfa planner protobuf configuration file.");
  		}

  		heuristic_factor_ = spfa_planner_config.heuristic_factor();
  		inaccessible_cost_ = spfa_planner_config.inaccessible_cost();
  		goal_search_tolerance_ = spfa_planner_config.goal_search_tolerance()/costmap_ptr->GetCostMap()->GetResolution();
                max_queue_time_ = spfa_planner_config.max_queue_time();
		//distance_cost_parameter_ = spfa_planner_config.distance_cost_parameter();
    }

    SPFAPlanner::~SPFAPlanner() {
    	cost_ = nullptr;
	}

	ErrorInfo SPFAPlanner::Plan(const geometry_msgs::PoseStamped &start,
                             const geometry_msgs::PoseStamped &goal,
                             std::vector<geometry_msgs::PoseStamped> &path) {

  		unsigned int start_x, start_y, goal_x, goal_y, tmp_goal_x, tmp_goal_y;
  		unsigned int valid_goal[2];
  		unsigned int shortest_dist = std::numeric_limits<unsigned int>::max();
  		bool goal_valid = false;

  		if (!costmap_ptr_->GetCostMap()->World2Map(start.pose.position.x,
                                             	start.pose.position.y,
                                             	start_x,
                                             	start_y)) {
    		ROS_WARN("Failed to transform start pose from map frame to costmap frame");
    		return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                     "Start pose can't be transformed to costmap frame.");
  		}

  		if (!costmap_ptr_->GetCostMap()->World2Map(goal.pose.position.x,
                                             	goal.pose.position.y,
                                             	goal_x,
                                             	goal_y)) {
    		ROS_WARN("Failed to transform goal pose from map frame to costmap frame");
    		return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                     "Goal pose can't be transformed to costmap frame.");
  		}

  		if (costmap_ptr_->GetCostMap()->GetCost(goal_x,goal_y)<inaccessible_cost_){ // The goal is feasible
    		valid_goal[0] = goal_x;
    		valid_goal[1] = goal_y;
    		goal_valid = true;
  		}
		else{
    		tmp_goal_x = goal_x;
    		tmp_goal_y = goal_y - goal_search_tolerance_;

    		while(tmp_goal_y <= goal_y + goal_search_tolerance_){
    			// if the goal is not valid, try to find a new valid goal within a square
    			// with a side length of 2*goal_search_tolerance_ around the original goal
     	 		tmp_goal_x = goal_x - goal_search_tolerance_;
      			while(tmp_goal_x <= goal_x + goal_search_tolerance_){
        			unsigned char cost = costmap_ptr_->GetCostMap()->GetCost(tmp_goal_x, tmp_goal_y);
        			unsigned int dist = abs(int(goal_x - tmp_goal_x)) + abs(int(goal_y - tmp_goal_y));
        			if (cost < inaccessible_cost_ && dist < shortest_dist ) {
          				shortest_dist = dist;
          				valid_goal[0] = tmp_goal_x;
          				valid_goal[1] = tmp_goal_y;
          				goal_valid = true;
        			}
        			tmp_goal_x += 1;
      			}
      			tmp_goal_y += 1;
    		}
  		}

  		ErrorInfo error_info;
  		if (!goal_valid){
    		error_info=ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR);
    		path.clear();
  		}
  		else{
    		unsigned int start_index, goal_index;
    		start_index = costmap_ptr_->GetCostMap()->GetIndex(start_x, start_y);
    		goal_index = costmap_ptr_->GetCostMap()->GetIndex(valid_goal[0], valid_goal[1]);

    		costmap_ptr_->GetCostMap()->SetCost(start_x, start_y, roborts_costmap::FREE_SPACE);

    		if(start_index == goal_index){
      			error_info=ErrorInfo::OK();
      			path.clear();
      			path.push_back(start);
      			path.push_back(goal);
    		}
    		else{
      			error_info = SearchPath(start_index, goal_index, path);
      			if ( error_info.IsOK() ){
        			path.back().pose.orientation = goal.pose.orientation;
        			path.back().pose.position.z = goal.pose.position.z;
      			}
    		}

  		}

  		return error_info;
	}

	ErrorInfo SPFAPlanner::SearchPath(const int &start_index,
                                       const int &goal_index,
                                       std::vector<geometry_msgs::PoseStamped> &path) {

        dis_.clear();
        queue_time_.clear();
        parent_.clear();
        state_.clear();
        gridmap_width_ = costmap_ptr_->GetCostMap()->GetSizeXCell();
        gridmap_height_ = costmap_ptr_->GetCostMap()->GetSizeYCell();
        ROS_INFO("Search in a map %d", gridmap_width_*gridmap_height_);
        cost_ = costmap_ptr_->GetCostMap()->GetCharMap();
        dis_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
        queue_time_.resize(gridmap_height_ * gridmap_width_, 0);
        parent_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
        state_.resize(gridmap_height_ * gridmap_width_, SearchState::NOT_HANDLED);

        //ROS_WARN("max_queue_time: %d", max_queue_time_);
        std::priority_queue<int, std::vector<int>, Compare> openlist;
        dis_.at(start_index) = 0;
        openlist.push(start_index);
        queue_time_.at(start_index) = queue_time_.at(start_index) + 1;

        std::vector<int> neighbors_index;
        int current_index, move_cost, count = 0;

        while ((!openlist.empty()) && (state_.at(goal_index) != SearchState::CLOSED) &&
                                          (queue_time_.at(goal_index) < max_queue_time_)) {
            //ROS_WARN("searching %d", count);
            current_index = openlist.top();
            openlist.pop();
            if (queue_time_.at(current_index) < max_queue_time_) {
                state_.at(current_index) = SearchState::NOT_HANDLED;
            }
            else {
                state_.at(current_index) = SearchState::CLOSED;
            }

            if (current_index == goal_index) {
                ROS_INFO("Search takes %d cycle counts", count);
                break;
            }

            GetEightNeighbors(current_index, neighbors_index);

            for (auto neighbor_index : neighbors_index) {

                if (neighbor_index < 0 ||
                    neighbor_index >= gridmap_height_ * gridmap_width_) {
                    continue;
                }

                if (cost_[neighbor_index] >= inaccessible_cost_ ||
                    state_.at(neighbor_index) == SearchState::CLOSED) {
                    continue;
                }

                GetMoveCost(current_index, neighbor_index, move_cost);

                if (dis_.at(neighbor_index) > dis_.at(current_index) + move_cost + cost_[neighbor_index]) {

                    dis_.at(neighbor_index) = dis_.at(current_index) + move_cost + cost_[neighbor_index];
                    parent_.at(neighbor_index) = current_index;

                    if (state_.at(neighbor_index) == SearchState::NOT_HANDLED) {
                        openlist.push(neighbor_index);
                        queue_time_.at(neighbor_index) = queue_time_.at(neighbor_index) + 1;
                        state_.at(neighbor_index) = SearchState::OPEN;
                    }
                }
            }
            count++;
        }

        if (current_index != goal_index) {
            ROS_WARN("Global planner can't search the valid path!");
            return ErrorInfo(ErrorCode::GP_PATH_SEARCH_ERROR, "Valid global path not found.");
        }

        unsigned int iter_index = current_index, iter_x, iter_y;

        geometry_msgs::PoseStamped iter_pos;
        iter_pos.pose.orientation.w = 1;
        iter_pos.header.frame_id = "map";
        path.clear();
        costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
        costmap_ptr_->GetCostMap()->Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
        path.push_back(iter_pos);

        while (iter_index != start_index) {
            iter_index = parent_.at(iter_index);
//    if(cost_[iter_index]>= inaccessible_cost_){
//      LOG_INFO<<"Cost changes through planning for"<< static_cast<unsigned int>(cost_[iter_index]);
//    }
            costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
            costmap_ptr_->GetCostMap()->Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
            path.push_back(iter_pos);
        }

        std::reverse(path.begin(),path.end());

        return ErrorInfo(ErrorCode::OK);

    }

    ErrorInfo SPFAPlanner::GetMoveCost(const int &current_index,
                                        const int &neighbor_index,
                                        int &move_cost) const {
        if (abs(neighbor_index - current_index) == 1 ||
            abs(neighbor_index - current_index) == gridmap_width_) {
            move_cost = 10;
        } else if (abs(neighbor_index - current_index) == (gridmap_width_ + 1) ||
                   abs(neighbor_index - current_index) == (gridmap_width_ - 1)) {
            move_cost = 14;
        } else {
            return ErrorInfo(ErrorCode::GP_MOVE_COST_ERROR,
                             "Move cost can't be calculated cause current neighbor index is not accessible");
        }
        return ErrorInfo(ErrorCode::OK);
    }

    void SPFAPlanner::GetManhattanDistance(const int &index1, const int &index2, int &manhattan_distance) const {
        manhattan_distance = heuristic_factor_* 10 * (abs(static_cast<int>(index1 / gridmap_width_ - index2 / gridmap_width_)) +
                                                      abs(static_cast<int>((index1 % gridmap_width_ - index2 % gridmap_width_))));
    }

    void SPFAPlanner::GetEightNeighbors(const int &current_index, std::vector<int> &neighbors_index) const {
        neighbors_index.clear();
        if(current_index - gridmap_width_ >= 0){
            neighbors_index.push_back(current_index - gridmap_width_);       //up
        }
        if(current_index - gridmap_width_ - 1 >= 0 && (current_index - gridmap_width_ - 1 + 1) % gridmap_width_!= 0){
            neighbors_index.push_back(current_index - gridmap_width_ - 1); //left_up
        }
        if(current_index - 1 >= 0 && (current_index - 1 + 1) % gridmap_width_!= 0){
            neighbors_index.push_back(current_index - 1);        //left
        }
        if(current_index + gridmap_width_ - 1 < gridmap_width_* gridmap_height_
           && (current_index + gridmap_width_ - 1 + 1) % gridmap_width_!= 0){
            neighbors_index.push_back(current_index + gridmap_width_ - 1); //left_down
        }
        if(current_index + gridmap_width_ < gridmap_width_* gridmap_height_){
            neighbors_index.push_back(current_index + gridmap_width_);     //down
        }
        if(current_index + gridmap_width_ + 1 < gridmap_width_* gridmap_height_
           && (current_index + gridmap_width_ + 1 ) % gridmap_width_!= 0){
            neighbors_index.push_back(current_index + gridmap_width_ + 1); //right_down
        }
        if(current_index  + 1 < gridmap_width_* gridmap_height_
           && (current_index  + 1 ) % gridmap_width_!= 0) {
            neighbors_index.push_back(current_index + 1);                   //right
        }
        if(current_index - gridmap_width_ + 1 >= 0
           && (current_index - gridmap_width_ + 1 ) % gridmap_width_!= 0) {
            neighbors_index.push_back(current_index - gridmap_width_ + 1); //right_up
        }
    }

}
