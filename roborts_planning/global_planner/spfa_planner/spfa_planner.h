#ifndef ROBORTS_PLANNING_GLOBAL_PLANNER_SPFA_PLANNER_H
#define ROBORTS_PLANNING_GLOBAL_PLANNER_SPFA_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "proto/spfa_planner_config.pb.h"

#include "alg_factory/algorithm_factory.h"
#include "state/error_code.h"
#include "costmap/costmap_interface.h"

#include "../global_planner_base.h"

namespace roborts_global_planner{
	class SPFAPlanner : public GlobalPlannerBase {
		public:
			SPFAPlanner(CostmapPtr costmap_ptr);
			virtual ~SPFAPlanner();
			/**
   * @brief Main Plan function(override the base-class function)
   * @param start Start pose input
   * @param goal Goal pose input
   * @param path Global plan path output
   * @return ErrorInfo which is OK if succeed
   */
  		roborts_common::ErrorInfo Plan(const geometry_msgs::PoseStamped &start,
                               const geometry_msgs::PoseStamped &goal,
                               std::vector<geometry_msgs::PoseStamped> &path);

		private:
			/**
   		* @brief Plan based on 1D Costmap list. Input the index in the costmap and get the plan path.
   		* @param start_index start pose index in the 1D costmap list
   		* @param goal_index goal pose index in the 1D costmap list
   		* @param path plan path output
   		* @return ErrorInfo which is OK if succeed
   		*/
  		roborts_common::ErrorInfo SearchPath(const int &start_index,
                                     const int &goal_index,
                                     std::vector<geometry_msgs::PoseStamped> &path);
	    	void Init();

		void SPFA();

	 	void Smooth(std::vector<geometry_msgs::PoseStamped> &path);

		void Dfs(int x,int y);

		bool FindAPath();

		/*std::pair<int,int> SPFAPlanner::operator+(const std::pair<int,int> &y) {
			std::pair<int,int> sum;
			sum.first = first + y.first;
			sum.second = second + y.second;
			return sum;
		}*/

		void SetValue(int upper, int lower, int left, int right, double value);

		//! heuristic_factor_
  		float heuristic_factor_;
  		//! inaccessible_cost
  		unsigned int inaccessible_cost_;
  		//! goal_search_tolerance
  		unsigned int goal_search_tolerance_;
  		//! maximum height size
  		static const unsigned int map_height_max_ = 110; //(N)
  		//! maximum width size
  		static const unsigned int map_width_max_ = 210; //(M)
  		
  		unsigned int distance_cost_parameter_; //(C)
  		//! gridmap height size
  		unsigned int gridmap_height_; //(n)
  		//! gridmap width size
  		unsigned int gridmap_width_; //(m)
  		//! gridmap cost array
  		unsigned char *cost_;
  		//! 2d costmap array
  		char s[map_height_max_][map_width_max_];

		static constexpr double eps = 1e-5; 		
		int d; 
		int start_x_; 
		int start_y_; 
		int goal_x_; 
		int goal_y_; //(d, x, y, xx, yy)
		bool flag[map_height_max_][map_width_max_];
		bool ff[map_height_max_][map_width_max_];
		double f[map_height_max_][map_width_max_]; 
		double value[map_height_max_][map_width_max_];
		std::pair<int,int> seq[map_height_max_*map_width_max_*5]; 
		std::pair<int,int> last[map_height_max_][map_width_max_]; 
		std::pair<int,int> c[4];
		std::pair<int,int> dd;
		std::pair<int,int> z[map_height_max_*map_width_max_];
	};

//std::vector<int> SPFAPlanner::f_score_;
roborts_common::REGISTER_ALGORITHM(GlobalPlannerBase,
                                 "spfa_planner",
                                 SPFAPlanner,
                                 std::shared_ptr<roborts_costmap::CostmapInterface>);


}

#endif
