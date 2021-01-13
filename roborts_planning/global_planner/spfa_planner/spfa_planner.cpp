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
			distance_cost_parameter_ = spfa_planner_config.distance_cost_parameter();
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

		//get_map
  		gridmap_width_ = costmap_ptr_->GetCostMap()->GetSizeXCell();
  		gridmap_height_ = costmap_ptr_->GetCostMap()->GetSizeYCell();
  		ROS_INFO("Search in a map %d", gridmap_width_*gridmap_height_);
		cost_ = costmap_ptr_->GetCostMap()->GetCharMap();
		// copy the data from cost_ to s (only record obstacle information)
		for (int i=0; i<gridmap_height_; i++) {
    		for (int j=0; j<gridmap_width_; j++) {
    			s[j][i] =  costmap_ptr_->GetCostMap()->GetCost(j, i);
			}
		}
		unsigned int start_x_tmp, start_y_tmp, goal_x_tmp, goal_y_tmp;
		costmap_ptr_->GetCostMap()->Index2Cells(start_index, start_x_tmp, start_y_tmp);
		costmap_ptr_->GetCostMap()->Index2Cells(goal_index, goal_x_tmp, goal_y_tmp);
		start_x_= start_x_tmp;
		start_y_ = start_y_tmp;
		goal_x_ = goal_x_tmp;
		goal_y_ = goal_y_tmp;

		Init();
		SPFA();
		if (!FindAPath()) {
			ROS_WARN("Global planner cannot search the valid path! ");
			return ErrorInfo(ErrorCode::GP_PATH_SEARCH_ERROR,  "Cannot find a path to current goal. ");
		}
		if (!	Smooth(path)) {
			ROS_WARN("Global planner cannot smooth the valid path! ");
			return ErrorInfo(ErrorCode::GP_PATH_SEARCH_ERROR,  "Smooth path failedl. ");
		}

  		return ErrorInfo::OK();

	}


	void SPFAPlanner::Init() {
    	c[0].first=c[2].second=1;
    	c[1].first=c[3].second=-1;

		int l=0,r=0;
    	for (int i=0; i<=gridmap_height_+1; i++){
    		for (int j=0; j<=gridmap_width_+1; j++){
            	value[i][j]=flag[i][j]=0;
            	if (s[i][j]!=roborts_costmap::FREE_SPACE){
                	flag[i][j]=1;
                	seq[++r]= std::make_pair(i,j);
            	}
        	}
		}

    	while (l<r){
        	l++;
			for (int i=0;i<4;i++){
            	dd.first = seq[l].second+c[i].second;
							dd.second =seq[l].second +c[i].second;
            	if (dd.first<0||dd.second<0)continue;
            	if (s[dd.first][dd.second]==roborts_costmap::FREE_SPACE &&!flag[dd.first][dd.second]){
                	value[dd.first][dd.second]=value[seq[l].first][seq[l].second]+1;
                	seq[++r]=dd;
					flag[dd.first][dd.second]=1;
            	}
        	}
    	}

    	for (int i=1; i<=gridmap_height_; i++){
    		for (int j=1; j<=gridmap_width_; j++) {
    	    	value[i][j]=1+distance_cost_parameter_/value[i][j];
			}
		}

	}


    void SPFAPlanner::SPFA() {
		int l=0,r=1;
		seq[r] = std::make_pair(start_x_, start_y_);
    	for (int i=1; i<=gridmap_height_; i++) {
        	for (int j=1; j<=gridmap_width_; j++) {
        		f[i][j]=1e15;
			}
		}

    	std::memset(flag,0,sizeof(flag));
    	std::memset(ff,0,sizeof(ff));
		std::memset(last,0,sizeof(last));
    	f[goal_x_][goal_y_]=0;

    	while (l<r&&r<5*map_height_max_*map_width_max_-4) {
        	l++;
			for (int i=0;i<4;i++) {
            	dd.first = seq[l].first +c[i].first;
							dd.second =seq[l].second + c[i].second;
            	if (s[dd.first][dd.second]==roborts_costmap::FREE_SPACE &&f[dd.first][dd.second]+eps>
                                             f[seq[l].first][seq[l].second]+value[seq[l].first][seq[l].second]){
                	f[dd.first][dd.second]=f[seq[l].first][seq[l].second]+value[seq[l].first][seq[l].second];
                	last[dd.first][dd.second]=seq[l];
                	ff[dd.first][dd.second]=1;
                	if (!flag[dd.first][dd.second]) {
                    	seq[++r]=dd;
						flag[dd.first][dd.second]=1;
                	}
            	}
        	}
			flag[seq[l].first][seq[l].second]=0;
    	}

	}


	bool SPFAPlanner::Smooth(std::vector<geometry_msgs::PoseStamped> &path) {
		path.clear();
		geometry_msgs::PoseStamped iter_pos;

		int x=0,y=0;
		int xx[map_height_max_*map_width_max_];
		int yy[map_height_max_*map_width_max_];
		int now_x=z[1].first,now_y=z[1].second;
		int dd=0;
    	//cout<<now_x<<' '<<now_y<<endl;
    	for (int i=2;i<=d;i++){
        	if (z[i].first-z[i-1].first){
            	x+=z[i].first-z[i-1].first;
            	if (y){
                	xx[++dd]=x;
					yy[dd]=y;
					x=0;
					y=0;
                	//cout<<xx[dd]<<' '<<yy[dd]<<endl;
            	}
        	}
			if (z[i].second-z[i-1].second){
            	y+=z[i].second-z[i-1].second;
            	if (x){
                	xx[++dd]=x;
					yy[dd]=y;
					x=y=0;
                	//cout<<xx[dd]<<' '<<yy[dd]<<endl;
            	}
        	}
    	}
		if (x!=0 or y!=0){
        	xx[++dd]=x;yy[dd]=y;x=y=0;
    	}
    	xx[dd+1]=yy[dd+1]=1e9;
    	for (int i=1;i<=dd;){
        	// cout<<now_x<<' '<<now_y<<endl;
        	int j=i,dx=0,dy=0;
        	for (;xx[j]==xx[i]&&yy[j]==yy[i];j++) dx+=xx[j],dy+=yy[j];
        	now_x+=dx;now_y+=dy;
        	//write(now_x);putchar(' ');write(now_y);putchar(' ');puts("This is new route.");
        	i=j;

    		costmap_ptr_->GetCostMap()->Map2World(now_x, now_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
    		path.push_back(iter_pos);
        	//zz[++top_zz]=make_pair(now_x,now_y);
    	}
		for (int i=1;i<=dd;i++)xx[i]+=xx[i-1],yy[i]+=yy[i-1];

		return true;
	}


	void SPFAPlanner::Dfs(int x,int y) {//to get the path
    	if (last[x][y].first!=0)
        	Dfs(last[x][y].first,last[x][y].second);
    	z[++d]= std::make_pair(x,y);
	}


	void SPFAPlanner::SetValue(int upper, int lower, int left, int right, double new_value) {
    	for (int i=upper; i<=lower; i++) {
	        for (int j=left; j<=right; j++) {
						value[i][j]=new_value;
					}
				}
	}

	bool SPFAPlanner::FindAPath() {
    	if (ff[goal_x_][goal_y_]) {
        	d=0;
			Dfs(goal_x_,goal_y_);
			return true;
    	}
		return false;
    //  for (int i=1;i<=n;i++){
    //     for (int j=1;j<=m;j++)write(f[i][j]),putchar(' ');
    //     puts("");
    }

}
