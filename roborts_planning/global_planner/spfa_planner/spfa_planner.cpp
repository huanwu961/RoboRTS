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
  		ROS_WARN("Map size: %d * %d", gridmap_width_, gridmap_height_);
		ROS_INFO("Search in a map %d", gridmap_width_*gridmap_height_);
		cost_ = costmap_ptr_->GetCostMap()->GetCharMap();

		int d;
	 	bool flag[map_height_max_][map_width_max_];
		bool ff[map_height_max_][map_width_max_];
		double f[map_height_max_][map_width_max_];
		double value[map_height_max_][map_width_max_];
		std::pair<int,int> last[map_height_max_][map_width_max_];
		std::pair<int,int> c[4];
		std::pair<int,int> z[map_height_max_*map_width_max_];

		unsigned int start_x, start_y, goal_x, goal_y;
		costmap_ptr_->GetCostMap()->Index2Cells(start_index, start_x, start_y);
		costmap_ptr_->GetCostMap()->Index2Cells(goal_index, goal_x, goal_y);

		Init(d, flag, f, ff, value, last, c);
		SPFA(start_x, start_y, goal_x, goal_y, d, flag, f, ff, value, last, c);
		if (!FindAPath(goal_x, goal_y, d, ff, last, z)) {
			ROS_WARN("Global planner cannot search the valid path [spfa_planner.cpp 147] cost:%d goal_x:%d goal_y:%d start_x:%d start_y:%d", costmap_ptr_->GetCostMap()->GetCost(goal_x, goal_y), goal_x, goal_y, start_x, start_y);
			return ErrorInfo(ErrorCode::GP_PATH_SEARCH_ERROR,  "Cannot find a path to current goal. ");
		}
		ROS_WARN("[spfa_planner.cpp 150]");
		if (!Smooth(path, d, z)) {
			ROS_WARN("Global planner cannot smooth the valid path [spfa_planner.cpp 152] ");
			return ErrorInfo(ErrorCode::GP_PATH_SEARCH_ERROR,  "Smooth path failedl. ");
		}

  		return ErrorInfo::OK();

	}


	void SPFAPlanner::Init(int &d,
		bool flag[map_height_max_][map_width_max_],
		double f[map_height_max_][map_width_max_],
		bool ff[map_height_max_][map_width_max_],
		double value[map_height_max_][map_width_max_],
		std::pair<int, int> last[map_height_max_][map_width_max_],
		std::pair<int, int> c[4]) {

    	c[0]=std::make_pair(1,0);
    	c[1]=std::make_pair(-1,0);
    	c[2]=std::make_pair(0,1);
    	c[3]=std::make_pair(0,-1);

		int l=0,r=0;
		std::pair<int, int> dd;
		std::pair<int, int> seq[map_height_max_*map_width_max_*5];	
    	for (int i=1; i<=gridmap_height_; i++){
    		for (int j=1; j<=gridmap_width_; j++){
            	value[i][j]=eps;
		flag[i][j]=0;
            	if (costmap_ptr_->GetCostMap()->GetCost(j, i)>= inaccessible_cost_){
                	flag[i][j]=1;
			seq[++r]= std::make_pair(i,j);
            	}
        	}
		}

    	while (l<r){
        	l++;//ROS_WARN("l_init:%d",l)	;
			for (int i=0;i<4;i++){
            	dd.first = seq[l].first+c[i].first;
				dd.second =seq[l].second +c[i].second;
            	if (dd.first<=0||dd.second<=0||dd.first>gridmap_width_||dd.second>gridmap_height_)continue;
            	if (costmap_ptr_->GetCostMap()->GetCost(dd.second, dd.first)< inaccessible_cost_ &&!flag[dd.first][dd.second]){
                	value[dd.first][dd.second]=value[seq[l].first][seq[l].second]+1;
			//ROS_WARN("dd:%d,%d,value_dd:%lf",dd.first,dd.second,value[dd.first][dd.second])	;
                	seq[++r]=dd;
					flag[dd.first][dd.second]=1;
            	}
        	flag[seq[l].first][seq[l].second] = 0;
			}
    	}

    	for (int i=1; i<=gridmap_height_; i++){
    			for (int j=1; j<=gridmap_width_; j++) {
    	    		value[i][j]=1+distance_cost_parameter_/value[i][j];
			}
		}

	}


    void SPFAPlanner::SPFA(const unsigned int &start_x, 
		const unsigned int &start_y,
		const unsigned int &goal_x,
		const unsigned int &goal_y,
		int &d,
		bool flag[map_height_max_][map_width_max_],
		double f[map_height_max_][map_width_max_],
		bool ff[map_height_max_][map_width_max_],
		double value[map_height_max_][map_width_max_],
		std::pair<int, int> last[map_height_max_][map_width_max_],
		std::pair<int, int> c[4]) {
			
		int l=0,r=1;
		std::pair<int,int> seq[map_height_max_*map_width_max_*5];
		std::pair<int,int> dd;
		seq[r] = std::make_pair(start_x, start_y);
    	for (int i=1; i<=gridmap_height_; i++) {
        	for (int j=1; j<=gridmap_width_; j++) {
        		f[i][j]=1e15;
			}
		}

		for (int i=0; i<map_height_max_; i++) {
			for (int j=0; j<map_width_max_; j++) {
				flag[i][j] = 0;
				ff[i][j]= 0;
				last[i][j] = std::make_pair(0, 0);
			}
		} //需要标记到最大表还是地图大小即可？和上面的循环合并？ 
		
    	f[start_x][start_y]=0;
    	ff[start_x][start_y]=1;

    	while (l<r) {
        	l++;
			if (l==map_height_max_*map_width_max_*5-4) l=1;
			bool moved = true;
			for (int i=0;i<4;i++) {
            	dd.first = seq[l].first +c[i].first;
		dd.second =seq[l].second+c[i].second;
            	//ROS_WARN("dd:%d %d, dd_value:%lf",dd.second, dd.first, value[dd.first][dd.second] );
		if (dd.first<=0||dd.second<=0||dd.first>gridmap_width_||dd.second>gridmap_height_) continue; //出界 继续搜索其他方向 
            	//if (costmap_ptr_->GetCostMap()->GetCost(dd.second-1, dd.first-1)<inaccessible_cost_ && flag[seq[l].first][seq[l].second == 0]) {
		if (costmap_ptr_->GetCostMap()->GetCost(dd.second, dd.first)< inaccessible_cost_ && f[dd.first][dd.second]>
                                           f[seq[l].first][seq[l].second]+value[seq[l].first][seq[l].second]){ //不是障碍物 & 通过当前路径代价更小 
			ROS_WARN("DD:%d %d, dd_value:%lf", dd.second, dd.first, value[dd.first][dd.second]);
			f[dd.first][dd.second]=f[seq[l].first][seq[l].second]+value[seq[l].first][seq[l].second];
                	last[dd.first][dd.second]=seq[l];
                	ff[dd.first][dd.second]=1;
                	if (!flag[dd.first][dd.second]) { //点dd从栈顶入栈（如果不在栈中） 
						r++;
						if (r==map_height_max_*map_width_max_*5-4)r=1;
                    		seq[r]=dd;
						flag[dd.first][dd.second]=1;
						moved = true;
                	}
            	}
        	}
			if (!moved) ROS_WARN("the point is not moved? %d", moved);
			flag[seq[l].first][seq[l].second]=0; //栈底出栈 
    	}
			//ROS_WARN("l:%d r:%d", l, r);
	}


	bool SPFAPlanner::Smooth(std::vector<geometry_msgs::PoseStamped> &path,
		int &d,
		std::pair<int, int> z[map_height_max_*map_width_max_]) {

		geometry_msgs::PoseStamped iter_pos;
		iter_pos.pose.orientation.w = 1;
		iter_pos.header.frame_id = "map";
		path.clear();
		int x=0,y=0;
		int xx[map_height_max_*map_width_max_];
		int yy[map_height_max_*map_width_max_];
		int now_x=z[1].first,now_y=z[1].second;
		int dd=0;
		
	//ROS_WARN("[spfa_planner.cpp 272]");
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
			ROS_WARN("[spfa_planner.cpp 299],%d,%d",dd,d);
    	for (int i=1;i<=dd;){
        	// cout<<now_x<<' '<<now_y<<endl;
        	int j=i,dx=0,dy=0;
					ROS_WARN("[spfa_planner.cpp 303]");
        	for (;xx[j]==xx[i]&&yy[j]==yy[i];j++) dx+=xx[j],dy+=yy[j];
        	now_x+=dx;now_y+=dy;
        	//write(now_x);putchar(' ');write(now_y);putchar(' ');puts("This is new route.");
        	i=j;
					ROS_WARN("[spfa_planner.cpp 306]");
    		costmap_ptr_->GetCostMap()->Map2World(now_x, now_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
    		path.push_back(iter_pos);
        	//zz[++top_zz]=make_pair(now_x,now_y);
    	}
		for (int i=1;i<=dd;i++)xx[i]+=xx[i-1],yy[i]+=yy[i-1];
		return true;
	}


	void SPFAPlanner::Dfs(int x,int y,int &d,
		std::pair<int, int> last[map_height_max_][map_width_max_],
		std::pair<int, int> z[map_height_max_*map_width_max_]) {//to get the path
    	if (last[x][y].first!=0)
        	Dfs(last[x][y].first,last[x][y].second, d, last, z);
    	z[++d]= std::make_pair(x,y);//ROS_WARN("x,y:%d,%d",x, y);
	}


	/*void SPFAPlanner::SetValue(int upper, int lower, int left, int right, double new_value) {
    	for (int i=upper; i<=lower; i++) {
	        for (int j=left; j<=right; j++) {
						value[i][j]=new_value;
					}
				}
	}*/

	bool SPFAPlanner::FindAPath(const unsigned int &goal_x,
			const unsigned int &goal_y,
			int &d,
			bool ff[map_height_max_][map_width_max_],
			std::pair<int, int> last[map_height_max_][map_width_max_],
			std::pair<int, int> z[map_height_max_*map_width_max_]) {
				
    	if (ff[goal_x][goal_y]) {
        	
		d=0;
			Dfs(goal_x,goal_y, d, last, z);
			return true;
    	}
	ROS_WARN("ff goal: %d\n size: %d %d", ff[goal_x][goal_y],gridmap_width_, gridmap_height_);
		return false;
    //  for (int i=1;i<=n;i++){
    //     for (int j=1;j<=m;j++)write(f[i][j]),putchar(' ');
    //     puts("");
	}

}
