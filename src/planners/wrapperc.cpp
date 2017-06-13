#include "planners/mergescurve.h"
#include "krssg_ssl_msgs/path_data.h"
#include "planners/ompl_planner.h"
#include "geometry_msgs/Pose2D.h"

#include <vector>

/*
	C wrappers for C++ classes:
		* vector<obstacle>
		* Vector2D
		* MergeSCurve
		* OMPL_Planner
*/

using namespace std;
using namespace Navigation;

extern "C"{

	struct OrientedPoint{
		float x, y, theta;
	};

	struct ListOrientedPoint{
		OrientedPoint List[50];
		int size;
	};

	struct ListObstacle{
		obstacle List[13];
		int size;
	};

	vector<obstacle>* _vector_obstaclep_new(void){
		return new vector<obstacle>();
	}
	void _vector_obstaclep_delete(vector<obstacle>* v){
		delete v;
	}
	int _vector_obstaclep_size(vector<obstacle>* v){
		return v->size();
	}
	obstacle* _vector_obstaclep_get(vector<obstacle>* v, int pos){
		return &(v->operator[](pos));
	}
	void _vector_obstaclep_push_back(vector<obstacle>* v, obstacle* ob){
		v->push_back(*ob);
	}

	MergeSCurve* _MergeSCurvep_new(void){
		return new MergeSCurve();
	}
	void _MergeSCurvep_delete(MergeSCurve* mc){
		delete mc;
	}
	bool _MergeSCurvep_plan( MergeSCurve* mc,
				 const Vector2D<int>& initial,
				 const Vector2D<int>& final,
				 Vector2D<int>* pt1,
				 Vector2D<int>* pt2,
				 vector<obstacle>* obs,
				 int obstacle_count,
				 int current_id,
				 bool teamBlue	){
		return mc->plan(initial, final, pt1, pt2, *obs, obstacle_count, current_id, teamBlue);
	}

	OMPL_Planner* _OMPL_Planner_new(OrientedPoint botPos, OrientedPoint targetPos, ListObstacle obs_vec){
		krssg_ssl_msgs::path_data::Request request;
		request.botPos.x = botPos.x;
		request.botPos.y = botPos.y;
		request.botPos.theta = botPos.theta;
		request.targetPos.x = targetPos.x;
		request.targetPos.y = targetPos.y;
		request.targetPos.theta = targetPos.theta;
		for(int i=0;i<obs_vec.size;i++){
			request.obs_vec[i].x = obs_vec.List[i].x;
			request.obs_vec[i].y = obs_vec.List[i].y;
			request.obs_vec[i].radius = obs_vec.List[i].radius;
		}
		request.obs_size = obs_vec.size;
		return new OMPL_Planner(request);
	}
	void _OMPL_Planner_delete(OMPL_Planner* ptr){
		delete ptr;
	}
	ListOrientedPoint& _OMPL_Planner_plan(OMPL_Planner* ptr){
		vector<geometry_msgs::Pose2D> response = (ptr->plan()).control_points;
		ListOrientedPoint l;
		l.size = 0;
		assert(response.size() <= 50);
		for(int i=0;i<response.size();i++){
			l.List[i].x = response[i].x;
			l.List[i].y = response[i].y;
			l.List[i].theta = response[i].theta;
			l.size ++;
		}
		return l;
	}
}
