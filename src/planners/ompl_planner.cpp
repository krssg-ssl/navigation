#include "ros/ros.h"
#include "planners/ompl_planner.h"

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;
using namespace krssg_ssl_msgs;


Planner::Planner(krssg_ssl_msgs::path_data::Request &request) {
	start_point  = request.botPos;
	final_point  = request.targetPos;
	numObstacles = request.obs_size;
	botID        = request.botID;

	for(int obs=0; obs<numObstacles; ++obs) {
		obstacles.push_back(request.obs_vec[obs]);
	}
}

Planner::~Planner() {

}

bool Planner::isStateValid(const ob::State *state) {
  const ob::SE2StateSpace::StateType *state_2d= state->as<ob::SE2StateSpace::StateType>();
  const double &x(state_2d->getX()), &y(state_2d->getY());

  for (int i = 0; i < numObstacles; ++i){
    if (sqrt(pow((obstacles[i].x - x), 2)+pow((obstacles[i].y - y), 2)) <= 10.0){
      return false;
    }
  }
	return true;
}

void Planner::plan() {
	ob::StateSpacePtr space(new ob::SE2StateSpace());
	ob::RealVectorBounds bounds(2);
	bounds.setLow(0, -HALF_FIELD_MAXX);
	bounds.setHigh(0, -HALF_FIELD_MAXY);
	bounds.setLow(1, HALF_FIELD_MAXX);
	bounds.setHigh(1, HALF_FIELD_MAXY);
	space->as<ob::SE2StateSpace>()->setBounds(bounds);

	og::SimpleSetup setup(space);
	setup.setStateValidityChecker(boost::bind(&Planner::isStateValid, this, _1));

	ob::ScopedState<ob::SE2StateSpace> start(space);
  	start->setXY(start_point.x, start_point.y);

	ob::ScopedState<ob::SE2StateSpace> final(space);
  	final->setXY(final_point.x, final_point.y);

  	setup.setStartAndGoalStates(start, final);

  	ob::PlannerPtr rrtc_planner(new og::RRTConnect(setup.getSpaceInformation()));
   setup.setPlanner(rrtc_planner);

   ob::PlannerStatus solved = setup.solve();

   if (solved) {
   	ompl::geometric::PathGeometric final_path = setup.getSolutionPath();

   	std::vector<ompl::base::State *> inter_states = final_path.getStates();

   	geometry_msgs::Pose2D temp_point;
   	for(std::vector<ompl::base::State *>::iterator itr =
   		inter_states.begin(); itr != inter_states.end(); ++itr) {
   		const ob::SE2StateSpace::StateType *inter_instance = (*itr)->as<ob::SE2StateSpace::StateType>();
   		temp_point.x = inter_instance->getX();
   		temp_point.y = inter_instance->getY();
   		response.control_points.push_back(temp_point);
   	}
   } else {
   	ROS_INFO("No path exists !");
   }
}


bool path_generator(krssg_ssl_msgs::path_data::Request &Request,
						  krssg_ssl_msgs::path_data::Response &Reponse) {
	ROS_INFO("Generating a new path for botID %d", Request.botID);

	Planner planner(Request);
	planner.plan();
	
	Reponse.control_points = planner.response.control_points;
	return true;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "path_planner_server");
	ros::NodeHandle nh;

	ros::ServiceServer service = nh.advertiseService("path_planner", path_generator);

	ros::spin();
	return 0;
}
