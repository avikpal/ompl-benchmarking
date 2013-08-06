#include<ompl/base/SpaceInformation.h>
#include<ompl/base/spaces/SE3StateSpace.h>
#include<ompl/geometric/planners/rrt/RRTConnect.h>
#include<ompl/geometric/SimpleSetup.h>
#include<ompl/config.h>
#include<ompl/geometric/planners/rrt/RRT.h>
#include<ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include<ompl/geometric/planners/kpiece/KPIECE1.h>
#include<ompl/geometric/planners/sbl/SBL.h>
#include<ompl/tools/benchmark/Benchmark.h>
#include<iostream>


// A function that matches the ompl::base::PlannerAllocator type.
// It will be used later to allocate an instance of EST
/*ompl::base::PlannerPtr myConfiguredPlanner(const ompl::base::SpaceInformationPtr &si)
{
    ompl::geometric::EST *est = new ompl::geometric::EST(si);
    est->setRange(100.0);
    return ompl::base::PlannerPtr(est);
}*/
// Create a state space for the space we are planning in
// Configure the problem to solve: set start state(s) and goal representation Everything must be set up to the point ss.solve()
// can be called. Setting up a planner is not needed.

bool isStateValid(const ompl::base::State *state)
{
	//cast the abstract state type to the expected type
	const ompl::base::SE3StateSpace::StateType *se3state= state->as<ompl::base::SE3StateSpace::StateType>();
	// extract the first component of the state and cast it to what we expect	
	const ompl::base::RealVectorStateSpace::StateType *pos = se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);
	// extract the second component of the state and cast it to what we expect
	const ompl::base::SO3StateSpace::StateType *rot = se3state->as<ompl::base::SO3StateSpace::StateType>(1);
	// check validity of state defined by pos & rot
        // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
       return (void*)rot != (void*)pos;
}

void benchmark()
      { ompl::base::StateSpacePtr space(new ompl::base::SE3StateSpace());   
       // set the bounds for the R^3 part of SE(3)
       ompl::base::RealVectorBounds bounds(3);
       bounds.setLow(-1);
       bounds.setHigh(1);
       space->as<ompl::base::SE3StateSpace>()->setBounds(bounds);   
      // define a simple setup class
      ompl::geometric::SimpleSetup ss(space);
      // set state validity checking for this space
      ss.setStateValidityChecker(boost::bind(&isStateValid, _1));
      // create a random start state
      ompl::base::ScopedState<> start(space);
      start.random();
      // create a random goal state
      ompl::base::ScopedState<> goal(space);
       goal.random();   
       // set the start and goal states
       ss.setStartAndGoalStates(start, goal);
// First we create a benchmark class:
ompl::tools::Benchmark b(ss, "my experiment");
// We add the planners to evaluate.
b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::KPIECE1(ss.getSpaceInformation())));
b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::RRT(ss.getSpaceInformation())));
b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::SBL(ss.getSpaceInformation())));
b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::LBKPIECE1(ss.getSpaceInformation())));
// etc
// For planners that we want to configure in specific ways,
// the ompl::base::PlannerAllocator should be used:
 //b.addPlannerAllocator(boost::bind(&myConfiguredPlanner, _1));
// etc.
// Now we can benchmark: 5 second time limit for each plan computation,
// 100 MB maximum memory usage per plan computation, 50 runs for each planner
// and true means that a text-mode progress bar should be displayed while
// computation is running.
ompl::tools::Benchmark::Request req;
req.maxTime = 5.0;
req.maxMem = 100.0;
req.runCount = 50;
req.displayProgress = true;
b.benchmark(req);
// This will generate a file of the form ompl_host_time.log
b.saveResultsToFile();}

main(){
      benchmark();
}
