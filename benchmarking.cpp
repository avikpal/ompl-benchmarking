#include"ompl/tools/benchmark/Benchmark.h"

ompl::base::PlannerPtr myConfiguredPlanner(const ompl::base::SpaceInformationPtr &si)
{
	geometric::EST *est = new ompl::geometric::EST(si);
	est->setRange(100.0);
	return ompl::base::PlannerPtr(est);
}

//Creating a state space for the space we are planning in 

ompl:: geometric::SimpleSetup ss(space);

// Configure the problem to solve: set start state(s)
// and goal representation
// Everything must be set up to the point ss.solve()
// can be called. Setting up a planner is not needed.

//Benchmarking Code starts here:

//First creating a benchmark class:

ompl::tools::Benchmark b(ss,"my experient");

//we add planners to evaluate

b.addPlanner(base::PlannerPtr(new geometric::KPIECE1(ss.getSpaceInformation())));
b.addPlanner(base::PlannerPtr(new geometric::RRT(ss.getSpaceInformation())));
b.addPlanner(base::PlannerPtr(new geometric::SBL(ss.getSpaceInformation())));
b.addPlanner(base::PlannerPtr(new geometric::LBKPIECE1(ss.getSpaceInformation())));

//for planners that we want to configure in specific ways the 
//ompl::base::PlannerAllocator should be used:

b.addPlannerAllocator(boost::bind(&myConfiguredPlanner,_1));
//....

// Now we can benchmark: 5 second time limit for each plan computation,
// 100 MB maximum memory usage per plan computation, 50 runs for each planner
// and true means that a text-mode progress bar should be displayed while
// computation is running.




