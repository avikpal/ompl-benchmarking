

namespace ob=ompl::base;
namespace og=ompl::geometric;

bool isStateValid(const ob::State *state){};
/*
class myStateValidityCheckerClass: public base::StateValidityChecker{
	public:
		myStateValidityCheckerClass(const base::SpaceInformationPtr &si):
			base::StateValidityChecker(si){
				}
	virtual bool isValid(const base::State *state)const
		{ return ...;
			}
};*/

bool my

void plainWithSimpleSetup(void){
	ob::StateSpacePtr space(new ob:: SE3StateSpace()); //constructing the state space we are planning in
	
	//Setting bounds for the R3 component of this state space
	ob::RealVectorBounds bounds(3);
	bounds.setLow(-1);
	bounds.setHigh(1);
    space->as<ob::SE3StateSpace>()->setBounds(bounds);

    og::SimpleSetup ss(space); //instance of ompl::geometric::SimpleSetup
	ss.setStateValidityChecker(boost::bind(&isStateValid, _1));	

	ob::ScopedState<>start(space);
	start.random();

	ob::ScopedState<>goal(space);
	goal.random();
	
	ss.setStartAndGoalStates(start,goal);
    	
	ob::PlannerStatus solved = ss.solve(1.0);
	
	if(solved){
		std::cout<<"Got the solution"<<endl;
		ss.simplyfySolution();
		ss.getSolutionPath().print(std::cout);	
	}

}
