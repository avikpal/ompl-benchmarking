bool myStateValidityCheckerFunction(const base::State *state)
{
     return ...;
}
base::SpaceInformationPtr si(space);
si->setStateValidityChecker(base::StateValidityCheckerPtr(new myStateValidityCheckerClass(si)));
si->setStateValidityCheckingResolution(0.03); // 3%
si->setup();

class myMotionValidator : public base::MotionValidator
{
public:
    // implement checkMotion()
};
base::SpaceInformationPtr si(space);
si->setMotionValidator(base::MotionValidatorPtr(new myMotionValidator(si)));
si->setup();
