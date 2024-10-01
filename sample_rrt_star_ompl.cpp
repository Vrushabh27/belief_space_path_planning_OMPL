#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/config.h>
#include <iostream>

// Namespace shortcuts
namespace ob = ompl::base;
namespace og = ompl::geometric;

// State validity checker for the space (returns true for all states in this example)
bool isStateValid(const ob::State *state)
{
    // In this simple example, all states are valid.
    // You can add obstacle checking here if needed.
    return true;
}

int main()
{
    // Create a 2D state space (x and y)
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);

    // Set the bounds of the space
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);  // Lower bound for both x and y
    bounds.setHigh(1);  // Upper bound for both x and y
    space->setBounds(bounds);

    // Create a SimpleSetup object
    og::SimpleSetup ss(space);

    // Set state validity checking for this space
    ss.setStateValidityChecker(isStateValid);

    // Set the start and goal states
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = -0.5;  // x coordinate
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = -0.5;  // y coordinate

    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.5;  // x coordinate
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.5;  // y coordinate

    ss.setStartAndGoalStates(start, goal);

    // Use the RRT* planner
    auto planner = std::make_shared<og::RRTstar>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    // Attempt to solve the problem within a given time (seconds)
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;

        // Simplify the solution (optional)
        ss.simplifySolution();

        // Print the path to screen
        ss.getSolutionPath().print(std::cout);
    }
    else
    {
        std::cout << "No solution found." << std::endl;
    }

    return 0;
}

