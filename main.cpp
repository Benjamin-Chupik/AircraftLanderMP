/*
By: Benjamin chupik
For algorithmic motion planning
Main function for final project

State Vector: [x, y, z, x_dot, y_dot, z_do, yaw, pitch, roll, yaw_dot, pitch_dot, roll_dot]
Input Vector: [d_r, d_e, d_a] (ruder, elevator, aileron)

*/

//--------------------------------------------------------------------
// Program Setup
//--------------------------------------------------------------------
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <valarray>
#include <limits>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/Path.h>
#include <ompl/base/StateSpace.h>

#include <tempest.h>
#include <barometric_formula.h>

// Name Spaces
namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

//--------------------------------------------------------------------
// Support Functions
//--------------------------------------------------------------------
bool isStateValid(const ob::State *state)
{
    // TODO: add state validity checking
    return true;
}

//--------------------------------------------------------------------
// Dynamics
//--------------------------------------------------------------------

void TempestODE(const oc::ODESolver::StateType &q, const oc::Control *control, oc::ODESolver::StateType &qdot)
{
    // TODO: hard coded the wind thing
    Eigen::Vector3d wind_inertial{0, 0, 0};

    // TempestODE - This function adapted for c++ from function provided by Professor Eric Frew. Adapted by Roland Ilyes

    // load in tempest aircraft parameters
    struct params ap;

    // cast control to the type we expect
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    // notes: reference u[0] to u[3], elevator, aileron, rudder, throttle

    // Turn relevant states into vectors
    Eigen::Vector3d pos_inertial{q[0], q[1], q[2]};
    Eigen::Vector3d euler_angles{q[3], q[4], q[5]};
    Eigen::Vector3d vel_body{q[6], q[7], q[8]};
    Eigen::Vector3d omega_body{q[9], q[10], q[11]};

    // Kinematics
    Eigen::Vector3d vel_inertial = TransformFromBodyToInertial(vel_body, euler_angles);
    Eigen::Vector3d euler_rates = EulerRatesFromOmegaBody(omega_body, euler_angles);

    // Aerodynamic force and moment
    double density = Formulae::barometricDensity(-pos_inertial[2]);

    Eigen::Vector3d fa_body;
    Eigen::Vector3d ma_body;
    Eigen::Vector3d wind_angles;

    Eigen::Matrix<double, 12, 1> q_as_vec{q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], q[9], q[10], q[11]};
    Eigen::Vector4d u_as_vec{u[0], u[1], u[2], u[3]};

    std::tie(fa_body, ma_body, wind_angles) = AeroForcesAndMoments_BodyState_WindCoeffs(q_as_vec, u_as_vec, wind_inertial, density);

    // Gravity
    Eigen::Vector3d fg_body;

    Eigen::Vector3d grav_vec{(-sin(euler_angles[1])), (sin(euler_angles[0]) * cos(euler_angles[1])), (cos(euler_angles[0]) * cos(euler_angles[1]))};

    fg_body = (ap.g * ap.m) * grav_vec;

    // Dynamics
    Eigen::Vector3d vel_body_dot;

    vel_body_dot = (-1 * (omega_body.cross(vel_body))) + ((1 / ap.m) * (fg_body + fa_body));

    Eigen::Matrix3d inertia_matrix{
        {ap.Ix, 0, ap.Ixz},
        {0, ap.Iy, 0},
        {-ap.Ixz, 0, ap.Iz}};

    Eigen::Vector3d omega_body_dot;

    omega_body_dot = inertia_matrix.inverse() * ((-1 * omega_body.cross(inertia_matrix * omega_body)) + ma_body);

    // State Derivative
    qdot[0] = vel_inertial[0];
    qdot[1] = vel_inertial[1];
    qdot[2] = vel_inertial[2];
    qdot[3] = euler_rates[0];
    qdot[4] = euler_rates[1];
    qdot[5] = euler_rates[2];
    qdot[6] = vel_body_dot[0];
    qdot[7] = vel_body_dot[1];
    qdot[8] = vel_body_dot[2];
    qdot[9] = omega_body_dot[0];
    qdot[10] = omega_body_dot[1];
    qdot[11] = omega_body_dot[2];
}

//--------------------------------------------------------------------
// Planner
//--------------------------------------------------------------------
bool plan()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE3StateSpace>());

    // Set bounds for state space
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-100); // TODO: make bounds realistic
    bounds.setHigh(100);

    space->setBounds(bounds);

    // Make object that holds all the state space info
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // Add the validity checker
    si->setStateValidityChecker(isStateValid);

    // Add the start and goal location
    ob::ScopedState<> start(space);
    start.random();
    start[0] = -50;
    start[1] = -50;
    start[2] = -50;

    ob::ScopedState<> goal(space);
    goal.random();
    goal[0] = 0;
    goal[1] = 0;
    goal[2] = 0;

    // Loading variables into the problem definition
    // Create prob def variable
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    // Load vars into prob def
    pdef->setStartAndGoalStates(start, goal);

    // Create planner (RRT)
    auto planner(std::make_shared<og::RRTConnect>(si));

    // Add the problem to the planner
    planner->setProblemDefinition(pdef);

    // Finalize planner setup
    planner->setup();

    // Solve Problem
    float maxSolveTime = 1.0; // Maximum time to spend on solving
    ob::PlannerStatus solved = planner->ob::Planner::solve(maxSolveTime);

    // If solution is solved:
    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        // path->print(std::cout);

        // Open file
        std::ofstream myfile;
        myfile.open("OutputPath.data");

        // Print path to file
        og::PathGeometric path1(dynamic_cast<const og::PathGeometric &>(*pdef->getSolutionPath()));
        // path1.printAsMatrix(std::cout); //output to terminal
        path1.printAsMatrix(myfile);

        // Close file

        return true; // Return sucsess
    }
    else
    {
        std::cout << "No Path Found" << std::endl;
        return false; // return a failure sence it didnt solve
    }
}

//--------------------------------------------------------------------
// Main Run Function
//--------------------------------------------------------------------
int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "Starting run\n";

    plan();

    return 0;
}
