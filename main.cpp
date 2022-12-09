/*
By: Benjamin chupik
For algorithmic motion planning
Main function for final project

From tempest file:
State Vector: [x, y, z, yaw, pitch, roll, x_dot, y_dot, z_dot, yaw_dot, pitch_dot, roll_dot]
Input Vector: [d_e, d_a, d_r, t] (elevator, aileron, ruder, thrust)

Axis Frame: NED (so z needs to be negative)

*/

//--------------------------------------------------------------------
// Program Setup
//--------------------------------------------------------------------
#include <ompl/control/SpaceInformation.h>
// #include <ompl/extensions/ode/OpenDEStateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/SpaceInformation.h>

#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <valarray>
#include <limits>
#include <ompl/control/planners/sst/SST.h>

#include <tempest.h>
#include <barometric_formula.h>

// Name Spaces
namespace ob = ompl::base;
namespace oc = ompl::control;

// Definition of the ODE
// Gound
void flightDynamics(const oc::ODESolver::StateType &q, const oc::Control *control, oc::ODESolver::StateType &qdot)
{
    // TODO: hard coded the wind thing
    Eigen::Vector3d wind_inertial{0, 0, 0};

    // TempestODE - This function adapted for c++ from function provided by Professor Eric Frew. Adapted by Roland Ilyes

    // load in tempest aircraft parameters
    struct params ap;

    // cast control to the type we expect
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    // notes: reference u[0] to u[3], elevator, aileron, rudder, throttle

    /*
    std::cout << "☺ \n"
              << "x: " << q[0] << "  |  y: " << q[1] << "  |  z: " << q[2] << "\n"
              << "roll: " << q[3] << "  |  pitch: " << q[4] << "  |  yaw: " << q[5] << "\n"
              << "d_e: " << u[0] << "  |  d_a:" << u[1] << "  |  d_r:" << u[2] << "  |  d_t:" << u[3] << "\n";
    */
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

void groundDynamics(const oc::ODESolver::StateType &q, const oc::Control *control, oc::ODESolver::StateType &qdot)
{

    // Turn relevant states into vectors
    Eigen::Vector3d pos_inertial{q[0], q[1], q[2]};
    Eigen::Vector3d euler_angles{q[3], q[4], q[5]};
    Eigen::Vector3d vel_body{q[6], q[7], q[8]};
    Eigen::Vector3d omega_body{q[9], q[10], q[11]};

    // Kinematics
    Eigen::Vector3d vel_inertial = TransformFromBodyToInertial(vel_body, euler_angles);
    Eigen::Vector3d euler_rates = EulerRatesFromOmegaBody(omega_body, euler_angles);

    // State Derivative
    qdot[0] = vel_inertial[0];
    qdot[1] = vel_inertial[1];
    qdot[2] = vel_inertial[2];
    qdot[3] = euler_rates[0];
    qdot[4] = euler_rates[1];
    qdot[5] = euler_rates[2];
    qdot[6] = -100.0;
    qdot[7] = 0.0;
    qdot[8] = 0.0;
    qdot[9] = 0.0;
    qdot[10] = 0.0;
    qdot[11] = 0.0;
}

void TempestODE(const oc::ODESolver::StateType &q, const oc::Control *control, oc::ODESolver::StateType &qdot)
{
    // If the z component is less than 0.5 meters its on the ground
    if (q[2] > -1)
    {
        groundDynamics(q, control, qdot);
    }
    else
    {
        flightDynamics(q, control, qdot);
    }
}

// This is a callback method invoked after numerical integration.
void KinematicCarPostIntegration(const ob::State * /*state*/, const oc::Control * /*control*/, const double /*duration*/, ob::State *result)
{

    //  Normalize orientation between 0 and 2*pi
    ompl::base::SO2StateSpace SO2;

    SO2.enforceBounds(result->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1));
    SO2.enforceBounds(result->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(2));
    SO2.enforceBounds(result->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(3));
    // SO2.enforceBounds(result->as<ob::CompoundState>()[5].as<ob::SO2StateSpace::StateType>(0));
}

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    double *pos = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
    double z = pos[2];
    //    ob::ScopedState<ob::SE2StateSpace>
    /*
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && (const void *)rot != (const void *)pos;
    */
    if (z > 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

class DemoControlSpace : public oc::RealVectorControlSpace
{
public:
    DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 4)
    {
    }
};

void planWithSimpleSetup()
{
    // auto space(std::make_shared<ob::SE3StateSpace>());
    //  Make state space (R3, SO2x3, R6)
    // ob::StateSpacePtr r3(new ob::RealVectorStateSpace(3));
    // auto r3(std::make_shared<ob::SE3StateSpace>());
    auto r3(std::make_shared<ob::RealVectorStateSpace>(3));
    // ob::StateSpacePtr so2(new ob::SO2StateSpace());
    auto so21(std::make_shared<ob::SO2StateSpace>());
    auto so22(std::make_shared<ob::SO2StateSpace>());
    auto so23(std::make_shared<ob::SO2StateSpace>());
    // ob::StateSpacePtr r6(new ob::RealVectorStateSpace(6));
    auto r6(std::make_shared<ob::RealVectorStateSpace>(6));

    // Make Bounds
    ob::RealVectorBounds posbounds(3);
    posbounds.setLow(0, -200);
    posbounds.setHigh(0, 200);
    posbounds.setLow(1, -200);
    posbounds.setHigh(1, 200);
    posbounds.setLow(2, -200);
    posbounds.setHigh(2, 0);
    r3->setBounds(posbounds);

    ob::RealVectorBounds velbounds(6);
    velbounds.setLow(-50);
    velbounds.setHigh(100);
    r6->setBounds(velbounds);

    // Combine smaller spaces into big main space
    ob::StateSpacePtr space = r3 + so21 + so22 + so23 + r6;

    // create a control space
    auto cspace(std::make_shared<DemoControlSpace>(space));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(4); // 4 dim control space
    cbounds.setLow(0, -.4);
    cbounds.setHigh(0, .4);

    cbounds.setLow(1, -.4);
    cbounds.setHigh(1, .4);

    cbounds.setLow(2, -.4);
    cbounds.setHigh(2, .4);

    cbounds.setLow(3, -.7);
    cbounds.setHigh(3, .7);

    cspace->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set state validity checking for this space
    oc::SpaceInformation *si = ss.getSpaceInformation().get();

    si->setMinControlDuration(1);
    si->setMaxControlDuration(10);
    si->setPropagationStepSize(0.1);

    ss.setStateValidityChecker([si](const ob::State *state)
                               { return isStateValid(si, state); });

    // Use the ODESolver to propagate the system.  Call KinematicCarPostIntegration
    // when integration has finished to normalize the orientation values.
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &TempestODE));
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &KinematicCarPostIntegration));

    ob::ScopedState<> start(space);
    start.random();
    start[0] = 0;
    start[1] = 0;
    start[2] = -20;

    start[3] = 0;
    start[4] = 0;
    start[5] = 0;

    start[6] = 15;
    start[7] = 0;
    start[8] = 0;

    start[9] = 0;
    start[10] = 0;
    start[11] = 0;

    class CustomGoal : public ob::GoalRegion
    {
    public:
        CustomGoal(const ob::SpaceInformationPtr &si) : ob::GoalRegion(si)
        {
            threshold_ = 0.5;
        }

        double distanceGoal(const ob::State *st) const override
        {

            double *pos = st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
            double dz = fabs(pos[2] + 0.2);

            double *vel = st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(4)->values;
            double zdot = fabs(vel[2] + 0.1);

            // std::cout << pos[0] <<"\n";
            return fabs(dz * dz + zdot * zdot);
        }
    };
    ss.setStartState(start);
    ss.setGoal(std::make_shared<CustomGoal>(ss.getSpaceInformation()));
    /*
    ob::ScopedState<> goal(space);
    goal.random();
    goal[0] = 0;
    goal[1] = 0;
    goal[2] = -0.2;

    goal[3] = 0;
    goal[4] = 0;
    goal[5] = 0;

    goal[6] = 0;
    goal[7] = 0;
    goal[8] = 0;

    goal[9] = 0;
    goal[10] = 0;
    goal[11] = 0;

    ss.setStartAndGoalStates(start, goal, 15);
    */
    // Change Planner
    ompl::base::PlannerPtr planner(new oc::SST(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // Finalize setup
    ss.setup();

    // ss.print();

    ob::PlannerStatus solved = ss.solve(.5 * 60.0);

    // std::cout << "NOT HERE **********************\n";

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;

        if (ss.haveExactSolutionPath())
        {
            std::cout << "Solution is exact" << std::endl;
        }
        else
        {
            std::cout << "Solution is not exact" << std::endl;
        }

        // Open file
        std::ofstream myfile_geo, myfile_cont;
        myfile_geo.open("OutputPath_geo.data");   // geometric data
        myfile_cont.open("OutputPath_cont.data"); // control data

        ss.getSolutionPath().asGeometric().printAsMatrix(myfile_geo);
        ss.getSolutionPath().printAsMatrix(myfile_cont);

        myfile_geo.close();
        myfile_cont.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    planWithSimpleSetup();

    return 0;
}