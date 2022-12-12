/*
By: Benjamin chupik
For algorithmic motion planning
Main function for final project

From tempest file:
State Vector: [x, y, z, roll, pitch, yaw , x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot, time]
Input Vector: [d_e, d_a, d_r, t] (elevator, aileron, ruder, thrust)

Axis Frame: NED (so z needs to be negative)

*/

//--------------------------------------------------------------------
// Program Setup
//--------------------------------------------------------------------
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/TimeStateSpace.h>
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

// Constants
const double zmax = -0.2;
double mu_ground = 0.6; // fricction coefficient on ground

const double xdotgoal = 1;
const double zdotgoal = 2;

// TODO: hard coded the wind thing
Eigen::Vector3d wind_inertial{0, 0, 0};

// Constants
const double maxAOA = 0.261799; // 15 deg in radians

// Definition of the ODE
void flightDynamics(const oc::ODESolver::StateType &q, const oc::Control *control, oc::ODESolver::StateType &qdot)
{

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
    /*
    Void out y and z velocities so y and z body dosent change
    x velocity is normal, but x acceleration is really negative untill its velocity is zero and then stops
    */

    // Turn relevant states into vectors
    Eigen::Vector3d pos_inertial{q[0], q[1], q[2]};
    Eigen::Vector3d euler_angles{q[3], q[4], q[5]};
    // Eigen::Vector3d vel_body{q[6], q[7], q[8]};
    Eigen::Vector3d vel_body{q[6], 0, 0}; // Void
    Eigen::Vector3d omega_body{q[9], q[10], q[11]};

    // Kinematics
    Eigen::Vector3d vel_inertial = TransformFromBodyToInertial(vel_body, euler_angles);
    Eigen::Vector3d euler_rates = EulerRatesFromOmegaBody(omega_body, euler_angles);

    // Allowing yaw correction on runway
    struct params ap;

    Eigen::Matrix3d inertia_matrix{
        {ap.Ix, 0, ap.Ixz},
        {0, ap.Iy, 0},
        {-ap.Ixz, 0, ap.Iz}};

    Eigen::Vector3d fa_body;
    Eigen::Vector3d ma_body;
    Eigen::Vector3d wind_angles;

    double density = Formulae::barometricDensity(-pos_inertial[2]);
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    // Pitch, roll and their derivatives set to 0
    Eigen::Matrix<double, 12, 1> q_pert{q[0], q[1], q[2], q[3], 0, 0, q[6], q[7], q[8], q[9], 0, 0};
    Eigen::Vector4d u_as_vec{0, 0, u[2], 0};

    std::tie(fa_body, ma_body, wind_angles) = AeroForcesAndMoments_BodyState_WindCoeffs(q_pert, u_as_vec, wind_inertial, density);

    Eigen::Vector3d omega_body_dot;
    omega_body_dot = inertia_matrix.inverse() * ((-1 * omega_body.cross(inertia_matrix * omega_body)) + ma_body);
    Eigen::Vector3d vel_body_dot;
    Eigen::Vector3d fg_body;

    Eigen::Vector3d grav_vec{(-sin(euler_angles[1])), (sin(euler_angles[0]) * cos(euler_angles[1])), (cos(euler_angles[0]) * cos(euler_angles[1]))};
    fg_body = (ap.g * ap.m) * grav_vec;
    vel_body_dot = (-1 * (omega_body.cross(vel_body))) + ((1 / ap.m) * (fg_body + fa_body));

    // State Derivative
    qdot[0] = vel_inertial[0];
    qdot[1] = vel_inertial[1];
    qdot[2] = 0;

    qdot[3] = euler_rates[0];
    qdot[4] = 0;
    qdot[5] = 0;

    qdot[6] = -ap.g * mu_ground + vel_body_dot[0];
    qdot[7] = 0.0;
    qdot[8] = 0.0;

    qdot[9] = 0.0;
    qdot[10] = 0.0;
    qdot[11] = omega_body_dot[2];
}

void TempestODE(const oc::ODESolver::StateType &q, const oc::Control *control, oc::ODESolver::StateType &qdot)
{
    // Update time component (does not change in either dynamics)
    qdot[12] = 1; // Time update

    // If the z component is less than 0.5 meters its on the ground, y and z velocity is less than 0.5 m/s
    if (q[2] > zmax) // && q[7] < fabs(0.5) && q[8] < fabs(0.5))
    {
        groundDynamics(q, control, qdot);
    }
    else
    {
        flightDynamics(q, control, qdot);
    }
}

// This is a callback method invoked after numerical integration.
void PostIntegration(const ob::State * /*state*/, const oc::Control * /*control*/, const double /*duration*/, ob::State *result)
{

    //  Normalize orientation between 0 and 2*pi
    ompl::base::SO2StateSpace SO2; // make a class so we have acsess to the bounds function

    // Enforce the bounds on all the angle states
    SO2.enforceBounds(result->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1));
    SO2.enforceBounds(result->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(2));
    SO2.enforceBounds(result->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(3));
}

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    // Unpack vectors
    double *pos = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
    double x = pos[0];
    double y = pos[1];
    double z = pos[2];

    double yaw = state->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1)->value;
    double pitch = state->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(2)->value;
    double roll = state->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(3)->value;

    // Limiting angle of attack
    double *vel = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(4)->values;
    double alpha = atan2(vel[2], vel[0]);

    Eigen::Vector3d eu_angles{yaw, pitch, roll};
    Eigen::Vector3d vel_states{vel[0], vel[1], vel[2]};
    Eigen::Vector3d vel_inert = TransformFromBodyToInertial(vel_states, eu_angles);

    if (z > 0 || alpha > maxAOA)
    {
        return false;
    }
    else
    {
        if (z > zmax)
        {
            if (vel_inert[2] < zdotgoal)
            { //&& vel_inert[2] < zdotgoal
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return true;
        }
    }
}

class DemoControlSpace : public oc::RealVectorControlSpace
{
public:
    DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 4)
    {
    }
};

class CustomGoal : public ob::GoalRegion
{
public:
    CustomGoal(const ob::SpaceInformationPtr &si) : ob::GoalRegion(si)
    {
        threshold_ = 0.5;
    }

    double distanceGoal(const ob::State *st) const override
    {
        // Break the state apart into components (position)
        double *pos = st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
        double x = pos[0];
        double y = pos[1];
        double z = pos[2];

        // Break the state apart into components (roll, pitch, yaw)
        double roll = st->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1)->value;
        double yaw = st->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(3)->value;
        double pitch = st->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(2)->value;

        // Break the state apart into components (velocity)
        double *vel = st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(4)->values;
        Eigen::Vector3d eu_angles{yaw, pitch, roll};
        Eigen::Vector3d vel_states{vel[0], vel[1], vel[2]};
        Eigen::Vector3d vel_inert = TransformFromBodyToInertial(vel_states, eu_angles);

        double xdot_in = vel_inert[0];
        double ydot_in = vel_inert[1];
        double zdot_in = vel_inert[2];
        //
        double dz = fabs(z);

        // Goal vectors
        double velocity = sqrt(zdot_in * zdot_in + ydot_in * ydot_in + xdot_in * xdot_in);
        double zPosNorm = sqrt(dz * dz);
        double anglenorm = sqrt(pitch * pitch + roll * roll);

        // Goal weighting of vectors (more weight is more important)
        return zPosNorm + velocity + 0.4 * anglenorm;
    }
};

void planWithSimpleSetup()
{
    //  Make state space (R3, SO2x3, R6)
    auto r3(std::make_shared<ob::RealVectorStateSpace>(3)); // R^3 (position)
    auto so21(std::make_shared<ob::SO2StateSpace>());       // so2 (roll)
    auto so22(std::make_shared<ob::SO2StateSpace>());       // so2 (pitch)
    auto so23(std::make_shared<ob::SO2StateSpace>());       // so2 (yaw)
    auto r6(std::make_shared<ob::RealVectorStateSpace>(6)); // R^6 (position velocity, anguar velocity)
    auto t(std::make_shared<ob::TimeStateSpace>());         // R (time)

    // Make State Space Bounds (so2 bounds allready fixed)
    ob::RealVectorBounds posbounds(3); // Position
    ob::RealVectorBounds velbounds(6); // Velocities
    // Position bounds
    posbounds.setLow(0, -200); // x
    posbounds.setHigh(0, 200); // x
    posbounds.setLow(1, -50);  // y
    posbounds.setHigh(1, 50);  // y
    posbounds.setLow(2, -50);  // z
    posbounds.setHigh(2, 0);   // z
    r3->setBounds(posbounds);  // set the bounds
    // Velocity bounds
    velbounds.setLow(-50); // TODO: make these bounds more realisitc
    velbounds.setHigh(100);
    r6->setBounds(velbounds);

    // Combine smaller spaces into big main space
    ob::StateSpacePtr space = r3 + so21 + so22 + so23 + r6 + t;

    // create a control space
    auto cspace(std::make_shared<DemoControlSpace>(space));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(4); // 4 dim control space
    cbounds.setLow(0, -.4);          // elevator deflection [rads]
    cbounds.setHigh(0, .4);          // elevator deflection [rads]
    cbounds.setLow(1, -.4);          // aileron deflection [rads]
    cbounds.setHigh(1, .4);          // aileron deflection [rads]
    cbounds.setLow(2, -.4);          // rudder deflection [rads]
    cbounds.setHigh(2, .4);          // rudder deflection [rads]
    cbounds.setLow(3, -.7);          // Throttle Ranage (percents not newtons)
    cbounds.setHigh(3, .7);          // Throttle Ranage

    cspace->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set state validity checking for this space
    oc::SpaceInformation *si = ss.getSpaceInformation().get();

    // Change integration and control timings
    si->setMinControlDuration(1);    // Lowest step of control
    si->setMaxControlDuration(10);   // Highest step osf control
    si->setPropagationStepSize(0.1); // Step size of time

    ss.setStateValidityChecker([si](const ob::State *state)
                               { return isStateValid(si, state); });

    // Use the ODESolver to propagate the system.  Call PostIntegration
    // when integration has finished to normalize the orientation values.
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &TempestODE));
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &PostIntegration));

    // Make start vector
    ob::ScopedState<> start(space);
    start.random();
    // Position
    start[0] = -199;
    start[1] = 0;
    start[2] = -20;
    // Angles
    start[3] = 0;
    start[4] = 0;
    start[5] = 0;
    // Velocity
    start[6] = 15;
    start[7] = 0;
    start[8] = 0;
    // Angular velocity
    start[9] = 0;
    start[10] = 0;
    start[11] = 0;

    // Set start and goal positions
    ss.setStartState(start);
    ss.setGoal(std::make_shared<CustomGoal>(ss.getSpaceInformation()));

    // Change Planner to SST
    ompl::base::PlannerPtr planner(new oc::SST(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // Finalize setup
    ss.setup();

    // ss.print(); // Print the setup information

    ob::PlannerStatus solved = ss.solve(20 * 60.0);

    // Displaying information
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

        // Print solution to files
        ss.getSolutionPath().asGeometric().printAsMatrix(myfile_geo);
        ss.getSolutionPath().printAsMatrix(myfile_cont);

        // Close files
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