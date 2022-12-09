//This cpp will contain all necessary functions for the nonlinear simulation of the tempest ap
//Roland Ilyes, November 25th 2021

#include <Eigen/Dense>
#include <iostream>
#include "tempest.h"

struct params ap;

Eigen::Matrix3d rotationMatrix321(double roll, double pitch, double yaw){
	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());

	//Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

	Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;

	return q.matrix().transpose();
}

Eigen::Vector3d TransformFromInertialToBody(Eigen::Vector3d vector_inertial, Eigen::Vector3d euler_angles){
	return rotationMatrix321(euler_angles[0],euler_angles[1],euler_angles[2])*vector_inertial;
}

Eigen::Vector3d TransformFromBodyToInertial(Eigen::Vector3d vector_inertial, Eigen::Vector3d euler_angles){
	return rotationMatrix321(euler_angles[0],euler_angles[1],euler_angles[2]).transpose()*vector_inertial;
}

Eigen::Vector3d EulerRatesFromOmegaBody(Eigen::Vector3d omega_body, Eigen::Vector3d euler_angles){
	Eigen::Matrix3d transform_matrix {
		{1, (sin(euler_angles[0])*tan(euler_angles[1])), (cos(euler_angles[0])*tan(euler_angles[1]))},
		{0, cos(euler_angles[0]),-sin(euler_angles[0])},
		{0, (sin(euler_angles[0])*(1/cos(euler_angles[1]))), (cos(euler_angles[0])*(1/cos(euler_angles[1])))}
	};

	return transform_matrix*omega_body;
}

Eigen::Vector3d WindAnglesFromVelocityBody(Eigen::Vector3d velocity_body){
	double V;
	double alpha;
	double beta;

	V = velocity_body.norm();

	if (V==0)
	{
		alpha = 0;
		beta = 0;
		V = 21;
	} else {
		alpha = atan2(velocity_body[2],velocity_body[0]);
		beta = asin(velocity_body[1]/V);
	}

	return {V,beta,alpha};
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> AeroForcesAndMoments_BodyState_WindCoeffs(Eigen::Matrix<double, 12, 1> aircraft_state, Eigen::Vector4d aircraft_surfaces, Eigen::Vector3d wind_inertial, double density){
	//Declare variables
	Eigen::Vector3d eu_angles {aircraft_state[3],aircraft_state[4],aircraft_state[5]};
	Eigen::Vector3d vel_states {aircraft_state[6],aircraft_state[7],aircraft_state[8]};
	double p = aircraft_state[9];
	double q = aircraft_state[10];
	double r = aircraft_state[11];
	double de = aircraft_surfaces[0];
	double da = aircraft_surfaces[1];
	double dr = aircraft_surfaces[2];
	double dt = aircraft_surfaces[3];


	//Redefine states and inputs for ease of use
	Eigen::Vector3d wind_body = TransformFromInertialToBody(wind_inertial, eu_angles);
	Eigen::Vector3d air_rel_vel_body = vel_states - wind_body; //this one's final state is a teency bit off

	Eigen::Vector3d wind_angles = WindAnglesFromVelocityBody(air_rel_vel_body);
	double V = wind_angles[0];
	double beta = wind_angles[1];
	double alpha = wind_angles[2];

	//double alpha_dot = 0;

	double Q = 0.5*density*V*V;

	double sa = sin(alpha);
	double ca = cos(alpha);

	// determine aero force coefficients
	double CL = ap.CL0 + (ap.CLalpha*alpha) + (ap.CLq*q*ap.c/(2*V)) + (ap.CLde*de);
	double CD = ap.CDpa + (ap.K*CL*CL);

	double CX = (-CD*ca) + (CL*sa);
	double CZ = (-CD*sa) - (CL*ca);

	double CY = ap.CY0 + (ap.CYbeta*beta) + ((ap.CYp*p*ap.b)/(2*V)) + ((ap.CYr*r*ap.b)/(2*V)) + (ap.CYda*da) + (ap.CYdr*dr);

	double Thrust = density*ap.Sprop*ap.Cprop*(V + dt*(ap.kmotor - V))*dt*(ap.kmotor-V); // Small UAS model as described in http://uavbook.byu.edu/lib/exe/fetch.php?media=shared:propeller_model.pdf

	// determine aero forces from coefficients
	double X = (Q*ap.S*CX) + Thrust;
	double Y = Q*ap.S*CY;
	double Z = Q*ap.S*CZ;

	Eigen::Vector3d aero_forces {X,Y,Z};

	// determine aero moment coefficients
	double Cl = ap.Cl0 + (ap.Clbeta*beta) + (ap.Clp*p*ap.b/(2*V)) + (ap.Clr*r*ap.b/(2*V)) + (ap.Clda*da) + (ap.Cldr*dr);
	double Cm = ap.Cm0 + (ap.Cmalpha*alpha) + (ap.Cmq*q*ap.c/(2*V)) + (ap.Cmde*de);
	double Cn = ap.Cn0 + (ap.Cnbeta*beta) + (ap.Cnp*p*ap.b/(2*V)) + (ap.Cnr*r*ap.b/(2*V)) + (ap.Cnda*da) + (ap.Cndr*dr);

	// determine aero moments from coefficients
	Eigen::Vector3d almost_aero_moments {(ap.b*Cl),(ap.c*Cm),(ap.b*Cn)};
	Eigen::Vector3d aero_moments = (Q*ap.S)*almost_aero_moments;

	return std::make_tuple(aero_forces,aero_moments,wind_angles);

}