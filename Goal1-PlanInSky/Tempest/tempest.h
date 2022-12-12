#pragma once

#include <Eigen/Dense>

Eigen::Matrix3d rotationMatrix321(double roll, double pitch, double yaw);

Eigen::Vector3d TransformFromInertialToBody(Eigen::Vector3d vector_inertial, Eigen::Vector3d euler_angles);

Eigen::Vector3d TransformFromBodyToInertial(Eigen::Vector3d vector_inertial, Eigen::Vector3d euler_angles);

Eigen::Vector3d EulerRatesFromOmegaBody(Eigen::Vector3d omega_body, Eigen::Vector3d euler_angles);

Eigen::Vector3d WindAnglesFromVelocityBody(Eigen::Vector3d velocity_body);

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> AeroForcesAndMoments_BodyState_WindCoeffs(Eigen::Matrix<double, 12, 1> aircraft_state, Eigen::Vector4d aircraft_surfaces, Eigen::Vector3d wind_inertial, double density);

struct params {
	//gravitational acceleration
	double g = 9.81;

	//pi
	double pi = 3.14159265358979323846;

	//geometry parameters
	double S = 0.6282; //[m^2] // EWF: Modified 2/3/16
	double b = 3.067; //[m] // EWF: Modified 2/3/16
	double c = 0.208; //[m]
	double AR = std::pow(b,2)/S;

	// Aircraft mass parameters
	double m = 5.74; //[kg]
	double W = m*g; //[N]

	// Inertias from Solidworks
	double SLUGFT2_TO_KGM2 = 14.5939/(3.2804*3.2804);
	double Ix = SLUGFT2_TO_KGM2*4106/std::pow(12,2)/32.2; //[kg m^2]
	double Iy = SLUGFT2_TO_KGM2*3186/std::pow(12,2)/32.2; //[kg m^2]
	double Iz = SLUGFT2_TO_KGM2*7089/std::pow(12,2)/32.2; //[kg m^2]
	double Ixz = SLUGFT2_TO_KGM2*323.5/std::pow(12,2)/32.2; //[kg m^2]


	// Drag terms
	double e = 0.95; //0.9693; //[-], AVL, Oswald's efficiency factor
	//K = 1/(pi*(b^2/S)*e); //drag polar coefficient (CD = CDpa + kCL^2)
	double K = 1/(pi*AR*e); //drag polar coefficient (CD = CDpa + kCL^2)
	double CDpa = 0.021; //[-] This is the parasite drag. Total drag is combination of parasite drag and induced drag.
	double CDmin = 0.021;
	double CLmin = 0.0;

	// Engine parameters
	double Sprop = 0.0707;//0.2027; //[m^2]
	double Cprop = 1;
	double kmotor = 40;

	// Zero angle of attack aerodynamic forces and moments
	//CL0 = 0.2219;
	//Cm0 = 0.0519;
	double CL0 = 0;
	double Cm0 = 0.1104;

	double CY0 = 0;
	double Cl0 = 0;
	double Cn0 = 0;

	// Longtidunal nondimensional stability derivatives from AVL
	double CLalpha = 6.196683; 
	double Cmalpha = -1.634010;
	double CLq = 10.137584; 
	double Cmq = -24.376066;


	// Neglected parameters, check units below if incorporated later
	double CLalphadot = 0; 
	double Cmalphadot = 0; 


	// Lateral-directional nondimensional stability derivatives from AVL
	double CYbeta = -0.367231; 
	double Clbeta = -0.080738; 
	double Cnbeta = 0.080613; 
	double CYp = -0.064992; 
	double Clp = -0.686618; 
	double Cnp = -0.039384; 
	double Clr = 0.119718; 
	double Cnr = -0.052324; 
	double CYr = 0.213412;  

	//Control surface deflection parameters

	// Elevator
	double CLde =   0.006776;
	double Cmde =  -0.028684; 

	// Aileron
	double CYda =  -0.000754;
	double Clda =  -0.006290;
	double Cnda =  -0.000078;

	// Rudder
	double CYdr =   0.003056;
	double Cldr =   0.000157;
	double Cndr =  -0.000856;

};