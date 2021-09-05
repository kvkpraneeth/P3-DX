#ifndef SMC_H
#define SMC_H

#include "eigen3/Eigen/Core"

class smc
{

	public:
		// Constructor
		smc(double k1_, double k2_);

		//Give Output.
		std::vector<double> getControlSignal();

		//Current State
		Eigen::MatrixXd X = Eigen::MatrixXd(3,1);

		//Desired State
		Eigen::MatrixXd D = Eigen::MatrixXd(3,1);
	
		// Sliding surface Matrix
		Eigen::MatrixXd S = Eigen::MatrixXd(2,1);
	
	private:
		// Error State
		Eigen::MatrixXd E = Eigen::MatrixXd(3,1);

		//Output Vector
		Eigen::MatrixXd U = Eigen::MatrixXd(2,1);

		double k1,k2;

};

#endif
