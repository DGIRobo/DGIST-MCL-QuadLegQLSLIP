#ifndef __FILTER_H__
#define __FILTER_H__


#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

double tustin_derivative(double input, double input_old, double output_old, double cutoff_freq);
double lowpassfilter(double input, double input_old, double output_old, double cutoff_freq);


#endif // !__FILTER_H__
