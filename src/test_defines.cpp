/*
 * test_defines.cpp
 *
 *  Created on: Feb 2, 2017
 *      Author: fraw
 */

#include "test_defines.h"

void HTMLOutput(std::ofstream &os, const std::vector<std::vector<double> > &results, const std::vector<std::string> &labels) {
	os.setf(std::ios::fixed, std::ios::floatfield);   // floatfield set to fixed

	os.precision(5);

	os << "<h2>Raw data</h2>" << std::endl;
	os << "<table border=1>" << std::endl;
	os << "<tr><td>Results in ms</td><td>CMAT</td><td>Eigen</td><td>None</td></tr>" << std::endl;

	for (size_t i = 0; i < results.size(); i++) {
		os << "<tr><td>" << labels[i] << "</td>";

		double best = *min_element(results[i].begin(), results[i].end());

		for (size_t j = 0; j < results[i].size(); j++) {
			if (results[i][j] == best) {
				os << "<td><font color=red>" << results[i][j] << "</font></td>";
			} else {
				os << "<td>" << results[i][j] << "</td>";
			}
		}

		os << "</tr>" << std::endl;
	}

	os << "</table>" << std::endl;
	os << "<h2>Normalised</h2>" << std::endl;
	os << "<table border=1>" << std::endl;
	os << "<tr><td>Speed up over slowest</td><td>CMAT</td><td>Eigen</td><td>None</td></tr>" << std::endl;

	os.precision(2);

	for (size_t i = 0; i < results.size(); i++) {
		os << "<tr><td>" << labels[i] << "</td>";

		double m = *max_element(results[i].begin(), results[i].end());
		double best = *min_element(results[i].begin(), results[i].end());

		for (size_t j = 0; j < results[i].size(); j++) {
			if (results[i][j] == best) {
				os << "<td><font color=red>" << (m / results[i][j]) << "x</font></td>";
			} else {
				os << "<td>" << (m / results[i][j]) << "x</td>";
			}
		}

		os << "</tr>" << std::endl;
	}

	os << "</table>" << std::endl;
}

double TimeDiff(timeval t1, timeval t2) {
	double t;
	t = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
	t += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms

	return t;
}

void PrintResult(const std::string type, const PinvSpecs specs, const double time){
	std::cout << type << "\t" << specs.nRows << "x" << specs.nCols << " \t " << time << std::endl;
}
