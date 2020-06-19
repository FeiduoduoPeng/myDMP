/*
 * @Author: Boris.Peng
 * @Date: 2020-06-18 14:58:26
 * @LastEditors: Boris.Peng
 * @LastEditTime: 2020-06-19 17:57:44
 */ 
#include "dmp/dmp.h"
using namespace dmp;

std::vector<DMPData> active_dmp_list;

int main(int argc, char **argv){
	dmpClass dmpc("./19.txt");

	// std::vector<double> vs{-8.39597e-08, 0.707108, -1.06886e-07, 0.707106, 0.11327, -0.27, 0.15001} ;
	std::vector<double> vs{1.6203700e-01, 1.8272600e-01, 8.6260000e-03, 9.6968100e-01, -1.6168900e-01, -2.2586900e-01, -6.3030000e-03};

	std::vector<double> vg{-4.5054000e-02, 6.9518000e-01, 3.1033000e-02, 7.1675100e-01, 1.0824800e-01, -3.3939300e-01, 1.5e-01};
	std::vector<std::vector<double>> plan;
	dmpc.dmp_service(0, vs, vg, plan);
	// dmpc.chooseData(0, vg, vg);

	std::cout<<plan.size()<<std::endl;
	std::ofstream of("../draw_dmp/res.txt");
	for(const auto &vec: plan){
		for(const auto &item:vec){
			// std::cout<<item<<"\t";
			of<<item<<"\t";
		}
		// std::cout<<std::endl;
		of<<std::endl;
	}
	of.close();
	return 0;
}
