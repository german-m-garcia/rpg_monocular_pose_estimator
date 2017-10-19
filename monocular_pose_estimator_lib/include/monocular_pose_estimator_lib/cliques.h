/*
 * cliques.h
 *
 *  Created on: Oct 19, 2017
 *      Author: tomatito
 */

#ifndef MONOCULAR_POSE_ESTIMATOR_LIB_INCLUDE_CLIQUES_H_
#define MONOCULAR_POSE_ESTIMATOR_LIB_INCLUDE_CLIQUES_H_

#include <vector>

void find_cliques(std::vector<std::vector<int> >& graph, int K , std::vector<std::vector<int> >& cliques);

void debug_graph(std::vector<std::vector<int> >& graph);


#endif /* MONOCULAR_POSE_ESTIMATOR_LIB_INCLUDE_CLIQUES_H_ */
