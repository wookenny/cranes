#pragma once

#include <vector>
#include <algorithm>

std::vector<bool> find_min_cut(const std::vector<std::tuple<int,int,double>> &edges,
								uint n);


