#pragma once

#include "Instance.h"

class Tours;

// TODO: This it the Stacker Crane Approximation with slight depot
// modification. Hence, multiple crane => CHANGE THE NAME TO SOMETHING REASONABLE

/**
This class encapsulates the algorithm described by Hecht et al.
in their paper "Approximation algorithms for some routing problems".
**/
class SingleCraneTourApproximation{

	public:
		Tours operator()(const Instance& inst) const;
				
	private:
		Tours large_arc_(const Instance& inst) const;
		Tours large_edge_(const Instance& inst) const;


};
