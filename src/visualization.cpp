#include "visualization.h"

Visualization::Visualization(double L1, double L2, double L3, double x, double y, double r) {
    this->L1 = L1;
    this->L2 = L2;
    this->L3 = L3;
    this->x = x;
    this->y = y;
    this->r = r;
    this->num_points = 0;
    this->q = std::vector<double>(3, 0.0); // Initialize q with 3 joint angles
}

Visualization::~Visualization() {
    // Destructor
    
}