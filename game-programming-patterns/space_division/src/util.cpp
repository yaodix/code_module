#include "util.h"
#include <cmath>


double DistSquared(double x1, double y1, double x2, double y2) {
    double dx = x1 - x2;
    double dy = y1 - y2;
    return dx * dx + dy * dy;
}


double Dist(double x1, double y1, double x2, double y2) {
    double dx = x1 - x2;
    double dy = y1 - y2;
    return sqrt(dx * dx + dy * dy);
}


double Length(double x, double y) {
    return Dist(x, y, 0, 0);
}

double DotProduct(double x1, double y1, double x2, double y2) {
    return x1 * x2 + y1 * y2;
}
