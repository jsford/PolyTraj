#include <polytraj.h>

int main() {
  PolyTraj::PathState xs, xe;
  xs << 0, 0, M_PI_2, 0.0;
  xe << 10, 10, M_PI_2, 0.0;

  PolyTraj::Path path = PolyTraj::generatePath(xs, xe, 100);

}
