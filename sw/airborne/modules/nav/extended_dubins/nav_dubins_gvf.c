#include "nav_dubins_gvf.h"

struct transition_spline_t
{
  int element_index; // Index of the element before the spline, such that it represents a transition from index to index+1
  float offset; // Offset from full curve to polynomial, since they take input in range [0,dx]
  float cx[6]; // Coefficients of polynomial for X axis
  float cy[6]; // Coefficients of polynomial for Y axis
} transition_spline;


void nav_dubins_gvf_track(DubinsElement_t* elements, int el_num, float openloop_w, float nominal_speed)
{
  float wb;
  if (openloop_w >= 0.)
  {
    wb = openloop_w;
  }
  else
  {
    wb = gvf_parametric_control.w * gvf_parametric_control.beta * gvf_parametric_control.s;
  }

  DubinsElement_t el = elements[el_num];
  Pose2D_t p = dubins_element_follow(&el,wb);
  Pose2D_t v = dubins_element_velocity(&el,wb);
  Pose2D_t a = dubins_element_accel(&el,wb);

  // Rescale velocity and accel according to nominal speed
  v.x *= nominal_speed;
  v.y *= nominal_speed;

  a.x *= nominal_speed*nominal_speed;
  a.y *= nominal_speed*nominal_speed;
  
  gvf_parametric_control_2D(1,1,p.x,p.y,v.x,v.y,a.x,a.y);
}