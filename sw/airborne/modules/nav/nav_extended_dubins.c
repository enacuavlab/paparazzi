#include "nav_extended_dubins.h"

#include <assert.h>
#include "math.h"

#include "firmwares/fixedwing/nav.h"
#include "firmwares/fixedwing/guidance/guidance_v.h"
#include "modules/gps/gps.h"



#define DEBUG 1

#ifdef DEBUG
#include <stdio.h>

bool draw_dubins = true;

#define IPRINTF(...) printf("%d : ",AC_ID) ; printf(__VA_ARGS__)

#else

bool draw_dubins = false;
#endif

bool HasStartExtension(DubinsType t)
{
  return (8 & t);
}
bool HasEndExtension(DubinsType t)
{
  return (2*8 & t);
}

bool NotExtendedDubins(DubinsType t)
{
  return t < 8;
}
bool StartExtendedDubins(DubinsType t)
{
  return HasStartExtension(t) && !HasEndExtension(t);
}
bool EndExtendedDubins(DubinsType t)
{
  return !HasStartExtension(t) && HasEndExtension(t);
}
bool BothExtendedDubins(DubinsType t)
{
  return HasStartExtension(t) && HasEndExtension(t);
}

DubinsType BaseDubinsType(DubinsType t)
{
  return t & 7;
}
bool ValidExtendedDubins(DubinsType t)
{
  return t < FIRST_INVALID_DUBINS_TYPE;
}


// -------------------- General maths -------------------- //

/**
 * @brief Reduce an angle in radian to [0,2*Pi]
 * 
 * @param x Input angle
 * @return double Reduced equivalent value in [0,2*Pi]
 */
[[gnu::const]]
static double mod_2pi(double x) 
{
    double output = fmod(x,2*M_PI);
    return (output > 0) ? output : output + 2*M_PI;
}

/**
 * @brief Reduce an angle to its central form, i.e. into the interval [-Pi,Pi]
 * 
 * @param x Angle
 * @return double Equivalent to x in [-pi,pi]
 */
[[gnu::const]]
static double central_angle(double x)
{
    double output = fmod(x,2*M_PI);

    if (output > M_PI)
    {
        return output - 2*M_PI;
    }

    if (output < -M_PI)
    {
        return output + 2*M_PI;
    }

    return output;
}   

//********************  Fundamental Dubins path computing  ********************//

// ********** Helpers ********** //

// A normalized Dubins problem, starting from (0,0,alpha) and going to (d,0,beta) with turn radius 1
typedef struct
{
  double alpha,beta,d; // Start angle, end angle and x coordinate of the ending point.
} NormalizedDubins_t;

// ----- Declarations ----- //

NormalizedDubins_t normalize_pts(float sx, float sy, float stheta, float ex, float ey, float etheta);
NormalizedDubins_t normalize_poses(Pose2D_t start, Pose2D_t end);
void shift_forward(Pose2D_t *p, float length);
Pose2D_t move_forward(Pose2D_t *p, float length);
void shift_circle(Pose2D_t *p, float length, float radius);
Pose2D_t move_circle(Pose2D_t *p, float length, float radius);
Pose2D_t dubins_element_end(DubinsElement_t *el);

// ----- Definitions ----- //

NormalizedDubins_t normalize_pts(float sx, float sy, float stheta, float ex, float ey, float etheta)
{
  double dx = ex - sx;
  double dy = ey - sy;
  double r_angle = atan2(dy,dx);

  NormalizedDubins_t output;
  output.d = sqrt(dx*dx + dy*dy);
  output.alpha = stheta - r_angle;
  output.beta = etheta - r_angle;

  return output;
}

NormalizedDubins_t normalize_poses(Pose2D_t start, Pose2D_t end)
{
  return normalize_pts(start.x,start.y,start.theta,end.x,end.y,end.theta);
}

void shift_forward(Pose2D_t *p, float length)
{
  double c = cos(p->theta);
  double s = sin(p->theta);

  p->x += length*c;
  p->y += length*s;
}

Pose2D_t move_forward(Pose2D_t *p, float length)
{
  Pose2D_t output = *p;
  shift_forward(&output,length);
  return output;
}

void shift_circle(Pose2D_t *p, float length, float radius)
{
  double angle = p->theta + length/radius;

  p->x += radius*(sin(angle) - sin(p->theta));
  p->y -= radius*(cos(angle) - cos(p->theta));
  p->theta = angle;
}

Pose2D_t move_circle(Pose2D_t *p, float length, float radius)
{
  Pose2D_t output = *p;
  shift_circle(&output, length, radius);
  return output;
}

Pose2D_t dubins_element_end(DubinsElement_t *el)
{
  if (el->radius == 0.)
  {
    return move_forward(&el->init_point,el->length);
  }
  else
  {
    return move_circle(&el->init_point,el->length,el->radius);
  }
}

/**
 * There are 6 fundamental Dubins paths, and 2 extras, described using three elementary moves among
 * right turns R, left turns L and straights S
 * 
 * Fundamental:
 * - LSL, RSR (always exist)
 * - RSL, LSR (long distance paths)
 * - RLR, LRL (short distance paths)
 * 
 * Extras:
 * - SRS, SLS ("aligned" poses)
 */


/********** LSL Lengths **********/

[[gnu::const]]
static double LSL_first_distance(double alpha, double beta, double d)
{
    double delta_cos = cos(beta)-cos(alpha);
    double delta_sin = sin(beta)-sin(alpha);
    double t_lsl = mod_2pi(-alpha + atan2(delta_cos,(d-delta_sin)));

    return t_lsl;
}

[[gnu::const]]
static double LSL_middle_distance(double alpha, double beta, double d)
{
    double delta_sin = sin(beta)-sin(alpha);
    double p_lsl = sqrt(2+d*d-2*cos(alpha-beta)-2*d*delta_sin);

    return p_lsl;
}

[[gnu::const]]
static double LSL_last_distance(double alpha, double beta, double d)
{
    double delta_cos = cos(beta)-cos(alpha);
    double delta_sin = sin(beta)-sin(alpha);
    double q_lsl = mod_2pi(beta - atan2(delta_cos,(d-delta_sin)));

    return q_lsl;
}

// [[gnu::const]]
// static double LSL_total_distance(double alpha, double beta, double d)
// {
//     return LSL_first_distance(alpha,beta,d) 
//         + LSL_middle_distance(alpha,beta,d) 
//         + LSL_last_distance(alpha,beta,d); 
// }

/********** RSR Lengths **********/

[[gnu::const]]
static double RSR_first_distance(double alpha, double beta, double d)
{
    double delta_cos = cos(beta)-cos(alpha);
    double delta_sin = sin(beta)-sin(alpha);
    return mod_2pi(alpha - atan2(-delta_cos,(d+delta_sin)));
}

[[gnu::const]]
static double RSR_middle_distance(double alpha, double beta, double d)
{
    double delta_sin = sin(beta)-sin(alpha);
    return sqrt(2+d*d-2*cos(alpha-beta)+2*d*delta_sin);
}

[[gnu::const]]
static double RSR_last_distance(double alpha, double beta, double d)
{
    double delta_cos = cos(beta)-cos(alpha);
    double delta_sin = sin(beta)-sin(alpha);
    return mod_2pi(- beta + atan2(-delta_cos,(d+delta_sin)));
}

// [[gnu::const]]
// static double RSR_total_distance(double alpha, double beta, double d)
// {
//     return RSR_first_distance(alpha,beta,d) 
//         + RSR_middle_distance(alpha,beta,d) 
//         + RSR_last_distance(alpha,beta,d); 
// }

/********** RSL Lengths **********/

[[gnu::const]]
static double RSL_middle_distance(double alpha, double beta, double d)
{
    double sum_sin = sin(alpha)+sin(beta);
    return sqrt(d*d-2+2*cos(alpha-beta)-2*d*sum_sin);
}

[[gnu::const]]
static double RSL_first_distance(double alpha, double beta, double d)
{
    double sum_cos = cos(alpha)+cos(beta);
    double sum_sin = sin(alpha)+sin(beta);
    return mod_2pi(alpha-atan2(sum_cos,(d-sum_sin)) + atan2(2,RSL_middle_distance(alpha,beta,d)));
}


[[gnu::const]]
static double RSL_last_distance(double alpha, double beta, double d)
{
    double sum_cos = cos(alpha)+cos(beta);
    double sum_sin = sin(alpha)+sin(beta);
    return mod_2pi(beta - atan2(sum_cos,(d-sum_sin)) + atan2(2,RSL_middle_distance(alpha,beta,d)));
}

// [[gnu::const]]
// static double RSL_total_distance(double alpha, double beta, double d)
// {
//     return RSL_first_distance(alpha,beta,d) 
//         + RSL_middle_distance(alpha,beta,d) 
//         + RSL_last_distance(alpha,beta,d); 
// }

/********** LSR Lengths **********/

[[gnu::const]]
static double LSR_middle_distance(double alpha, double beta, double d)
{
    double sum_sin = sin(alpha)+sin(beta);
    return sqrt(-2+d*d+2*cos(alpha-beta)+2*d*sum_sin);
}

[[gnu::const]]
static double LSR_first_distance(double alpha, double beta, double d)
{
    double sum_cos = cos(alpha)+cos(beta);
    double sum_sin = sin(alpha)+sin(beta);
    return mod_2pi(-alpha+atan2(-sum_cos,(d+sum_sin))-atan2(-2,LSR_middle_distance(alpha,beta,d)));
}

[[gnu::const]]
static double LSR_last_distance(double alpha, double beta, double d)
{
    double sum_cos = cos(alpha)+cos(beta);
    double sum_sin = sin(alpha)+sin(beta);
    return mod_2pi(-beta + atan2(-sum_cos,(d+sum_sin))-atan2(-2,LSR_middle_distance(alpha,beta,d)));
}

// [[gnu::const]]
// static double LSR_total_distance(double alpha, double beta, double d)
// {
//     return LSR_first_distance(alpha,beta,d) 
//         + LSR_middle_distance(alpha,beta,d) 
//         + LSR_last_distance(alpha,beta,d); 
// }

/********** RLR Lengths **********/

[[gnu::const]]
static double RLR_middle_distance(double alpha, double beta, double d)
{
    double delta_sin = sin(beta)-sin(alpha);
    return acos((6-d*d+2*cos(alpha-beta)-2*d*delta_sin)/8);
}

[[gnu::const]]
static double RLR_first_distance(double alpha, double beta, double d)
{
    double delta_cos = cos(beta)-cos(alpha);
    double delta_sin = sin(beta)-sin(alpha);
    return mod_2pi(alpha - atan2(-delta_cos,(d+delta_sin)) + RLR_middle_distance(alpha,beta,d)/2);
}

[[gnu::const]]
static double RLR_last_distance(double alpha, double beta, double d)
{
    return mod_2pi(alpha-beta-RLR_first_distance(alpha,beta,d)+RLR_middle_distance(alpha,beta,d));
}

// [[gnu::const]]
// static double RLR_total_distance(double alpha, double beta, double d)
// {
//     return RLR_first_distance(alpha,beta,d) 
//         + RLR_middle_distance(alpha,beta,d) 
//         + RLR_last_distance(alpha,beta,d); 
// }

/********** LRL Lengths **********/

[[gnu::const]]
static double LRL_middle_distance(double alpha, double beta, double d)
{
    double delta_sin = sin(beta)-sin(alpha);
    return acos((6-d*d+2*cos(alpha-beta)+2*d*delta_sin)/8);
}

[[gnu::const]]
static double LRL_first_distance(double alpha, double beta, double d)
{
    double delta_cos = cos(beta)-cos(alpha);
    double delta_sin = sin(beta)-sin(alpha);
    return mod_2pi(-alpha+atan2(delta_cos,(d-delta_sin)) + LRL_middle_distance(alpha,beta,d)/2);
}

[[gnu::const]]
static double LRL_last_distance(double alpha, double beta, double d)
{
    return mod_2pi(beta-alpha+LRL_middle_distance(alpha,beta,d) - LRL_first_distance(alpha,beta,d));
}

// [[gnu::const]]
// static double LRL_total_distance(double alpha, double beta, double d)
// {
//     return LRL_first_distance(alpha,beta,d) 
//         + LRL_middle_distance(alpha,beta,d) 
//         + LRL_last_distance(alpha,beta,d); 
// }

/********** SRS Lengths **********/

[[gnu::const]]
static double SRS_first_distance(double alpha, double beta, double d)
{
    // We don't care about the special case where alpha=beta=0 because then the solution
    // is a straight line, which will overlap with RSR
    double da = central_angle(alpha-beta);
    if (ABS(da) - M_PI == 0)
    {
        return NAN;
    }

    double cs = d * ABS(sin(beta)/sin(da));

    double output = cs - tan(da/2);  
    return (output >= 0) ? output : NAN;
}

[[gnu::const]]
static double SRS_middle_distance(double alpha, double beta, [[maybe_unused]] double d)
{
    return mod_2pi(alpha-beta);
}

[[gnu::const]]
static double SRS_last_distance(double alpha, double beta, double d)
{
    double da = central_angle(alpha-beta);
    if (ABS(da) - M_PI == 0)
    {
        return NAN;
    }
    
    double ec = d * ABS(sin(alpha)/sin(da));

    double output = ec - tan(da/2);
    return (output >= 0) ? output : NAN;
}

// [[gnu::const]]
// static double SRS_total_distance(double alpha, double beta, double d)
// {
//     return SRS_first_distance(alpha,beta,d) 
//         + SRS_middle_distance(alpha,beta,d) 
//         + SRS_last_distance(alpha,beta,d); 
// }


/********** SLS Lengths **********/

[[gnu::const]]
static double SLS_first_distance(double alpha, double beta, double d)
{
    double da = central_angle(beta-alpha);
    if (ABS(da) - M_PI == 0)
    {
        return NAN;
    }
    
    double cs = d * ABS(sin(beta)/sin(da));

    double output = cs - tan(da/2);
    return (output >= 0) ? output : NAN;
}

[[gnu::const]]
static double SLS_middle_distance(double alpha, double beta, [[maybe_unused]] double d)
{
    return mod_2pi(beta-alpha);
}

[[gnu::const]]
static double SLS_last_distance(double alpha, double beta, double d)
{
    double da = central_angle(beta-alpha);
    if (ABS(da) - M_PI == 0)
    {
        return NAN;
    }
    
    double ec = d * ABS(sin(alpha)/sin(da));

    double output = ec - tan(da/2);
    return (output >= 0) ? output : NAN;
}



// [[gnu::const]]
// static double SLS_total_distance(double alpha, double beta, double d)
// {
//     return SLS_first_distance(alpha,beta,d) 
//         + SLS_middle_distance(alpha,beta,d) 
//         + SLS_last_distance(alpha,beta,d); 
// }

// ******************** Dubins Fitting ******************** //

typedef struct
{
  DubinsType type;
  float radius;
  DubinsElement_t elements[5];
} ExtendedDubins_t;

static ExtendedDubins_t fit_basic_dubins(DubinsType type, Pose2D_t start, Pose2D_t end, float radius)
{
  ExtendedDubins_t output;
  output.type = type;
  output.radius = radius;
  output.elements[0].length = 0.;
  output.elements[1].init_point = start;
  output.elements[4].length = 0.;

  NormalizedDubins_t norm_pb = normalize_poses(start,end);

  switch (type)
  {
  case RSR:
    output.elements[1].length = RSR_first_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[1].radius = -radius;

    output.elements[2].init_point = dubins_element_end(&output.elements[1]);
    output.elements[2].length = RSR_middle_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[2].radius = 0.;

    output.elements[3].init_point = dubins_element_end(&output.elements[2]);
    output.elements[3].length = RSR_last_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[3].radius = -radius;

    output.elements[4].init_point = dubins_element_end(&output.elements[3]);
    break;
  case LSL:
    output.elements[1].length = LSL_first_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[1].radius = radius;

    output.elements[2].init_point = dubins_element_end(&output.elements[1]);
    output.elements[2].length = LSL_middle_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[2].radius = 0.;

    output.elements[3].init_point = dubins_element_end(&output.elements[2]);
    output.elements[3].length = LSL_last_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[3].radius = radius;

    output.elements[4].init_point = dubins_element_end(&output.elements[3]);
    break;
  case RSL:
    output.elements[1].length = RSL_first_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[1].radius = -radius;

    output.elements[2].init_point = dubins_element_end(&output.elements[1]);
    output.elements[2].length = RSL_middle_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[2].radius = 0.;

    output.elements[3].init_point = dubins_element_end(&output.elements[2]);
    output.elements[3].length = RSL_last_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[3].radius = radius;

    output.elements[4].init_point = dubins_element_end(&output.elements[3]);
    break;
  case LSR:
    output.elements[1].length = LSR_first_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[1].radius = radius;

    output.elements[2].init_point = dubins_element_end(&output.elements[1]);
    output.elements[2].length = LSR_middle_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[2].radius = 0.;

    output.elements[3].init_point = dubins_element_end(&output.elements[2]);
    output.elements[3].length = LSR_last_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[3].radius = -radius;

    output.elements[4].init_point = dubins_element_end(&output.elements[3]);
    break;
  case RLR:
    output.elements[1].length = RLR_first_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[1].radius = -radius;

    output.elements[2].init_point = dubins_element_end(&output.elements[1]);
    output.elements[2].length = RLR_middle_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[2].radius = radius;

    output.elements[3].init_point = dubins_element_end(&output.elements[2]);
    output.elements[3].length = RLR_last_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[3].radius = -radius;

    output.elements[4].init_point = dubins_element_end(&output.elements[3]);
    break;
  case LRL:
    output.elements[1].length = LRL_first_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[1].radius = radius;

    output.elements[2].init_point = dubins_element_end(&output.elements[1]);
    output.elements[2].length = LRL_middle_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[2].radius = -radius;

    output.elements[3].init_point = dubins_element_end(&output.elements[2]);
    output.elements[3].length = LRL_last_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[3].radius = radius;

    output.elements[4].init_point = dubins_element_end(&output.elements[3]);
    break;
  case SRS:
    output.elements[1].length = SRS_first_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[1].radius = 0.;

    output.elements[2].init_point = dubins_element_end(&output.elements[1]);
    output.elements[2].length = SRS_middle_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[2].radius = -radius;

    output.elements[3].init_point = dubins_element_end(&output.elements[2]);
    output.elements[3].length = SRS_last_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[3].radius = 0.;

    output.elements[4].init_point = dubins_element_end(&output.elements[3]);
    break;
  case SLS:
    output.elements[1].length = SLS_first_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[1].radius = 0.;

    output.elements[2].init_point = dubins_element_end(&output.elements[1]);
    output.elements[2].length = SLS_middle_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[2].radius = radius;

    output.elements[3].init_point = dubins_element_end(&output.elements[2]);
    output.elements[3].length = SLS_last_distance(norm_pb.alpha,norm_pb.beta,norm_pb.d/radius)*radius;
    output.elements[3].radius = 0.;

    output.elements[4].init_point = dubins_element_end(&output.elements[3]);
    break;
  
  
  default: // Should be unreachable ; create assertion error if reached
    assert(NotExtendedDubins(type));
    break;
  }

  return output;
}

static ExtendedDubins_t fit_dubins(DubinsPb_t* pb)
{
  assert(ValidExtendedDubins(pb->type));

  if (NotExtendedDubins(pb->type))
  {
    return fit_basic_dubins(pb->type,pb->start_p,pb->end_p,pb->radius);
  }
  else
  {
    if (StartExtendedDubins(pb->type))
    {
      Pose2D_t shifted_start = move_forward(&pb->start_p,pb->extra);
      ExtendedDubins_t result = fit_basic_dubins(BaseDubinsType(pb->type),shifted_start,pb->end_p,pb->radius);
      result.elements[0].init_point = pb->start_p;
      result.elements[0].radius = 0.;
      result.elements[0].length = pb->extra;
      return result;
    }
    else if (EndExtendedDubins(pb->type))
    {
      Pose2D_t shifted_end = move_forward(&pb->end_p,-pb->extra);
      ExtendedDubins_t result = fit_basic_dubins(BaseDubinsType(pb->type),pb->start_p,shifted_end,pb->radius);
      result.elements[4].radius = 0.;
      result.elements[4].length = pb->extra;
      return result;
    }
    else // BothExtendedDubins(pb->type)
    {
      Pose2D_t shifted_start = move_forward(&pb->start_p,pb->extra/2);
      Pose2D_t shifted_end = move_forward(&pb->end_p,-pb->extra/2);

      ExtendedDubins_t result = fit_basic_dubins(BaseDubinsType(pb->type),shifted_start,shifted_end,pb->radius);
      result.elements[0].init_point = pb->start_p;
      result.elements[0].radius = 0.;
      result.elements[0].length = pb->extra/2;

      result.elements[4].radius = 0.;
      result.elements[4].length = pb->extra/2;
      return result;
    }
  }

}

// ******************** Mission mode ******************** //

static DubinsPb_t ref_problem;
static DubinsElement_t path_elements[5];
static int curr_path_element = 0;

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_dubins_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
{
  if (flag == MissionInit && nb == 12)
  {
  #ifdef DEBUG
    IPRINTF("Mission init : %d params\n",nb);
  #endif

    ref_problem.start_p.x     = params[0];
    ref_problem.start_p.y     = params[1];
    ref_problem.start_p.theta = params[2];

    ref_problem.end_p.x       = params[3];
    ref_problem.end_p.y       = params[4];
    ref_problem.end_p.theta   = params[5];

    ref_problem.target_alt    = params[6];

    ref_problem.start_time    = params[7];
    ref_problem.end_time      = params[8];

    ref_problem.type          = (int)params[9];
    ref_problem.radius        = params[10];
    ref_problem.extra         = params[11];

    return nav_extended_dubins_init();
  }
  else if (flag == MissionRun)
  {
    return nav_extended_dubins_track();
  }
  

  // not a valid case
  return false;
  //TODO: Add update, with the possibility of changing the end_time
}

#endif

// ******************** Dubins navigation ******************** //

static void draw_dubins_element(DubinsElement_t* el, uint8_t id, uint8_t color)
{

}

static bool track_dubins_element(DubinsElement_t* el, float* remaining_length)
{
  if(el->radius == 0)
  {
    // STRAIGHT
    Pose2D_t endpoint = move_forward(&el->init_point,el->length);
    nav_route_xy(el->init_point.x, el->init_point.y, endpoint.x, endpoint.y);

    /** distance to waypoint **/
    float pw_x = endpoint.x - stateGetPositionEnu_f()->x;
    float pw_y = endpoint.y - stateGetPositionEnu_f()->y;
    *remaining_length = sqrtf(pw_x*pw_x + pw_y*pw_y);

    return (! nav_approaching_xy(endpoint.x, endpoint.y, el->init_point.x, el->init_point.y, CARROT));
  }
  else
  {
    // TURN
    float c = cos(el->init_point.theta);
    float s = sin(el->init_point.theta);

    float midx = el->init_point.x - el->radius*s;
    float midy = el->init_point.y + el->radius*c;

    nav_circle_XY(midx, midy, -el->radius);
    // return (! CloseRadAngles(el->length/el->radius, nav_circle_radians)); // Measure based on travelled angle

    float end_angle =  - (el->init_point.theta + el->length/el->radius);
    float curr_angle = RadOfDeg(NavCircleQdr());

    *remaining_length = (end_angle - curr_angle)*el->radius;
    return (! NavQdrCloseTo(DegOfRad(end_angle)));
  }
}

void extended_dubins_set_start(float x, float y, float theta)
{
  ref_problem.start_p.x = x;
  ref_problem.start_p.y = y;
  ref_problem.start_p.theta = RadOfDeg(theta);
}

void extended_dubins_set_start_wp(uint8_t wp, float theta)
{
  ref_problem.start_p.x = WaypointX(wp);
  ref_problem.start_p.y = WaypointY(wp);
  ref_problem.start_p.theta = RadOfDeg(theta);
}

void extended_dubins_set_end(float x, float y, float a, float theta)
{
  ref_problem.end_p.x = x;
  ref_problem.end_p.y = y;
  ref_problem.end_p.theta = RadOfDeg(theta);
  ref_problem.target_alt = a;
}

void extended_dubins_set_end_wp(uint8_t wp, float theta)
{
  ref_problem.end_p.x = WaypointX(wp);
  ref_problem.end_p.y = WaypointY(wp);
  ref_problem.end_p.theta = RadOfDeg(theta);
  ref_problem.target_alt = WaypointAlt(wp);
}

void extended_dubins_set_radius(float radius)
{
  ref_problem.radius = radius;
}

void extended_dubins_set_pathtype(DubinsType type, float extra)
{
  ref_problem.type = type;
  ref_problem.extra = extra;
}


bool nav_extended_dubins_init()
{
  #if USE_MISSION
  mission_register(nav_dubins_mission,"DUBIN");
  #endif

  // Airspeed mode
  v_ctl_speed_mode = V_CTL_SPEED_AIRSPEED;

  // Handle verticality
  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(ref_problem.target_alt, 0.);

  ExtendedDubins_t sol = fit_dubins(&ref_problem);
  curr_path_element = (NotExtendedDubins(ref_problem.type)) ? 1 : 0;
  for(int i = 0; i < 5; i++)
  {
    path_elements[i] = sol.elements[i];
    #ifdef DEBUG
    IPRINTF("Element %d : Length %.3f ; Radius %.3f ; Start (%.3f , %.3f , %.3f)\n",
        i,
        sol.elements[i].length,
        sol.elements[i].radius,
        sol.elements[i].init_point.x,
        sol.elements[i].init_point.y,
        sol.elements[i].init_point.theta);
    #endif
  }
  return nav_extended_dubins_track();
}

bool nav_extended_dubins_track(void)
{
  // All elements done, return false to finish
  if (curr_path_element > 4)
  {
    #ifdef DEBUG
    IPRINTF("Dubins done!\n");
    #endif
    return false;
  }

  // Current element has null length, skip it and try the next one
  if (path_elements[curr_path_element].length < 1e-6)
  {
    #ifdef DEBUG
    IPRINTF("Section %d is too short!\n",curr_path_element);
    #endif
    curr_path_element++;
    nav_circle_radians = 0.;
    return true;
  }

  float remaining_el_distance;
  bool tracking = track_dubins_element(&path_elements[curr_path_element],&remaining_el_distance);

  // If current element is (almost) done, skip to the next
  if (!tracking)
  {
    #ifdef DEBUG
    IPRINTF("Section %d is done!\n",curr_path_element);
    #endif
    curr_path_element++;
    nav_circle_radians = 0.;
  }
  else
  {
    float remaining_distance = remaining_el_distance;
    for(int i = curr_path_element+1; i <= 4; i++)
    {
      remaining_distance += path_elements[i].length;
    }

    float f_tow = gps.tow / 1000;
    float dt = ref_problem.end_time - f_tow;
    // v_ctl_auto_groundspeed_setpoint = (remaining_distance/dt);
    v_ctl_auto_airspeed_setpoint = (remaining_distance/dt);
  }

  return true;
}

