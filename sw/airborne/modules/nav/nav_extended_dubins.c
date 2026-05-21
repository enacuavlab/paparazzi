#include "nav_extended_dubins.h"

#include <assert.h>
#include "math.h"

#include "firmwares/fixedwing/nav.h"
#include "firmwares/fixedwing/guidance/guidance_v.h"

#include "modules/gps/gps.h"
#include "modules/datalink/downlink.h"
#include "modules/display/draw.h"


#define DEBUG 1

#ifdef DEBUG
#include <stdio.h>

bool dubins_draw = true;

#define IPRINTF(...) printf("%d : ",AC_ID) ; printf(__VA_ARGS__)

#else

bool dubins_draw = false;
#define IPRINTF(...)
#endif

int dubins_draw_samples = 10;

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

static const char* dubinsTypeStr(DubinsType t)
{
  switch (t)
  {
  case RSR:
    return "RSR";
  case LSL:
    return "LSL";
  case RSL:
    return "RSL";
  case LSR:
    return "LSR";
  case RLR:
    return "RLR";
  case LRL:
    return "LRL";
  case SLS:
    return "SLS";
  case SRS:
    return "SRS";

  case S_RSR:
    return "SRSR";
  case S_LSL:
    return "SLSL";
  case S_RSL:
    return "SRSL";
  case S_LSR:
    return "SLSR";
  case S_RLR:
    return "SRLR";
  case S_LRL:
    return "SLRL";
  case S_SLS:
    return "SSLS";
  case S_SRS:
    return "SSRS";

  case RSR_S:
    return "RSRS";
  case LSL_S:
    return "LSLS";
  case RSL_S:
    return "RSLS";
  case LSR_S:
    return "LSRS";
  case RLR_S:
    return "RLRS";
  case LRL_S:
    return "LRLS";
  case SLS_S:
    return "SLSS";
  case SRS_S:
    return "SRSS";

  case S_RSR_S:
    return "SRSRS";
  case S_LSL_S:
    return "SLSLS";
  case S_RSL_S:
    return "SRSLS";
  case S_LSR_S:
    return "SLSRS";
  case S_RLR_S:
    return "SRLRS";
  case S_LRL_S:
    return "SLRLS";
  case S_SLS_S:
    return "SSLSS";
  case S_SRS_S:
    return "SSRSS";
  
  default:
    return "UNDEF";
  }
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

static ExtendedDubins_t fit_basic_dubins(DubinsType type, Pose2D_t start, Pose2D_t end, float radius, float length_hint)
{
  ExtendedDubins_t output;
  for(int i = 0; i < 5; i++)
  {
    output.elements[i].length = 0.;
    output.elements[i].radius = 0.;
  }
  output.type = type;
  output.radius = radius;
  output.elements[1].init_point = start;

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

  // Apply hint fixing
  if (length_hint > 0)
  {
    float total_length = output.elements[1].length + output.elements[2].length + output.elements[3].length;
    float length_error = total_length - length_hint;

    // A "significant" error
    if (ABS(length_error) > 1e-2)
    {
      // Try fixing by replacing full circles by nothing or vice-versa
      for(int i = 1; i <= 3; i++)
      {
        // Skip straights
        if (output.elements[i].radius != 0)
        {
          if (length_error > 0)
          {
            // Try replacing by nothing if it is a full circle
            if (ABS(output.elements[i].length - ABS(2*M_PI*output.elements[i].radius)) < 1e-2)
            {
              float new_error = length_error - 2*M_PI*ABS(output.elements[i].radius);
              if (ABS(new_error) < ABS(length_error))
              {
                length_error = new_error;
                output.elements[i].length = 0.;
              }
            }
          }
          else
          {
            // Try replacing by a full circle if it is nothing
            if (ABS(output.elements[i].length) < 1e-2)
            {
              float new_error = length_error + 2*M_PI*ABS(output.elements[i].radius);
              if (ABS(new_error) < ABS(length_error))
              {
                length_error = new_error;
                output.elements[i].length = 2*M_PI*ABS(output.elements[i].radius);
              }
            }
          }
        }
      }
    }
  }

  return output;
}

static ExtendedDubins_t fit_dubins(DubinsPb_t* pb)
{
  assert(ValidExtendedDubins(pb->type));

  ExtendedDubins_t result;
  for(int i = 0; i < 5; i++)
  {
    result.elements[i].length = 0.;
    result.elements[i].radius = 0.;
  }

  if (NotExtendedDubins(pb->type))
  {
    return fit_basic_dubins(pb->type,pb->start_p,pb->end_p,pb->radius, pb->length);
  }
  else
  {
    if (StartExtendedDubins(pb->type))
    {
      Pose2D_t shifted_start = move_forward(&pb->start_p,pb->extra);
      result = fit_basic_dubins(BaseDubinsType(pb->type),shifted_start,pb->end_p,pb->radius, pb->length - pb->extra);
      result.elements[0].init_point = pb->start_p;
      result.elements[0].radius = 0.;
      result.elements[0].length = pb->extra;
      return result;
    }
    else if (EndExtendedDubins(pb->type))
    {
      Pose2D_t shifted_end = move_forward(&pb->end_p,-pb->extra);
      result = fit_basic_dubins(BaseDubinsType(pb->type),pb->start_p,shifted_end,pb->radius, pb->length - pb->extra);
      result.elements[4].radius = 0.;
      result.elements[4].length = pb->extra;
      return result;
    }
    else // BothExtendedDubins(pb->type)
    {
      Pose2D_t shifted_start = move_forward(&pb->start_p,pb->extra/2);
      Pose2D_t shifted_end = move_forward(&pb->end_p,-pb->extra/2);

      result = fit_basic_dubins(BaseDubinsType(pb->type),shifted_start,shifted_end,pb->radius, pb->length - pb->extra);
      result.elements[0].init_point = pb->start_p;
      result.elements[0].radius = 0.;
      result.elements[0].length = pb->extra/2;

      result.elements[4].radius = 0.;
      result.elements[4].length = pb->extra/2;
      return result;
    }
  }

}

// ******************** Dubins navigation ******************** //

static DubinsPb_t ref_problem;
static DubinsElement_t path_elements[5];
static int curr_path_element = 0;
static float initial_nav_rad_angle = NAN;

static struct LlaCoor_i lla_i_from_enu_xyz_f(float x, float y, float z)
{
  struct LlaCoor_i lla_coords_i;

  if (state.utm_initialized_f)
  {
    struct UtmCoor_f utm_coords = state.utm_origin_f;
    utm_coords.east += x;
    utm_coords.north += y;
    utm_coords.alt += z;

    struct UtmCoor_i utm_coords_i;
    UTM_BFP_OF_REAL(utm_coords_i,utm_coords);

    lla_of_utm_i(&lla_coords_i,&utm_coords_i);
  }
  else // Assume state.ned_initialized
  {
    struct EnuCoor_f enu_coords;
    enu_coords.x = x;
    enu_coords.y = y;
    enu_coords.z = z;

    if (state.ned_initialized_i)
    {
      struct EnuCoor_i enu_coords_i;
      ENU_BFP_OF_REAL(enu_coords_i,enu_coords);

      struct EcefCoor_i ecef_coords_i;
      ecef_of_enu_pos_i(&ecef_coords_i,&state.ned_origin_i,&enu_coords_i);
      lla_of_ecef_i(&lla_coords_i,&ecef_coords_i);
    }
    else // state.ned_initialized_f
    {
      struct EcefCoor_f ecef_coords_f;
      ecef_of_enu_vect_f(&ecef_coords_f,&state.ned_origin_f,&enu_coords);

      struct LlaCoor_f lla_coords_f;
      lla_of_ecef_f(&lla_coords_f,&ecef_coords_f);

      LLA_BFP_OF_REAL(lla_coords_i,lla_coords_f);
    }
  }
  return lla_coords_i;
}

static struct LlaCoor_i lla_i_from_enu_f(struct EnuCoor_f coords)
{
  return lla_i_from_enu_xyz_f(coords.x,coords.y,coords.z);
}

/**
 * @brief Send a DRAW message to draw the given Dubins shape
 * 
 * @param el        Shape to draw
 * @param id        Id associated to drawing
 * @param color     Color field (opacity,line color,fill color)
 * @param samples   Number of samples. Irrelevant for lines. 
 *                  For circles, if less than 2, the whole circle is drawn ; otherwise, the number of samples is used for a broken line approx
 */
static void draw_dubins_element(DubinsElement_t* el, uint8_t id, uint8_t color, uint8_t samples)
{
  if (el->radius == 0.)
  {
    // STRAIGHT
    struct LlaCoor_i startpoint_coords = lla_i_from_enu_xyz_f(el->init_point.x,el->init_point.y,0.);

    Pose2D_t endpoint = move_forward(&el->init_point,el->length);
    struct LlaCoor_i endpoint_coords = lla_i_from_enu_xyz_f(endpoint.x,endpoint.y,0.);

    uint8_t shape = DRAW_LINE;
    uint8_t status = DRAW_CREATE;
    float radius = 0.;

    int32_t lats[2] = {startpoint_coords.lat,endpoint_coords.lat};
    int32_t lons[2] = {startpoint_coords.lon,endpoint_coords.lon};

    char txt[1] = " ";

    DOWNLINK_SEND_DRAW(DefaultChannel, DefaultDevice,
      &id,&color,&shape,&status,&radius,
      2,lats,2,lons,1,txt);
  }
  else
  {
    // CIRCLE
    if (samples < 2)
    {
      // Draw full circle
      float c = cos(el->init_point.theta);
      float s = sin(el->init_point.theta);

      float midx = el->init_point.x - el->radius*s;
      float midy = el->init_point.y + el->radius*c;
      struct LlaCoor_i mid = lla_i_from_enu_xyz_f(midx,midy,0.);

      uint8_t shape = DRAW_CIRCLE;
      uint8_t status = DRAW_CREATE;
      float radius = el->radius;

      char txt[1] = " ";

      DOWNLINK_SEND_DRAW(DefaultChannel, DefaultDevice,
        &id,&color,&shape,&status,&radius,
        1,&mid.lat,1,&mid.lon,1,txt);
    }
    else
    {
      // Sample then draw broken line
      assert(samples <= 11); // Make sure it respects the 100 bytes limit !

      int32_t lats[11];
      int32_t lons[11];

      for(int i = 0; i < samples; i++)
      {
        Pose2D_t p = move_circle(&el->init_point,i*(el->length/(samples-1)),el->radius);
        struct LlaCoor_i coords = lla_i_from_enu_xyz_f(p.x,p.y,0.);

        lats[i] = coords.lat;
        lons[i] = coords.lon;
      }

      uint8_t shape = DRAW_LINE;
      uint8_t status = DRAW_CREATE;
      float radius = 0.;

      char txt[1] = " ";

      DOWNLINK_SEND_DRAW(DefaultChannel, DefaultDevice,
        &id,&color,&shape,&status,&radius,
        samples,lats,samples,lons,1,txt);
    }
  }
}

static void remove_drawn(uint8_t id)
{
  uint8_t color = 0;
  uint8_t shape = 0;
  uint8_t status = DRAW_DELETE;
  float radius = 0.;

  int32_t lat = 0.;
  int32_t lon = 0.;

  char txt[1] = " ";

  DOWNLINK_SEND_DRAW(DefaultChannel, DefaultDevice,
    &id,&color,&shape,&status,&radius,
    1,&lat,1,&lon,1,txt);
}

static inline uint8_t make_draw_id(uint8_t el_id)
{
  return (AC_ID << 3) + el_id;
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

    return (! nav_approaching_xy(endpoint.x, endpoint.y, el->init_point.x, el->init_point.y, 1.));
  }
  else
  {
    // TURN
    float c = cos(el->init_point.theta);
    float s = sin(el->init_point.theta);

    float midx = el->init_point.x - el->radius*s;
    float midy = el->init_point.y + el->radius*c;

    nav_circle_XY(midx, midy, -el->radius);
    if (isnan(initial_nav_rad_angle))
    {
      initial_nav_rad_angle = nav_circle_radians;
    }

    float turn_amount_rad = el->length/el->radius;

    float end_orientation = el->init_point.theta + turn_amount_rad;
    NormCourseRad(end_orientation);
    float end_trigo_qdr = end_orientation + ((el->radius > 0) ? (-M_PI_2) : (M_PI_2));
    float end_rad_qdr = M_PI_2 - end_trigo_qdr;
    NormCourseRad(end_rad_qdr);
 
    float curr_rad_qdr = RadOfDeg(NavCircleQdr());

    float dtheta = end_rad_qdr - curr_rad_qdr;
    NormCourseRad(dtheta);
    if (el->radius > 0)
    {
      *remaining_length = dtheta*el->radius;
    }
    else
    {
      *remaining_length = -(2*M_PI - dtheta)*el->radius;
    }

    // IPRINTF("Expected turning: %.2f ° ; Current turning : %.2f °\n",
    //   DegOfRad(el->length/el->radius),
    //   DegOfRad(nav_circle_radians-initial_nav_rad_angle)
    // );

    float turning_done_rad = nav_circle_radians-initial_nav_rad_angle;
    bool at_least_half_turn;
    if (turn_amount_rad < 0)
    {
      at_least_half_turn = (turning_done_rad < turn_amount_rad/2);
    }
    else
    {
      at_least_half_turn = (turning_done_rad > turn_amount_rad/2);
    }

    return !(NavQdrCloseTo(DegOfRad(end_rad_qdr)) && at_least_half_turn);
  }
}
void extended_dubins_set_start(float x, float y, float theta)
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


void dubins_setup()
{
  // Airspeed mode
  v_ctl_speed_mode = V_CTL_SPEED_AIRSPEED;

  // Handle verticality
  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(ref_problem.target_alt, 0.);

  ExtendedDubins_t sol = fit_dubins(&ref_problem);
  curr_path_element = (NotExtendedDubins(ref_problem.type)) ? 1 : 0;
  IPRINTF("Path type: %s\n",dubinsTypeStr(sol.type));
  for(int i = 0; i < 5; i++)
  {
    path_elements[i] = sol.elements[i];

    IPRINTF("Element %d : Length %.3f ; Radius %.3f ; Start (%.3f , %.3f , %.3f)\n",
        i,
        sol.elements[i].length,
        sol.elements[i].radius,
        sol.elements[i].init_point.x,
        sol.elements[i].init_point.y,
        sol.elements[i].init_point.theta);

    #if DEBUG
    assert(sol.elements[i].length >= 0.);
    assert(!isnan(sol.elements[i].length));
    #endif
    
    if (dubins_draw)
    {
      uint8_t id = make_draw_id(i);
      uint8_t color = DRAW_make_line(AC_ID);

      remove_drawn(id);

      if (sol.elements[i].length > 1e-6)
      {
        draw_dubins_element(&path_elements[i],id,color,dubins_draw_samples);
      }
    }

  }
  initial_nav_rad_angle = NAN;
  return true;
}

bool nav_extended_dubins_track(void)
{
  // All elements done, return false to finish
  if (curr_path_element > 4)
  {
    IPRINTF("Dubins done!\n");
    return false;
  }

  // Current element has null length, skip it and try the next one
  if (path_elements[curr_path_element].length < 1e-6)
  {
    IPRINTF("Section %d is too short!\n",curr_path_element);

    remove_drawn(make_draw_id(curr_path_element));

    curr_path_element++;
    initial_nav_rad_angle = NAN;
    return true;
  }

  float remaining_el_distance;
  bool tracking = track_dubins_element(&path_elements[curr_path_element],&remaining_el_distance);

  // If current element is (almost) done, skip to the next
  if (!tracking)
  {
    IPRINTF("Section %d is done!\n",curr_path_element);

    remove_drawn(make_draw_id(curr_path_element));

    curr_path_element++;
    initial_nav_rad_angle = NAN;
  }
  else
  {
    if (ref_problem.end_time > 0.)
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
  }

  return true;
}

// ******************** Mission mode ******************** //

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_dubins_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
{
  if (flag == MissionInit && nb == 12)
  {
    ref_problem.start_p.x     = params[0];
    ref_problem.start_p.y     = params[1];
    ref_problem.start_p.theta = params[2];

    ref_problem.end_p.x       = params[3];
    ref_problem.end_p.y       = params[4];
    ref_problem.end_p.theta   = params[5];

    ref_problem.target_alt    = params[6];

    ref_problem.length        = params[7];
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

static DubinsElement_t solo_element;

static bool nav_dubins_element_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
{
  float remaining_length;
  if (flag == MissionInit && nb == 6)
  {
    solo_element.init_point.x = params[0];
    solo_element.init_point.y = params[1];
    solo_element.init_point.theta = params[2];
    solo_element.radius = params[3];
    solo_element.length = params[4];
    float altitude = params[5];
    // Airspeed mode
    v_ctl_speed_mode = V_CTL_SPEED_AIRSPEED;

    // Handle verticality
    NavVerticalAutoThrottleMode(0); /* No pitch */
    NavVerticalAltitudeMode(altitude, 0.);

    // draw_dubins_element(&solo_element,make_draw_id(mission.current_idx),DRAW_make_line(AC_ID),dubins_draw_samples);
    initial_nav_rad_angle = NAN;
    return track_dubins_element(&solo_element,&remaining_length);
  }
  else if (flag == MissionRun)
  {
    return track_dubins_element(&solo_element,&remaining_length);
  }

  // not a valid case
  return false;
}

#endif

void dubins_setup()
{
  #if USE_MISSION
  mission_register(nav_dubins_mission,"DUBIN");
  mission_register(nav_dubins_element_mission,"DUBEL");
  #endif

  ref_problem.end_time = 0.;
}

