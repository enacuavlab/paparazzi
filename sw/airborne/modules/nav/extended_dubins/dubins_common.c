#include "dubins_common.h"

#include <assert.h>

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

const char* dubinsTypeStr(DubinsType t)
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


Pose2D_t dubins_element_follow(DubinsElement_t *el, float length)
{
  if (el->radius == 0.)
  {
    return move_forward(&el->init_point,length);
  }
  else
  {
    return move_circle(&el->init_point,length,el->radius);
  }
}

Pose2D_t dubins_element_end(DubinsElement_t *el)
{
  return dubins_element_follow(el,el->length);
}

Pose2D_t dubins_element_velocity(DubinsElement_t *el, float length)
{
  // Speed is always aligned with heading for Dubins paths
  Pose2D_t pose = dubins_element_follow(el,length);
  pose.x = cos(pose.theta);
  pose.y = sin(pose.theta);
  return pose;
}

Pose2D_t dubins_element_accel(DubinsElement_t *el, float length)
{
  Pose2D_t output = {0.,0.,0.};

  // No acceleration in a straight
  if (el->radius == 0.)
  {
    return output;
  }
  else
  {
    // Otherwise, acceleration is purely centripetal (in a circle at constant unit speed)

    Pose2D_t pose = dubins_element_follow(el,length);
    output.x = cos(pose.theta+M_PI_2)/el->radius;
    output.y = sin(pose.theta+M_PI_2)/el->radius;

    return output;
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


// ******************** Dubins Fitting ******************** //

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

ExtendedDubins_t fit_dubins(DubinsPb_t* pb)
{
  assert(ValidExtendedDubins(pb->type));

  ExtendedDubins_t result;
  for(int i = 0; i < 5; i++)
  {
    result.elements[i].length = 0.;
    result.elements[i].radius = 0.;
    result.elements[i].init_point.x     = 0.;
    result.elements[i].init_point.y     = 0.;
    result.elements[i].init_point.theta = 0.;
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
      result.type = pb->type;
      return result;
    }
    else if (EndExtendedDubins(pb->type))
    {
      Pose2D_t shifted_end = move_forward(&pb->end_p,-pb->extra);
      result = fit_basic_dubins(BaseDubinsType(pb->type),pb->start_p,shifted_end,pb->radius, pb->length - pb->extra);
      result.elements[4].radius = 0.;
      result.elements[4].length = pb->extra;
      result.type = pb->type;
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
      result.type = pb->type;
      return result;
    }
  }
}

float dubins_estimate_current_parameter(DubinsPb_t* pb, float nominal_speed, float now)
{
  // Does not make sense when there is no expected time of arrival
  if (pb->end_time < 0)
  {
    return NAN;
  }

  float start_time = pb->end_time - pb->length/nominal_speed;
  return (now-start_time)*nominal_speed;
}

float dubins_find_current_element(DubinsElement_t* el, int* el_num, float length)
{
  int el_i = 0;
  while (el_i < *el_num && length > el[el_i].length)
  {
    length -= el[el_i].length;
    el_i++;
  }

  if (el_i < *el_num)
  {
    *el_num = el_i;
  }
  return length;
}