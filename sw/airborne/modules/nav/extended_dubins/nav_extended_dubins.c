#include "nav_extended_dubins.h"

#include "math.h"

#include "firmwares/fixedwing/nav.h"
#include "firmwares/fixedwing/guidance/guidance_v.h"

#include "modules/gps/gps.h"
#include "modules/datalink/downlink.h"
#include "modules/display/draw.h"

#include "filters/low_pass_filter.h"

// #include "nav_dubins_gvf.h"

#define DEBUG 1

#ifdef DEBUG
#include <stdio.h>
#define IPRINTF(...) printf("%d : ",AC_ID) ; printf(__VA_ARGS__)
#else
#define IPRINTF(...)
#endif

bool dubins_draw = true;
int dubins_draw_samples = 10;
float extra_straight_length = 60;
bool mission_init_straight_flag = false;
float end_of_straight_time = 1.;
bool dubins_use_gvf = false;


// ****************************** Dubins navigation ****************************** //

#define EXTENDED_DUBINS_PATH_ELEMENTS_N 7

static DubinsPb_t ref_problem;
static DubinsElement_t path_elements[EXTENDED_DUBINS_PATH_ELEMENTS_N];
static int curr_path_element = 0;
static float initial_nav_rad_angle = NAN;

// ******************** Utils ******************** //

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

// static struct LlaCoor_i lla_i_from_enu_f(struct EnuCoor_f coords)
// {
//   return lla_i_from_enu_xyz_f(coords.x,coords.y,coords.z);
// }

// ******************** Drawing ******************** //

/**
 * @brief Send a DRAW message to draw the given Dubins shape
 * 
 * @param el        Shape to draw
 * @param id        Id associated to drawing
 * @param color     Color field (opacity,line color,fill color)
 * @param samples   Number of samples. Irrelevant for lines. Bounded between 2 and 11 for circles.
 */
static void draw_dubins_element(DubinsElement_t* el, uint8_t id, uint8_t color, uint8_t samples, float wind_x, float wind_y, float reach_time)
{
  if (el->radius == 0.)
  {
    // STRAIGHT
    struct LlaCoor_i startpoint_coords = lla_i_from_enu_xyz_f(el->init_point.x + wind_x * reach_time,el->init_point.y + wind_y * reach_time,0.);
    float reach_end_time = reach_time + el->length/NOMINAL_AIRSPEED;

    Pose2D_t endpoint = dubins_element_end(el);
    struct LlaCoor_i endpoint_coords = lla_i_from_enu_xyz_f(endpoint.x + wind_x * reach_end_time, endpoint.y + wind_y * reach_end_time,0.);

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

    // Sample then draw broken line
    samples = (samples > 11) ? 11 : samples; // Make sure it respects the 100 bytes limit !
    samples = (samples < 2) ? 2 : samples; // Make sure there are at least 2 points to draw a line

    int32_t lats[11];
    int32_t lons[11];

    for(int i = 0; i < samples; i++)
    {
      float dl = i * el->length / (samples - 1);
      Pose2D_t p = dubins_element_follow(el, dl);
      struct LlaCoor_i coords = lla_i_from_enu_xyz_f(p.x + wind_x * (reach_time + dl/NOMINAL_AIRSPEED), p.y + wind_y * (reach_time + dl/NOMINAL_AIRSPEED), 0.);

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

// ******************** Interface setting functions ******************** //

void extended_dubins_set_start(float x, float y, float theta)
{
  ref_problem.start_p.x     = x;
  ref_problem.start_p.y     = y;
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

// ******************** Underlying tracking function ******************** //

static bool track_dubins_element(DubinsElement_t* el, float* remaining_length)
{
  if(el->radius == 0)
  {
    // STRAIGHT
    Pose2D_t endpoint = dubins_element_end(el);
    nav_route_xy(el->init_point.x, el->init_point.y, endpoint.x, endpoint.y);

    /** distance to waypoint **/
    float pw_x = endpoint.x - stateGetPositionEnu_f()->x;
    float pw_y = endpoint.y - stateGetPositionEnu_f()->y;
    *remaining_length = sqrtf(pw_x*pw_x + pw_y*pw_y);

    return (! nav_approaching_xy(endpoint.x, endpoint.y, el->init_point.x, el->init_point.y, end_of_straight_time));
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
    // IPRINTF("Current QDR: %.1f ; Target QDR: %.1f ; Delta (normalized): %.1f ; ",
      // DegOfRad(curr_rad_qdr),
      // DegOfRad(end_rad_qdr),
      // DegOfRad(dtheta));

    // float dx_to_center = midx - stateGetPositionEnu_f()->x;
    // float dy_to_center = midy - stateGetPositionEnu_f()->y;

    // float dist_to_center = sqrtf(dx_to_center*dx_to_center + dy_to_center*dy_to_center);


    if (el->radius > 0)
    {
      *remaining_length = (2*M_PI - dtheta)*el->radius;
      // IPRINTF("Remaining length (going positive): %.2f\n",*remaining_length);
    }
    else
    {
      *remaining_length = -(dtheta)*el->radius;
      // IPRINTF("Remaining length (going negative): %.2f\n",*remaining_length);
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

// ******************** Main functions ******************** //

static struct FirstOrderLowPass speed_sp_filter;

bool nav_extended_dubins_init()
{
  // Airspeed mode
  v_ctl_speed_mode = V_CTL_SPEED_AIRSPEED;
  v_ctl_auto_airspeed_setpoint = NOMINAL_AIRSPEED; // m/s

  init_first_order_low_pass(&speed_sp_filter, 1.,  1./20.f, NOMINAL_AIRSPEED); // Set up a filter with a characteristic time of 1s for 20Hz update rate (nav update rate).

  // Handle verticality
  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(ref_problem.target_alt, 0.);
  // IPRINTF("Target altitude: %.3f\n",ref_problem.target_alt);

  curr_path_element = 0;

  // Extra initial straight at mission start 
  if (mission_init_straight_flag)
  {
    path_elements[0].init_point  = ref_problem.start_p;
    path_elements[0].radius      = 0.;
    path_elements[0].length      = extra_straight_length;

    ref_problem.start_p.x += extra_straight_length * cosf(ref_problem.start_p.theta);
    ref_problem.start_p.y += extra_straight_length * sinf(ref_problem.start_p.theta);

    ref_problem.end_p.x   -= extra_straight_length * cosf(ref_problem.end_p.theta);
    ref_problem.end_p.y   -= extra_straight_length * sinf(ref_problem.end_p.theta);

    path_elements[EXTENDED_DUBINS_PATH_ELEMENTS_N-1].init_point  = ref_problem.end_p;
    path_elements[EXTENDED_DUBINS_PATH_ELEMENTS_N-1].radius      = 0.;
    path_elements[EXTENDED_DUBINS_PATH_ELEMENTS_N-1].length      = extra_straight_length*2; // Include the end and 'pre-plan' the starting straight of the next plan
    
    if (ref_problem.end_time > 0)
    {      
      ref_problem.end_time += extra_straight_length/NOMINAL_AIRSPEED; // Include the 'pre-plan' (the starting straight of the next plan)

      // Deduce the start time from the end time, length and nominal airspeed. Total length is given length plus start extra straight plus two end extra straights
      ref_problem.start_time = ref_problem.end_time - (ref_problem.length+3*extra_straight_length)/NOMINAL_AIRSPEED;
    }
  }
  else
  {
    path_elements[0].radius = 0.;
    path_elements[0].length = 0.;

    path_elements[EXTENDED_DUBINS_PATH_ELEMENTS_N-1].radius = 0.;
    path_elements[EXTENDED_DUBINS_PATH_ELEMENTS_N-1].length = 0.;
  }

  ExtendedDubins_t sol = fit_dubins(&ref_problem);
  
  if (mission_init_straight_flag)
  {
    IPRINTF("Path type: S%sS\n",dubinsTypeStr(sol.type));

    ref_problem.length += 3*extra_straight_length; // Add back the straights' lengths for correct computations after fitting
  }
  else
  {
    IPRINTF("Path type: %s\n",dubinsTypeStr(sol.type));
  }
  
  for(int i = 0; i < 5; i++)
  {
    path_elements[1+i] = sol.elements[i];
  }

  float reach_time = 0.;
  for(int j = 0; j < EXTENDED_DUBINS_PATH_ELEMENTS_N; j++)
  {
    IPRINTF("Element %d : Length %.3f ; Radius %.3f ; Start (%.3f , %.3f , %.3f)\n",
        j,
        path_elements[j].length,
        path_elements[j].radius,
        path_elements[j].init_point.x,
        path_elements[j].init_point.y,
        path_elements[j].init_point.theta);
    
      
    if (dubins_draw)
    {
      uint8_t id = make_draw_id(j);
      uint8_t color;

      // Special case for some demo.
      // A flight-plan based color would be nice...
      switch (AC_ID)
      {
      case 60:
        color = DRAW_make_line(DRAW_BLUE);
        break;

      case 61:
        color = DRAW_make_line(DRAW_WHITE);
        break;

      case 62:
        color = DRAW_make_line(DRAW_RED);
        break;
      
      default:
        color = DRAW_make_line(AC_ID);
        break;
      }
       

      remove_drawn(id);

      if (path_elements[j].length > 1e-6)
      {
        draw_dubins_element(&path_elements[j],id,color,dubins_draw_samples,ref_problem.wind_x,ref_problem.wind_y,reach_time);
      }
    }

    reach_time += path_elements[j].length/NOMINAL_AIRSPEED;
  }

  mission_init_straight_flag = false; // Reset for ulterior utilisation
  initial_nav_rad_angle = NAN;

  // if (dubins_use_gvf)
  // {
    // nav_dubins_gvf_init(&path_elements);
  // }

  return true;
}

bool nav_extended_dubins_track(void)
{
  static float speed_sp = NOMINAL_AIRSPEED;
  float remaining_el_distance;

  // All elements done, return false to finish
  if (curr_path_element >= EXTENDED_DUBINS_PATH_ELEMENTS_N)
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

  bool tracking;

  // Desired position along the path
  float f_tow = ((float)gps_tow_from_sys_ticks(sys_time.nb_tick)) / 1000.f;
  float traveled = dubins_estimate_current_parameter(&ref_problem, NOMINAL_AIRSPEED, f_tow);
  int elements_num = EXTENDED_DUBINS_PATH_ELEMENTS_N;
  float w = dubins_find_current_element(path_elements, &elements_num, traveled);
  DubinsElement_t el_th = path_elements[elements_num];
  Pose2D_t p = dubins_element_follow(&el_th,w);

  // Speed control
  if (ref_problem.end_time > 0.)
  {
    // Current time in seconds since GPS epoch
    float dt = ref_problem.end_time - f_tow;
    float ellapsed = f_tow - ref_problem.start_time;
    
    // Wind induced dift
    float drift_x = ref_problem.wind_x * ellapsed;
    float drift_y = ref_problem.wind_y * ellapsed;

    p.x += drift_x;
    p.y += drift_y;

    // Pre-compute for distance error
    float remaining_th_distance = ref_problem.length - traveled;
    float dx = p.x - stateGetPositionEnu_f()->x;
    float dy = p.y - stateGetPositionEnu_f()->y;
    float theta = atan2f(dy,dx);
    float len_error = sqrtf(dx*dx+dy*dy)*cosf(theta);
    float remaining_distance;

    // Include drift in the tracking of the current element
    DubinsElement_t el = path_elements[curr_path_element];
    if (el.radius != 0.)
    {
      // For a circle, simply drift the center point dynamically
      el.init_point.x += drift_x;
      el.init_point.y += drift_y;
      tracking = track_dubins_element(&el,&remaining_el_distance);
      remaining_distance = remaining_el_distance;
      for(int i = curr_path_element+1; i < EXTENDED_DUBINS_PATH_ELEMENTS_N; i++)
      {
        remaining_distance += path_elements[i].length;
      }
    }
    else
    {
      // For a straight, drift the start and end points of the line according to plan
      // (Yes, we redo it each time instead of changing the plan)
      float distance_to_curr = 0.;
      for(int i = 0; i < curr_path_element; i++)
      {
        distance_to_curr += path_elements[i].length;
      }

      Pose2D_t startpoint = el.init_point;
      Pose2D_t endpoint   = dubins_element_end(&el);
      startpoint.x += (distance_to_curr/NOMINAL_AIRSPEED) * ref_problem.wind_x;
      startpoint.y += (distance_to_curr/NOMINAL_AIRSPEED) * ref_problem.wind_y;

      endpoint.x += ((distance_to_curr+el.length)/NOMINAL_AIRSPEED) * ref_problem.wind_x;
      endpoint.y += ((distance_to_curr+el.length)/NOMINAL_AIRSPEED) * ref_problem.wind_y;

      float dx = endpoint.x - startpoint.x;
      float dy = endpoint.y - startpoint.y;

      startpoint.theta = atan2f(dy,dx);
      DubinsElement_t el_wind = {startpoint,0.,sqrtf(dx*dx+dy*dy)};
      tracking = track_dubins_element(&el_wind,&remaining_el_distance);

      // Pose2D_t end   = dubins_element_end(&el);
      // float dx_to_end = end.x - stateGetPositionEnu_f()->x;
      // float dy_to_end = end.y - stateGetPositionEnu_f()->y;
      // remaining_el_distance = sqrtf(dx_to_end*dx_to_end+dy_to_end*dy_to_end);
      remaining_distance = remaining_th_distance - len_error;
    }

    float current_dt = remaining_distance/stateGetAirspeed_f();
    float new_speed;

    IPRINTF("Current DT: %.2f (s) ; Plan DT: %.2f (s) ; Remaining l: %.2f (m) ; Speed sp: %.2f\n",current_dt,dt,remaining_distance,v_ctl_auto_airspeed_setpoint);

    if (ABS(current_dt - dt) > 0.1)
    {
      if (current_dt > dt)
      {
          new_speed = 1.00*remaining_distance/dt;
      }
      else
      {
          new_speed = 0.97*remaining_distance/dt;
      }
    }
    else
    {
      new_speed = remaining_distance/dt;
    }

    speed_sp = new_speed;
    v_ctl_auto_airspeed_setpoint = update_first_order_low_pass(&speed_sp_filter, speed_sp);
    // IPRINTF("Current DT: %.2f (s) ; Plan DT: %.2f (s) ; Remaining l: %.2f (m) ; Speed sp: %.2f\n",current_dt,dt,remaining_distance,v_ctl_auto_airspeed_setpoint);
  }
  else
  {
    // If no time reference given, cannot estimate drift induced by wind, so just track the element without any compensation 
    tracking = track_dubins_element(&path_elements[curr_path_element],&remaining_el_distance);
  }

  // If current element is (almost) done, skip to the next
  if (!tracking)
  {
    IPRINTF("Section %d is done!\n",curr_path_element);

    remove_drawn(make_draw_id(curr_path_element));

    curr_path_element++;
    initial_nav_rad_angle = NAN;
  }

  if (dubins_draw)
  {
      desired_x = p.x;
      desired_y = p.y;
  }

  return true;
}

// ******************** Mission mode ******************** //

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_dubins_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
{
  if (flag == MissionInit && nb >= 12)
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

    if (nb == 14)
    {
      ref_problem.wind_x = params[12];
      ref_problem.wind_y = params[13];
    }

    mission_init_straight_flag = (extra_straight_length > 0);

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
