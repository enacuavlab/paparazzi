/*Copyright (C) Florian Sansou <florian.sansou@enac.fr> 
 This file is directly generated with MATLAB to obtain the coefficient set of the looping
*/

#define CTRL_HOVER_WIND_NUM_INTEGRATOR_STATE  2
#define CTRL_HOVER_WIND_INPUT 11
#define CTRL_HOVER_WIND_NUM_ACT 4

const float kf = 0.000000018767000;
const float mot_max_speed = 16066.000000000000000;

const float ueq[CTRL_HOVER_WIND_NUM_ACT][1] = {{2.8798},{2.8798},{0},{0}};

const float H[CTRL_HOVER_WIND_NUM_INTEGRATOR_STATE][CTRL_HOVER_WIND_INPUT] = { 
{-0.022486426644278,-0.000045165156909,-0.757045642104201,-0.532835519018862,0.695220582978306,5.949384170164751,1.238779033943984,0.101461509654756,-0.405242798974882,0.304941861611146,0.884966185685174},
{0.572963891724311,0.002729854568607,0.075401827733356,-4.800294095772698,-0.640990215084238,-0.335928361534555,-2.561855043980301,-0.601190395772543,-0.678155848753662,1.868722775790358,-0.756432042216752}
};

const float K[CTRL_HOVER_WIND_NUM_ACT][CTRL_HOVER_WIND_INPUT] = { 
{-0.232198556071961,-0.507552158248380,-4.592718463406880,-13.075619975169410,-41.411800669239888,144.157520727001810,-122.274681472625105,-2.255152889370321,-19.706447659741897,-6.316779544953815,-43.553638918614119},
{-0.232198556071961,0.507552158248380,-4.592718463406880,-13.075619975169410,41.411800669239888,144.157520727001810,122.274681472625105,2.255152889370321,19.706447659741897,6.316779544953815,43.553638918614119},
{3.075186188072180,1.049034929658930,0.310689384692408,-101.910755739046877,-15.692608049696954,-3.396237342642561,-55.179949232465056,-0.941435842355761,-14.096595568363762,57.188522405898993,-9.844658327013402},
{3.075186188072180,-1.049034929658930,0.310689384692408,-101.910755739046877,15.692608049696954,3.396237342642561,55.179949232465056,0.941435842355761,14.096595568363762,57.188522405898993,9.844658327013402}
};

const float num[3] = {0.000000000000000,-0.000423648675924,0.000416664122699};

const float den[3] = {1.000000000000000,-1.931270935519421,0.931270935552772};

