/*
 * Copyright (C) Aroun Settouraman <aroun.settouraman@alumni.enac.fr>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/nav/nav_thermals.c"
 * @author Aroun Settouraman <aroun.settouraman@alumni.enac.fr>
 * Thermals centering based on energy for gliders
 */

#include "modules/nav/nav_thermals.h"
#include "filters/low_pass_filter.h"
#include "state.h"
#include <stdio.h>
#include "subsystems/abi.h"
#include "generated/airframe.h"


struct NavThermalsParams nav_thermals_params;  //structure of parameters displayed on the GCS
static struct FirstOrderLowPass filter;  //structure for the filter based on the file low_pass_filter.h

float time;
float last_filter;
float filter_value;
float delta = 1/4;

int i = 0;

float t;

float last_energy = 0.0;
float new_energy;
float last_gradient = 3.0;
float new_gradient;


void nav_thermals_init(void)  //initialization of the parameters and of the filters
{
    nav_thermals_params.roll = 15.0;   //parameter dispalyed on thhe GCS
    t = 0;
    init_first_order_low_pass(&filter, 5, 0.25, 15.0);   //filter initialization
    last_filter = 15.0;
}


void nav_thermals_setup(void){      //function that allows to initialize the filter at any moment when called
    init_first_order_low_pass(&filter, t, 0.25, 0);
}

float nav_thermals_run(void){    //main function that returns the value of the roll to the file thermals.xml 
    float airspeed, altitude; 

    airspeed = stateGetAirspeed_f();
    altitude = stateGetPositionEnu_f()->z;
    
    nav_thermals_params.roll = nav_thermals_energy(altitude, airspeed);   //computing the value of the roll

    return nav_thermals_params.roll;
}

float nav_thermals_energy(float altitude, float speed){   //function that calculates the  value of the energy and calculates the gradient 2 times 
    float specific_energy; 
    float gradient;
    float gradient2;
    float roll, roll_filter, new_roll;
    
    specific_energy = (altitude + 0.5 * speed * speed / 9.81);

    new_energy = specific_energy;
    
    if (i < 2){
        new_roll = 15.0;
    }
    if (i >= 1){
        gradient = (new_energy - last_energy) / 0.25;
        
        new_gradient = gradient;        
        
        if (i >= 2){
            gradient2 = (new_gradient - last_gradient) / 0.25;
            
            roll = (16 - 23 * gradient2);   //computing the value of the roll based on the value of the gradient
            
            roll_filter = update_first_order_low_pass(&filter, roll);       //using a low pass filter
            filter_value = roll_filter - last_filter * delta;      //cumputing the gradient part of the filter
            last_filter = filter_value;   
            new_roll = filter_value;
            /*
            if (new_roll >= 40){   //limits for the value of the roll
                new_roll = 40;
            }
            if (new_roll <=0){
                new_roll = 0;
            }
            */
        }
    }
    last_energy = new_energy;
    last_gradient = new_gradient;
    i = i + 1;
    return new_roll;
}





float average(float en[1000], int l){
    float avrg;
    float sum = 0;
    int i = l;
    while(i > l - 70){
        sum = sum + en[i];
        i = i - 1;
        
    }
    
    avrg = sum/70;
    //printf("iteration : %i ; somme : %f ; avrg : %f \n", i, sum, avrg);
    
    return avrg;
}

/*

bool exit_thermal(void){
    
}
*/

float get_acc(void){
    float acc;
    acc = stateGetAccelNed_f()->z;
    printf("vertical acc : %f  \n", acc);
    return acc;
}

int limit = 100;
int incr = 0;

bool exit_thermals(void){   //function that returns true or false indicating if the drone could stay on the thermal or not
    float speed_z;
    bool val = true;
    speed_z = stateGetSpeedEnu_f()->z;
    
    if (speed_z < 0){
        incr = incr + 1;
    }
    else{
        incr = 0;
    }
    
    if (incr >= limit){
        val = false;
        incr = 0;
    }
    
    
    //printf("%i  %f \n", incr, speed_z);
    return val;
}

