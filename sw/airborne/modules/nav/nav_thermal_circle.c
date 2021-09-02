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

/** @file "modules/nav/nav_thermal_circle.c"
 * @author Aroun Settouraman <aroun.settouraman@alumni.enac.fr>
 * Thermals centering based on circles for gliders
 */

#include "modules/nav/nav_thermal_circle.h"
#include "firmwares/fixedwing/nav.h"
#include "firmwares/fixedwing/nav.h"
#include "subsystems/navigation/common_nav.h"
#include "filters/low_pass_filter.h"
#include <stdio.h>


static struct FirstOrderLowPass filter;  //structure for the filter based on the file low_pass_filter.h


int my_id;
float new_x = 50.0;
float new_y = 50.0;
float k_centering = 1.1;
float last_en = 0.0;
int cond = 0;
float last_filter;
float filter_value;
int cond_list = 1;        //création d'une variable pour initialiser la liste des angles (ensemble de portions)
int j = 0;

int nb = 8;               //Définition du nombre de portions
int angles[8];            //A modifier en fonction de la valeur de nb
float list_gradient[8];   //A modifier en fonction de la valeur de nb

void nav_thermal_circle_init(void)
{
    
    init_first_order_low_pass(&filter, 10, 0.25, 0.0);   //initialisation ddu filtre
    last_filter = 1.0;
    
}

void thermal_circle_setup(char id){
    my_id = id;
}


bool circle(void){
    float angle;
    float circle_count;
    float d_climb;
    bool value = true;
    float airspeed, altitude;
    float specific_energy;
    float gradient;
    float en_filter, new_en;
    int i = 0;
    int k = 0;
    float max_gradient = 0.0;
    float max_angle = 0.0;
    float min_gradient = 0.0;
    
    airspeed = stateGetAirspeed_f();
    altitude = stateGetPositionEnu_f()->z;
    
    angle = NavCircleQdr();
    circle_count = NavCircleCount();
    
    if (cond_list == 1){                           //On réalise l'initialisation de la liste des angles (ensemble des portions) une seule fois
        angles[0] = angle;
        for (i=1 ; i<nb ; i++){
            angles[i] = ((int)angles[0] + (360/nb) * i) % 360;     //On partitionne les cercle à partir du moment où le mode therique est activé
        }
        cond_list = 0;        //On change la variable pour ne plus initialiser la liste, on le réalis seulement une fois
    }
    
    specific_energy = (altitude + 0.5 * airspeed * airspeed / 9.81);
    
    if (cond >= 1){          //Cette condition permet d'attendre une itération avant de commencer les calculs de dérivés pour avoir des bonnes valeurs et ne pas partir de 0.0  
        gradient = (specific_energy - last_en)/0.25;   //delta_t
        
        en_filter = update_first_order_low_pass(&filter, gradient);       //filtrage de la valeur
        filter_value = en_filter - last_filter * 0.25;      
        last_filter = filter_value;   
        new_en = filter_value;
        
        if (angle > (int)angles[j] && angle < ((int)angles[j] + 10)){   //Condition permettant d'associer la position angulaire du drone avec la portion dans laquelle il se trouve
            list_gradient[j] = new_en;
            
            for (k=0; k<nb; k++){                  //Recherche du min et du max
                if (max_gradient < list_gradient[k]){
                    max_gradient = list_gradient[k];
                    max_angle = angles[k];
                }
                
                if (min_gradient > list_gradient[k]){
                    min_gradient = list_gradient[k];
                }
            }
            
            
            
            d_climb = max_gradient - min_gradient;     //Calcul de la pente
            
            if (circle_count >= 1){    //Pendant le premier tour, on ne déplace pas le centre du cercle. On attend de trouver le min et max sur un cercle complet
                //new_x = new_x + cos((360 - max_angle) + 90) * d_climb * k_centering;
                //new_y = new_y + sin((360 - max_angle) + 90) * d_climb * k_centering;
                
                new_x = new_x + cos((360 - max_angle + 90) * 2 * PI / 360) * d_climb * k_centering;   //Dans le cos et sin on convertit pour passer de QDR en trigo puis en radian
                new_y = new_y + sin((360- max_angle + 90) * 2 * PI / 360) * d_climb * k_centering;
                
                waypoints[my_id].x = new_x;   //On déplace les waypoints
                waypoints[my_id].y = new_y;
                
            }
            j = (j + 1) % nb;    //On incrémente la variable de parcours de liste des angles
            
        }
        
    }
    
    last_en = specific_energy;   //Energie mis à jour
    
    
    cond = cond + 1;   //On incrémente pour pouvoir permettre le calcul de gradient
    
    return value;
}




