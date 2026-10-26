//
// Created by LEGION on 26-1-12.
//

#ifndef SHOOT_BEHAVIOUR_H
#define SHOOT_BEHAVIOUR_H


typedef enum {
    SHOOT_OPEN,
    SHOOT_STOP,
    SHOOT_REVERSE,
    FRIC_OPEN,
    FRIC_STOP,
} shoot_behaviour_all;

shoot_behaviour_all detect_shoot_mode_switch(void);
extern shoot_behaviour_all shoot_behaviour_mode;
void shoot_behaviour_mode_set() ;

#endif //SHOOT_BEHAVIOUR_H
