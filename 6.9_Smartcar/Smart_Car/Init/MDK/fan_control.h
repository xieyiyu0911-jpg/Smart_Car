#ifndef __FAN_CONTROL_H_
#define __FAN_CONTROL_H_

extern uint8 xdata Element_Config;
extern uint8 xdata Element_State;
extern float xdata Flat_distance;
extern float xdata Flat_encoder_ave;


void fan_flatground(void);
void Element_Update(void);

#endif
