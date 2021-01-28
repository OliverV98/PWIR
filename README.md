# PWIR

## PINOUT ON STM32CUBEIDE

You can find out more information in PCB schematics or from mainboard V2.ioc

```bash
PF1 - GPIO-I ENC3A - Encoder Motor 3 encoder A
PA0 - GPIO-I ENC2_A - Encoder Motor 2 encoder A
PA2 - GPIO-I ENC1A - Encoder Motor 1 encoder A
PA3 - GPIO-I ENC1B - Encoder Motor 1 encoder B
PA4 - GPIO-I ENC3B - Encoder Motor 3 encoder B
PA5 - GPIO-O ESCPWM 
PA15 - GPIO-O SERVOPWM
PA6 - M1-PWM1 - Mootor 1 PWM1
PA7 - M1-PWM2 - Mootor 1 PWM2
PA8 - M2-PWM1 - Mootor 2 PWM1
PA9 - M2-PWM2 - Mootor 2 PWM2
PB6 - M3-PWM1 - Mootor 3 PWM1
PB7 - M3-PWM2 - Mootor 3 PWM2
PA11 - USB_DM
PA12 - USB_DP
PA13 - SWDIO
PA14 - SWCLK
```


## Functionality of motor drivers

```bash
You get information about the postion of the motor only from one encoder. That is done via GPIO-I(look pinout above). That that is used in the mainboard code to calculate PID and set the motor speed accordingly. Motor drivers get their initial PWM from mainboard that are digitally isolated(really intersting how its done).
```

## Functionality of mainboard
```bash
You send the information from the computer via USB and setup the mainboard using SWDIO and SWCLK. That is done using ST-LINK(ask instructors for it). Mainboard sends out sends out in total 8PWM signals (that of 6 are in pairs for motor drivers,1ESC,1 for servo). Becouse TIMERS were not set up correctly we had to use GIPO-O to generate the PWM signals neccesary for the functionality of ESC and SERVO. Also for the same reasons GIPO-I were used for encoder reading.
```
2021 DELTAX
