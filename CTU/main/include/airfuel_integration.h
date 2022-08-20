/**
 * /////////////////////////// Description ///////////////////////////////
 * Internal structure for CTU airfuel standard reference. It only contains
 * elements that are purely AFA focused. So far, it implements all the
 * definitions and the macros needed for the CRU Control characteristic.
 * 
*/

#ifndef __AIRFUEL_STANDARD_H__
#define __AIRFUEL_STANDARD_H__

// CRU CONTROL
// Enables byte
#define ENABLE_CRU_OUTPUT  (1 << 7)
#define DISABLE_CRU_OUTPUT (0 << 7)

#define ENABLE_CRU_CHARGE_INDICATOR  (1 << 6)
#define DISABLE_CRU_CHARGE_INDICATOR (0 << 6)

#define MAXIMUM_POWER   (00 << 4)
#define TWO_THIRD_POWER (01 << 4)
#define ONE_THIRD_POWER (10 << 4)
#define MINIMUM_POWER   (11 << 4)

#define NO_CRU_EMULATION (000 << 1)
#define EMULATE_CAT_1    (001 << 1)
#define EMULATE_CAT_2    (010 << 1)
#define EMULATE_CAT_3    (011 << 1)
#define EMULATE_CAT_4    (100 << 1)
#define EMULATE_CAT_5    (101 << 1)
#define EMULATE_CAT_6    (110 << 1)

// Permission byte
#define PERMITTED                   0b00000000
#define PERMITTED_WITH_WAITING_TIME 0b00000001
#define DENIED_CROSS_CONNECTION     0b10000000
#define DENIED_AVAILABLE_POWER      0b10000001
#define DENIED_NUMBER_OF_DEVICE     0b10000010
#define DENIED_CLASS_SUPPORT        0b10000011
#define DENIED_HIGH_TEMPERATURE     0b10000100

/* The next 2 signal definitions are examples of what can be implemented
    to signall (through the control characteristic) the CRU of any changes */
#define FIRST_CONNEXION_CRU_CONTROL_VAL DISABLE_CRU_OUTPUT | DISABLE_CRU_CHARGE_INDICATOR | MAXIMUM_POWER | NO_CRU_EMULATION

#define CONTROL_POWER_CRU_VAL(en, rate) (uint8_t)((en) ? (ENABLE_CRU_OUTPUT | ENABLE_CRU_CHARGE_INDICATOR | (rate) | NO_CRU_EMULATION) : \
                                                                (DISABLE_CRU_OUTPUT | DISABLE_CRU_CHARGE_INDICATOR | (rate) | NO_CRU_EMULATION))

#endif