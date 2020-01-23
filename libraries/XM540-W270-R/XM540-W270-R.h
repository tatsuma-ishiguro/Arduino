/* CONTROL TABLE */
//present as follows
/* 
 * #define (Address) (AddressNumber) // (READonly(R) or WRITE(W) or BOTH(R/W)), (DefaultValue), (dataSize/Range)
 */

#include<DynamixelSDK.h>

// Control Table Address (EEPROM) */
#define ADDR_MODEL_NUMBER           0   //R, -, uint16
#define ADDR_MODEL_INFORMATION      2   //R, 0, uint32
#define ADDR_FIRMWARE_VERSION       6   //R, ?, uint8
#define ADDR_ID                     7   //R/W(NVM), 1, uint8(0 - 252)
#define ADDR_BAUDRATE               8   //R/W(NVM), 1, uint8(0 - 254)
#define ADDR_RETURN_DELAY_TIME      9   //R/W(NVM), 250, uint8(0 - 254)
#define ADDR_DRIVE_MODE             10  //R/W(NVM), 0, uint8(0 - 255)
#define ADDR_OPERATING_MODE         11  //R/W(NVM), 3, uint8(0 - 16)
#define ADDR_SECONDARY_ID           12  //R/W(NVM), 255, uint8(0 - 255)
#define ADDR_PROTOCOL_VERSION       13  //R/W(NVM), 2, uint8(1 - 2)
#define ADDR_HOMING_OFFSET          20  //R/W(NVM), 0, int32(-1044479 - 1044479)
#define ADDR_MOVING_THRESHOLD       24  //R/W(NVM), 10, uint32(0 - 1023)
#define ADDR_TEMPERATURE_LIMIT      31  //R/W(NVM), -, uint8(0 - 100)
#define ADDR_MAX_VOLTAGE_LIMIT      32  //R/W(NVM), -, uint16
#define ADDR_MIN_VOLTAGE_LIMIT      34  //R/W(NVM), -, uint16
#define ADDR_PWM_LIMIT              36  //R/W(NVM), 885, uint16(0 - 885)
#define ADDR_CURRENT_LIMIT          38  //R/W(NVM), -, uint16
#define ADDR_ACCELERATION_LIMIT     40  //R/W(NVM), 32767, uint32(0 - 32676)
#define ADDR_VELOCITY_LIMIT         44  //R/W(NVM), -, uint32(0 - 1023)
#define ADDR_MAX_POSITION_LIMIT     48  //R/W(NVM), 4095, uint32(0 - 4095)
#define ADDR_MIN_POSITION_LIMIT     52  //R/W(NVM), 0, uint32(0 - 4095)
#define ADDR_EXTERNAL_PORT_MODE_1   56  //R/W(NVM), 3, uint8(0 - 3)
#define ADDR_EXTERNAL_PORT_MODE_2   57  //R/W(NVM), 3, uint8(0 - 3)
#define ADDR_EXTERNAL_PORT_MODE_3   58  //R/W(NVM), 3, uint8(0 - 3)
#define ADDR_SHUTDOWN               63  //R/W(NVM), 52(0x34), uint8(0 - 63)

// Control Table Address (RAM) */
#define ADDR_TORQUE_ENABLE                  64  //R/W, 0, uint8(0 - 1)
#define ADDR_LED                            65  //R/W, 0, uint8(0 - 1)
#define ADDR_STATUS_RETURN_LEVEL            68  //R/W, 2, uint8(0 - 2)
#define ADDR_REGISTERED_INSTRUCTION         69  //R, 0, uint8
#define ADDR_HARDWARE_ERROR_STATUS          70  //R, 0, uint8
#define ADDR_VELOCITY_I_GAIN                76  //R/W, -, uint16(0 - 16383)
#define ADDR_VELOCITY_P_GAIN                78  //R/W, -, uint16(0 - 16383)
#define ADDR_POSITION_D_GAIN                80  //R/W, -, uint16(0 - 16383)
#define ADDR_POSITION_I_GAIN                82  //R/W, -, uint16(0 - 16383)
#define ADDR_POSITION_P_GAIN                84  //R/W, -, uint16(0 - 16383)
#define ADDR_FEEDFORWARD_ACCELERATION_GAIN  88  //R/W, 0, uint16(0 - 16383)
#define ADDR_FEEDFORWARD_VELOCITY_GAIN      90  //R/W, 0, uint16(0 - 16383)
#define ADDR_BUS_WATCHDOG                   98  //R/W, 0, int8(-1 - 127)
#define ADDR_GOAL_PWM                       100 //R/W, -, int16(-PWM_LIMIT - PWM_LIMIT)
#define ADDR_GOAL_CURRENT                   102 //R/W, -, int16(-CURRENT_LIMIT - CURRENT_LIMIT)
#define ADDR_GOAL_VELOCITY                  104 //R/W, -, int32(-VELOCITY_LIMIT - VELOCITY_LIMIT)
#define ADDR_PROFILE_ACCELERATION           108 //R/W, 0, uint32(0 - 32767)
#define ADDR_PROFILE_VELOCITY               112 //R/W, 0, uint32(0 - 32767)
#define ADDR_GOAL_POSITION                  116 //R/W, 0, int32(MIN_POSITION_LIMIT - MAX_POSITION_LIMIT)
#define ADDR_REALTIME_TICK                  120 //R, 0, uint8(0 - 1)
#define ADDR_MOVING                         122 //R, 0, uint8
#define ADDR_MOVING_STATUS                  123 //R, 0, uint8
#define ADDR_PRESENT_PWM                    124 //R, -, int16
#define ADDR_PRESENT_CURRENT                126 //R, -, int32
#define ADDR_PRESENT_VELOCITY               128 //R, -, int32
#define ADDR_PRESENT_POSITION               132 //R, -, int32
#define ADDR_VELOCITY_TRAJECTORY            136 //R, -, -
#define ADDR_POSITION_TRAJECTORY            140 //R, -, -
#define ADDR_PRESENT_INPUT_VOLTAGE          144 //R, -, uint16
#define ADDR_PRESENT_TEMPERATURE            146 //R, -, uint8
#define ADDR_EXTERNAL_PORT_DATA_1           152 //R/W, -, uint16
#define ADDR_EXTERNAL_PORT_DATA_2           154 //R/W, -, uint16
#define ADDR_EXTERNAL_PORT_DATA_3           156 //R/W, -, uint16

//Indirect
#define ADDR_INDIRECT_ADDRESS_1     168 //R/W, 224, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_2     170 //R/W, 225, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_3     172 //R/W, 226, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_4     174 //R/W, 227, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_5     176 //R/W, 228, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_6     178 //R/W, 229, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_7     180 //R/W, 230, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_8     182 //R/W, 231, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_9     184 //R/W, 232, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_10    186 //R/W, 233, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_11    188 //R/W, 234, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_12    190 //R/W, 235, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_13    192 //R/W, 236, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_14    194 //R/W, 237, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_15    196 //R/W, 238, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_16    198 //R/W, 239, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_17    200 //R/W, 240, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_18    202 //R/W, 241, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_19    204 //R/W, 242, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_20    206 //R/W, 243, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_21    208 //R/W, 244, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_22    210 //R/W, 245, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_23    212 //R/W, 246, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_24    214 //R/W, 247, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_25    216 //R/W, 248, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_26    218 //R/W, 249, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_27    220 //R/W, 250, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_28    222 //R/W, 251, uint16(64 - 661)
#define ADDR_INDIRECT_DATA_1        224 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_2        225 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_3        226 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_4        227 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_5        228 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_6        229 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_7        230 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_8        231 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_9        232 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_10       233 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_11       234 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_12       235 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_13       236 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_14       237 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_15       238 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_16       239 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_17       240 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_18       241 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_19       242 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_20       243 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_21       244 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_22       245 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_23       246 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_24       247 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_25       248 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_26       249 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_27       250 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_28       251 //R/W, 0, uint8
#define ADDR_INDIRECT_ADDRESS_29    578 //R/W, 634, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_30    580 //R/W, 635, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_31    582 //R/W, 636, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_32    584 //R/W, 637, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_33    586 //R/W, 638, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_34    588 //R/W, 639, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_35    590 //R/W, 640, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_36    592 //R/W, 641, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_37    594 //R/W, 642, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_38    596 //R/W, 643, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_39    598 //R/W, 644, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_40    600 //R/W, 645, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_41    602 //R/W, 646, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_42    604 //R/W, 647, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_43    606 //R/W, 648, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_44    608 //R/W, 649, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_45    610 //R/W, 650, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_46    612 //R/W, 651, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_47    614 //R/W, 652, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_48    616 //R/W, 653, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_49    618 //R/W, 654, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_50    620 //R/W, 655, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_51    622 //R/W, 656, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_52    624 //R/W, 657, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_53    626 //R/W, 658, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_54    628 //R/W, 659, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_55    630 //R/W, 660, uint16(64 - 661)
#define ADDR_INDIRECT_ADDRESS_56    632 //R/W, 661, uint16(64 - 661)
#define ADDR_INDIRECT_DATA_29       634 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_30       635 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_31       636 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_32       637 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_33       638 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_34       639 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_35       640 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_36       641 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_37       642 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_38       643 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_39       644 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_40       645 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_41       646 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_42       647 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_43       648 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_44       649 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_45       650 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_46       651 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_47       652 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_48       653 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_49       654 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_50       655 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_51       656 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_52       657 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_53       658 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_54       659 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_55       660 //R/W, 0, uint8
#define ADDR_INDIRECT_DATA_56       661 //R/W, 0, uint8


/* OPERATING MODE */
#define CURRENT_CONTROL_MODE                0
#define VELOCITY_CONTROL_MODE               1
#define POSITION_CONTROL_MODE               3   
#define EXTENDTED_POSITION_CONTROL_MODE     4
#define CURRENT_BASE_POSITION_CONTROL_MODE  5
#define PWM_CONTROL_MODE                    16

/* Coefficient */
#define SCALING_FACTOR          2.69
#define VELOCITY_COEFFICIENT    0.229

//Protocol Version
#define PROTOCOL_VERSION            2.0

/* Error */
int dxl_comm_result = COMM_TX_FAIL;
uint8_t dxl_error = 0;

