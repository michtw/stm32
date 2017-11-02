#ifndef __MC_H
#define __MC_H

#define MC_PROTOCOL_CODE_SET_REG         0x01
#define MC_PROTOCOL_CODE_GET_REG         0x02
#define MC_PROTOCOL_CODE_EXECUTE_CMD     0x03
#define MC_PROTOCOL_CODE_STORE_TOADDR    0x04
#define MC_PROTOCOL_CODE_LOAD_FROMADDR   0x05
#define MC_PROTOCOL_CODE_GET_BOARD_INFO  0x06
#define MC_PROTOCOL_CODE_SET_RAMP        0x07
#define MC_PROTOCOL_CODE_GET_REVUP_DATA  0x08
#define MC_PROTOCOL_CODE_SET_REVUP_DATA  0x09
#define MC_PROTOCOL_CODE_SET_CURRENT_REF 0x0A
#define MC_PROTOCOL_CODE_GET_MP_INFO     0x0B

#define MC_PROTOCOL_CMD_START_MOTOR   0x01
#define MC_PROTOCOL_CMD_STOP_MOTOR    0x02
#define MC_PROTOCOL_CMD_STOP_RAMP     0x03
#define MC_PROTOCOL_CMD_RESET         0x04
#define MC_PROTOCOL_CMD_PING          0x05
#define MC_PROTOCOL_CMD_START_STOP    0x06
#define MC_PROTOCOL_CMD_FAULT_ACK     0x07
#define MC_PROTOCOL_CMD_ENCODER_ALIGN 0x08
#define MC_PROTOCOL_CMD_IQDREF_CLEAR  0x09   
#define MC_PROTOCOL_CMD_PFC_ENABLE    0x0A
#define MC_PROTOCOL_CMD_PFC_DISABLE   0x0B
#define MC_PROTOCOL_CMD_PFC_FAULT_ACK 0x0C
#define MC_PROTOCOL_CMD_SC_START      0x0D
#define MC_PROTOCOL_CMD_SC_STOP       0x0E

/* Exported constants --------------------------------------------------------*/
#define FRAME_ERROR_NONE              0x00
#define FRAME_ERROR_TRANSFER_ONGOING  0x01
#define FRAME_ERROR_WAITING_TRANSFER  0x02 
#define FRAME_ERROR_INVALID_PARAMETER 0x03 
#define FRAME_ERROR_TIME_OUT          0x04
#define FRAME_ERROR_INVALID_FRAME     0x05

#define FRAME_MAX_BUFFER_LENGTH  0x80
#define FRAME_HEADER_SIZE        0x02
#define FRAME_CRC_SIZE           0x01
#define FRAME_MAX_SIZE           (FRAME_HEADER_SIZE + FRAME_MAX_BUFFER_LENGTH + FRAME_CRC_SIZE)

#define FRAME_ACK_SIZE           0x01
#define FRAME_ACK_CODE           0xFF

#define ACK_NOERROR 0xF0
#define ACK_ERROR   0xFF

#define START_MOTOR   0x01
#define STOP_MOTOR    0x02

#define USART         "/dev/ttyS0"
#define MOTOR_1       0x20

typedef enum {U8, S8, U16, S16, U32, S32} TYPE_t;

typedef enum {UDRC_STATE_IDLE, UDRC_STATE_REQUESTED, UDRC_STATE_EOC} UDRC_State_t;

/** @defgroup MC_Protocol_REG Motor control protocol registers
* @{
*/
typedef enum {
  MC_PROTOCOL_REG_TARGET_MOTOR,          /* 0   */
  MC_PROTOCOL_REG_FLAGS,                 /* 1   */
  MC_PROTOCOL_REG_STATUS,                /* 2   */
  MC_PROTOCOL_REG_CONTROL_MODE,          /* 3   */
  MC_PROTOCOL_REG_SPEED_REF,             /* 4   */
  MC_PROTOCOL_REG_SPEED_KP,              /* 5   */
  MC_PROTOCOL_REG_SPEED_KI,              /* 6   */
  MC_PROTOCOL_REG_SPEED_KD,              /* 7   */
  MC_PROTOCOL_REG_TORQUE_REF,            /* 8   */
  MC_PROTOCOL_REG_TORQUE_KP,             /* 9   */
  MC_PROTOCOL_REG_TORQUE_KI,             /* 10  */
  MC_PROTOCOL_REG_TORQUE_KD,             /* 11  */
  MC_PROTOCOL_REG_FLUX_REF,              /* 12  */
  MC_PROTOCOL_REG_FLUX_KP,               /* 13  */
  MC_PROTOCOL_REG_FLUX_KI,               /* 14  */
  MC_PROTOCOL_REG_FLUX_KD,               /* 15  */
  MC_PROTOCOL_REG_OBSERVER_C1,           /* 16  */
  MC_PROTOCOL_REG_OBSERVER_C2,           /* 17  */
  MC_PROTOCOL_REG_OBSERVER_CR_C1,        /* 18  */
  MC_PROTOCOL_REG_OBSERVER_CR_C2,        /* 19  */
  MC_PROTOCOL_REG_PLL_KI,                /* 20  */
  MC_PROTOCOL_REG_PLL_KP,                /* 21  */
  MC_PROTOCOL_REG_FLUXWK_KP,             /* 22  */
  MC_PROTOCOL_REG_FLUXWK_KI,             /* 23  */
  MC_PROTOCOL_REG_FLUXWK_BUS,            /* 24  */
  MC_PROTOCOL_REG_BUS_VOLTAGE,           /* 25  */
  MC_PROTOCOL_REG_HEATS_TEMP,            /* 26  */
  MC_PROTOCOL_REG_MOTOR_POWER,           /* 27  */
  MC_PROTOCOL_REG_DAC_OUT1,              /* 28  */
  MC_PROTOCOL_REG_DAC_OUT2,              /* 29  */
  MC_PROTOCOL_REG_SPEED_MEAS,            /* 30  */
  MC_PROTOCOL_REG_TORQUE_MEAS,           /* 31  */
  MC_PROTOCOL_REG_FLUX_MEAS,             /* 32  */
  MC_PROTOCOL_REG_FLUXWK_BUS_MEAS,       /* 33  */
  MC_PROTOCOL_REG_RUC_STAGE_NBR,         /* 34  */
  MC_PROTOCOL_REG_I_A,                   /* 35  */
  MC_PROTOCOL_REG_I_B,                   /* 36  */
  MC_PROTOCOL_REG_I_ALPHA,               /* 37  */
  MC_PROTOCOL_REG_I_BETA,                /* 38  */
  MC_PROTOCOL_REG_I_Q,                   /* 39  */
  MC_PROTOCOL_REG_I_D,                   /* 40  */
  MC_PROTOCOL_REG_I_Q_REF,               /* 41  */
  MC_PROTOCOL_REG_I_D_REF,               /* 42  */
  MC_PROTOCOL_REG_V_Q,                   /* 43  */
  MC_PROTOCOL_REG_V_D,                   /* 44  */
  MC_PROTOCOL_REG_V_ALPHA,               /* 45  */
  MC_PROTOCOL_REG_V_BETA,                /* 46  */
  MC_PROTOCOL_REG_MEAS_EL_ANGLE,         /* 47  */
  MC_PROTOCOL_REG_MEAS_ROT_SPEED,        /* 48  */
  MC_PROTOCOL_REG_OBS_EL_ANGLE,          /* 49  */
  MC_PROTOCOL_REG_OBS_ROT_SPEED,         /* 50  */
  MC_PROTOCOL_REG_OBS_I_ALPHA,           /* 51  */
  MC_PROTOCOL_REG_OBS_I_BETA,            /* 52  */
  MC_PROTOCOL_REG_OBS_BEMF_ALPHA,        /* 53  */
  MC_PROTOCOL_REG_OBS_BEMF_BETA,         /* 54  */
  MC_PROTOCOL_REG_OBS_CR_EL_ANGLE,       /* 55  */
  MC_PROTOCOL_REG_OBS_CR_ROT_SPEED,      /* 56  */
  MC_PROTOCOL_REG_OBS_CR_I_ALPHA,        /* 57  */
  MC_PROTOCOL_REG_OBS_CR_I_BETA,         /* 58  */
  MC_PROTOCOL_REG_OBS_CR_BEMF_ALPHA,     /* 59  */
  MC_PROTOCOL_REG_OBS_CR_BEMF_BETA,      /* 60  */
  MC_PROTOCOL_REG_DAC_USER1,             /* 61  */
  MC_PROTOCOL_REG_DAC_USER2,             /* 62  */
  MC_PROTOCOL_REG_MAX_APP_SPEED,         /* 63  */
  MC_PROTOCOL_REG_MIN_APP_SPEED,         /* 64  */
  MC_PROTOCOL_REG_IQ_SPEEDMODE,          /* 65  */
  MC_PROTOCOL_REG_EST_BEMF_LEVEL,        /* 66  */
  MC_PROTOCOL_REG_OBS_BEMF_LEVEL,        /* 67  */
  MC_PROTOCOL_REG_EST_CR_BEMF_LEVEL,     /* 68  */
  MC_PROTOCOL_REG_OBS_CR_BEMF_LEVEL,     /* 69  */
  MC_PROTOCOL_REG_FF_1Q,                 /* 70  */
  MC_PROTOCOL_REG_FF_1D,                 /* 71  */
  MC_PROTOCOL_REG_FF_2,                  /* 72  */
  MC_PROTOCOL_REG_FF_VQ,                 /* 73  */
  MC_PROTOCOL_REG_FF_VD,                 /* 74  */
  MC_PROTOCOL_REG_FF_VQ_PIOUT,           /* 75  */
  MC_PROTOCOL_REG_FF_VD_PIOUT,           /* 76  */
  MC_PROTOCOL_REG_PFC_STATUS,            /* 77  */
  MC_PROTOCOL_REG_PFC_FAULTS,            /* 78  */
  MC_PROTOCOL_REG_PFC_DCBUS_REF,         /* 79  */
  MC_PROTOCOL_REG_PFC_DCBUS_MEAS,        /* 80  */
  MC_PROTOCOL_REG_PFC_ACBUS_FREQ,        /* 81  */
  MC_PROTOCOL_REG_PFC_ACBUS_RMS,         /* 82  */
  MC_PROTOCOL_REG_PFC_I_KP,              /* 83  */
  MC_PROTOCOL_REG_PFC_I_KI,              /* 84  */
  MC_PROTOCOL_REG_PFC_I_KD,              /* 85  */
  MC_PROTOCOL_REG_PFC_V_KP,              /* 86  */
  MC_PROTOCOL_REG_PFC_V_KI,              /* 87  */
  MC_PROTOCOL_REG_PFC_V_KD,              /* 88  */
  MC_PROTOCOL_REG_PFC_STARTUP_DURATION,  /* 89  */
  MC_PROTOCOL_REG_PFC_ENABLED,           /* 90  */
  MC_PROTOCOL_REG_RAMP_FINAL_SPEED,      /* 91  */
  MC_PROTOCOL_REG_RAMP_DURATION,         /* 92  */
  MC_PROTOCOL_REG_HFI_EL_ANGLE,          /* 93  */
  MC_PROTOCOL_REG_HFI_ROT_SPEED,         /* 94  */
  MC_PROTOCOL_REG_HFI_CURRENT,           /* 95  */
  MC_PROTOCOL_REG_HFI_INIT_ANG_PLL,      /* 96  */
  MC_PROTOCOL_REG_HFI_INIT_ANG_SAT_DIFF, /* 97  */
  MC_PROTOCOL_REG_HFI_PI_PLL_KP,         /* 98  */
  MC_PROTOCOL_REG_HFI_PI_PLL_KI,         /* 99  */
  MC_PROTOCOL_REG_HFI_PI_TRACK_KP,       /* 100 */
  MC_PROTOCOL_REG_HFI_PI_TRACK_KI,       /* 101 */
  MC_PROTOCOL_REG_SC_CHECK,              /* 102 */
  MC_PROTOCOL_REG_SC_STATE,              /* 103 */
  MC_PROTOCOL_REG_SC_RS,                 /* 104 */
  MC_PROTOCOL_REG_SC_LS,                 /* 105 */
  MC_PROTOCOL_REG_SC_KE,                 /* 106 */
  MC_PROTOCOL_REG_SC_VBUS,               /* 107 */
  MC_PROTOCOL_REG_SC_MEAS_NOMINALSPEED,  /* 108 */
  MC_PROTOCOL_REG_SC_STEPS,              /* 109 */
  MC_PROTOCOL_REG_SPEED_KP_DIV,          /* 110 */
  MC_PROTOCOL_REG_SPEED_KI_DIV,          /* 111 */
  MC_PROTOCOL_REG_UID,                   /* 112 */
  MC_PROTOCOL_REG_HWTYPE,                /* 113 */
  MC_PROTOCOL_REG_CTRBDID,               /* 114 */
  MC_PROTOCOL_REG_PWBDID,                /* 115 */
  MC_PROTOCOL_REG_SC_PP,                 /* 116 */
  MC_PROTOCOL_REG_SC_CURRENT,            /* 117 */
  MC_PROTOCOL_REG_SC_SPDBANDWIDTH,       /* 118 */
  MC_PROTOCOL_REG_SC_LDLQRATIO,          /* 119 */
  MC_PROTOCOL_REG_SC_NOMINAL_SPEED,      /* 120 */
  MC_PROTOCOL_REG_SC_CURRBANDWIDTH,      /* 121 */
  MC_PROTOCOL_REG_SC_J,                  /* 122 */
  MC_PROTOCOL_REG_SC_F,                  /* 123 */
  MC_PROTOCOL_REG_SC_MAX_CURRENT,        /* 124 */
  MC_PROTOCOL_REG_SC_STARTUP_SPEED,      /* 125 */
  MC_PROTOCOL_REG_SC_STARTUP_ACC,        /* 126 */
  MC_PROTOCOL_REG_SC_PWM_FREQUENCY,      /* 127 */
  MC_PROTOCOL_REG_SC_FOC_REP_RATE,       /* 128 */
  MC_PROTOCOL_REG_PWBDID2,               /* 129 */
  MC_PROTOCOL_REG_SC_COMPLETED,          /* 130 */

  MC_PROTOCOL_REG_MOTOR_POSITION,        /* 131 */
  MC_PROTOCOL_REG_ACC_DURATION,          /* 132 */
  MC_PROTOCOL_REG_SLOW_SOWN_DURATION,    /* 133 */

  MC_PROTOCOL_REG_UNDEFINED
} MC_Protocol_REG_t;

/** @defgroup DAC_Channels DAC channels
* @{
*/
typedef enum {
	DAC_CH0,
	DAC_CH1,
	DAC_CH2,
	DAC_CH3,
	DAC_MAX_Channel_nbr
} DAC_Channel_t;
/**
  * @}
  */

/** @defgroup DAC_UserChannel DAC user channels
* @{
*/
typedef enum {
	DAC_USER1,
	DAC_USER2
} DAC_UserChannel_t;

typedef struct Framedata_s {
    uint8_t Code;
    uint8_t Size;
    uint8_t Buffer[FRAME_MAX_BUFFER_LENGTH];
    uint8_t nCRC;
} FrameData_t, *PFrameData_t;


#endif /* __MC_H */

