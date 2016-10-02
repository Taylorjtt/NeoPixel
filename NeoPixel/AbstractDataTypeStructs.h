/*
 * AbstractDataTypeStructs.h
 *
 *  Created on: Sep 14, 2016
 *      Author: JohnTaylor
 */

#ifndef ABSTRACTDATATYPESTRUCTS_H_
#define ABSTRACTDATATYPESTRUCTS_H_

typedef struct _LCD_CONTROLLER_OBJ_ *LCDControllerHandle;
typedef struct WIFI_CONTROLLER_OBJ *WifiControllerHandle;
typedef struct _STOPWATCH_OBJ_ *StopwatchHandle;
typedef struct _EX_RES_DETECT_S_ *ExcessiveResistanceDetectorHandle;
typedef struct _AUXILIARY_TIMER_OBJ_ *AuxiliaryTimerHandle;
typedef struct S_ADCHandler *ADCHandlerHandle;
typedef struct S_EncoderHandler *EncoderHandle;
typedef struct _FAULT_DETECTOR_OBJ_ *FaultDetectorHandle;
typedef struct ON_OFF_ANGLE_PAIR_T *OnOffAnglePairHandle;
typedef struct GEOMETRY_LIMITS_T *GeometricLimitsHandle;
typedef struct SYSTEM_GEOMETRY_T *SystemGeometryHandle;
typedef struct _WORKOUT_CONTROLLER_CLASS_ *MyocycleControllerHandle;
typedef struct _USER_PROFILE_OBJ_ *UserProfileHandle;
typedef struct _MOTOR_CONTROLLER_OBJ_ *MotorControllerHandle;
typedef struct _STIMULATION_CONTROLLER_OBJ_ *StimulationControllerHandle;
typedef struct USER_GEOMETRY_T *UserGeometryHandle;
typedef struct TORQUE_TRANSFER_T *TorqueTransferHandle;
typedef struct _TORQUE_ESTIMATOR_OBJ_ *TorqueEstimatorHandle;
typedef struct _TORQUE_MODEL_OBJ_ *WormGearTorqueModelHandle;
typedef struct STIMULATION_REGIONS_T *StimulationRegionsHandle;


#endif /* ABSTRACTDATATYPESTRUCTS_H_ */
