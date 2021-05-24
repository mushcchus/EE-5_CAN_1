/**
 * \file
 *
 * \brief CAN-IDs used for the communication in the car.
 *
 * Use the CAN-spreadsheet (https://docs.google.com/spreadsheets/d/1979RevYD4dG_xUvvYzqewqQUp6rYs6EIdrc7aWQrX6E/edit#gid=6837543)
 * to make sure the IDs don't clash with the reserved IDs from the competition
 * \author Wouter Burns
 */

#ifndef CAN_IDS_H_
#define CAN_IDS_H_

/** \addtogroup can
 * \brief code to control the messages sent and received over the CAN-bus
 *  @{
 */

///@{
///@name CAN-ID Configuration
///@brief used to make the can-ids from the different function codes and node ids
#define FUNCTION_CODE_LENGTH 6 ///< b10..5 of the can id contains the function code
#define NODE_ID_LENGTH 5 ///< b4..0 of the can id contains the node id
///@}

///@{
///@name function code
///@brief all functions in the can-protocol

#define FUN_SCS_WRITE_RESPONSE 0 ///< used as a response to safety critical signals
#define FUN_SCS_WRITE_REQUEST 1 ///< used for sending safety critical signals
#define FUN_STOP_NODE 2 ///< stop a node from sending periodical sensor data over can
#define FUN_START_NODE 3 ///< allow a node to send periodical sensor data
#define FUN_EVENT_RESPONSE 4 ///< acknowledge of an event
#define FUN_EVENT_REQUEST 5 ///< send an event
#define FUN_RD_NODE_STATE 6 ///< ask or answer if a node is in start or stop mode
#define FUN_RD_DATA_RESPONSE 7 ///< send sensor data
#define FUN_RD_DATA_REQUEST 8 ///< ask for a specific sensor
#define FUN_CHANGE_ACT_RESPONSE 9 ///< acknowledge or respons with the new actuator state
#define FUN_CHANGE_ACT_REQUEST 10 ///< ask for the state or change the state of an actuator
#define FUN_CONFIG_ECU_RESPONSE 11 ///< acknowledge the change of an ecu setting
#define FUN_CONFIG_ECU_REQUEST 12 ///< change an ecu setting
#define FUN_CONFIG_SENSOR_NODES_RESPONSE 13 ///< acknowledge the change of a sensor node setting
#define FUN_CONFIG_SENSOR_NODES_REQUEST 14 ///< change a sensor node setting
#define FUN_ECHO_DATA_RESPONSE 15 ///< response on the ECHO_DATA_REQUEST with the same data
#define FUN_ECHO_DATA_REQUEST 16 ///< debug message, ask for a response with the same data
#define FUN_IMU_DATA 17 ///< receive data from the IMU (it can't have a node ID)
#define FUN_TEL_COMMS 18 ///< used for communication between telemetry and datalogger
#define FUN_CONFIG_SAS_NODE_RESPONSE 19 ///< acknowledge the change of a SAS node setting
#define FUN_CONFIG_SAS_NODE_REQUEST 20 ///< change a SAS node setting
#define FUN_STEERING_ACT_REQUEST 21 ///< change the steering actuator (DV only)
#define FUN_STEERING_ACT_RESPONSE 22 ///< answer from the steering actuator (DV only)
///@}

///@{
///@name node id
///@brief all nodes connected to the can-bus

// RESERVED 0
#define NODE_BROADCAST 1 ///< if of a broadcast message
#define NODE_ECU 2 ///< id of the ECU node
#define NODE_BATTERY 3 ///< id of the BATTERY node
#define NODE_ASSI 4 ///< id of the ASSI node
#define NODE_EBS 5 ///< id of the EBS node
#define NODE_DASHBOARD 6 ///< id of the DASHBOARD node
#define NODE_FRONT_RIGHT 7 ///< id of the FRONT_RIGHT node
#define NODE_FRONT_LEFT 8 ///< id of the FRONT_LEFT node
#define NODE_MID_RIGHT 9 ///< id of the MID_RIGHT node
#define NODE_MID_LEFT 10 ///< id of the MID_LEFT node
#define NODE_REAR_RIGHT 11 ///< id of the REAR_RIGHT node
#define NODE_REAR_LEFT 12 ///< id of the REAR_LEFT node
#define NODE_TELEMETRY 13 ///< id of the TELEMETRY node
#define NODE_SAS_REAR 14 ///< id of the SAS_REAR node
#define NODE_SAS_FRONT 15 ///< id of the SAS_FRONT node
#define NODE_ACCEL_FL 16 ///< id of the ACCEL_FL nod

#define NODE_EXTRA1 23

/*
 * _________________________________________________ADDED NODE________________________________________________________
 */
#define NODE_EXTRA 23 ///< id of the FAN_AUDIO node

// RESERVED 17
#define NODE_ACCEL_FR 18 ///< id of the ACCEL_FR node
#define NODE_ACCEL_RL 19 ///< id of the ACCEL_RL node
#define NODE_ACCEL_RR 20 ///< id of the ACCEL_RR node
#define NODE_DATALOGGER 21 ///< id of the DATALOGGER node
///@}

///@{
///@name Event ids
///@brief all events and related constants that can be generated in the car

#define EVENT_COUNT 28 ///< number of events (can be used for an arrray)
#define EVENT_NO_ERROR 0 ///< no error event flag
#define EVENT_ERROR 128 ///< error event flag
#define EVENT_ID_MASK 127 ///< id bit mask
#define EVENT_DEBUG 0 ///< id of the DEBUG event
#define EVENT_SHUT_GLVMS 1 ///< id of the SHUT_GLVMS event
#define EVENT_SHUT_INT_BAT 2 ///< id of the SHUT_INT_BAT event
#define EVENT_SHUT_IMD 3 ///< id of the SHUT_IMD event
#define EVENT_SHUT_AMS 4 ///< id of the SHUT_AMS event
#define EVENT_SHUT_BUTTON_R 5 ///< id of the SHUT_BUTTON_R event
#define EVENT_SHUT_BUTTON_L 6 ///< id of the SHUT_BUTTON_L event
#define EVENT_SHUT_INT_FL 7 ///< id of the SHUT_INT_FL event
#define EVENT_SHUT_BOTS 8 ///< id of the SHUT_BOTS event
#define EVENT_SHUT_INT_FR 9 ///< id of the SHUT_INT_FR event
#define EVENT_SHUT_BUTTON_CP 10 ///< id of the SHUT_BUTTON_CP event
#define EVENT_SHUT_INERTIA 11 ///< id of the SHUT_INERTIA event
#define EVENT_SHUT_INT_RR 12 ///< id of the SHUT_INT_RR event
#define EVENT_SHUT_INT_HVD 13 ///< id of the SHUT_INT_HVD event
#define EVENT_SHUT_INT_RL 14 ///< id of the SHUT_INT_RL event
#define EVENT_SHUT_BSPD 15 ///< id of the SHUT_BSPD event
#define EVENT_SHUT_TSMS 16 ///< id of the SHUT_TSMS event
#define EVENT_BAT_TEMP 17 ///< id of the BAT_TEMP event
#define EVENT_BAT_VOLT_LOW 18 ///< id of the BAT_VOLT_LOW event
#define EVENT_BAT_VOLT_HIGH 19 ///< id of the BAT_VOLT_HIGH event
#define EVENT_BAT_HV 20 ///< Event when the car is charged above 60 V
#define EVENT_ECU_HV_ACT 21 ///< Event when the car can turn on the high voltage
#define EVENT_DASH_START 22 ///< Event when the start button is pressed
#define EVENT_ECU_RTD 23 ///< Event when the car can go into ready to drive
#define EVENT_BATTERY_POWER 24 ///< id of the BATTERY_POWER event
#define EVENT_MOTOR_ERROR 25 ///< id of the MOTOR_ERROR event
#define EVENT_DASH_ECO 26 ///< id of the DASH_ECO event
#define EVENT_DASH_RAIN 27 ///< id of the DASH_RAIN event
///@}

///@{
///@name Actuator IDs
///@brief all actuators inside the car (wheels are controller seperately)

#define ACT_COUNT 36 ///< number of actuators (can be used for an array)
#define ACT_ERROR_DEBUG 0 ///< id of the ERROR_DEBUG
#define ACT_BRAKELIGHT 1 ///< id of the BRAKELIGHT, 0 = off, 1 or greater = on
#define ACT_RTDS 2 ///< id of the RTDS, 0 = off, 1 or greater = on
#define ACT_PUMP 3 ///< id of the PUMP, 0 = off, 1 or greater = on
#define ACT_FAN_LEFT 4 ///< id of the FAN_LEFT, PWM: 0 = off, 255 on
#define ACT_FAN_RIGHT 5 ///< id of the FAN_RIGHT, PWM: 0 = off, 255 on
#define ACT_DRS 6 ///< id of the DRS, 0 = DRS not activated, 1 = DRS activated
#define ACT_SEAT_VALVE 7 ///< id of the SEAT_VALVE, 0 = off, 1 or greater = on
#define ACT_SAS_VALVE_FR_1 8 ///< id of the SAS_VALVE_FR_1, setpoint in mA (16 bit uint, capped on 1.5A)
#define ACT_SAS_VALVE_FR_2 9 ///< id of the SAS_VALVE_FR_2, setpoint in mA (16 bit uint, capped on 1.5A)
#define ACT_SAS_VALVE_FL_1 10 ///< id of the SAS_VALVE_FL_1, setpoint in mA (16 bit uint, capped on 1.5A)
#define ACT_SAS_VALVE_FL_2 11 ///< id of the SAS_VALVE_FL_2, setpoint in mA (16 bit uint, capped on 1.5A)
#define ACT_SAS_VALVE_RR_1 12 ///< id of the SAS_VALVE_RR_1, setpoint in mA (16 bit uint, capped on 1.5A)
#define ACT_SAS_VALVE_RR_2 13 ///< id of the SAS_VALVE_RR_2, setpoint in mA (16 bit uint, capped on 1.5A)
#define ACT_SAS_VALVE_RL_1 14 ///< id of the SAS_VALVE_RL_1, setpoint in mA (16 bit uint, capped on 1.5A)
#define ACT_SAS_VALVE_RL_2 15 ///< id of the SAS_VALVE_RL_2, setpoint in mA (16 bit uint, capped on 1.5A)
#define ACT_EBS_VALVE 16 ///< id of the EBS actuator, for releasing the brake (0 is activated, 1 is released)
#define ACT_EBS_SDC 17 ///< id for opening/closing the shutdown circuit at the EBS (0 is opened, 1 is closed)
///@}

///@{
///@name Sensor IDs
///@brief all sensors and related constants in the car

#define SENSOR_COUNT 130 ///< number of sensors (can be used for an array)
#define SENSOR_ERROR_DEBUG 0 ///< id of the ERROR_DEBUG sensor
#define SENSOR_APPS1 1 ///< id of the APPS1 sensor
#define SENSOR_APPS2 2 ///< id of the APPS2 sensor
#define SENSOR_BRAKE_SENSE 3 ///< id of the BRAKE_SENSE sensor
#define SENSOR_BRAKE_P1 4 ///< id of the BRAKE_P1 sensor
#define SENSOR_BRAKE_P2 5 ///< id of the BRAKE_P2 sensor
#define SENSOR_STEER 6 ///< id of the STEER sensor
#define SENSOR_TIRE_TEMP_FR1 7 ///< id of the TIRE_TEMP_FR1 sensor
#define SENSOR_TIRE_TEMP_FR2 8 ///< id of the TIRE_TEMP_FR2 sensor
#define SENSOR_TIRE_TEMP_FR3 9 ///< id of the TIRE_TEMP_FR3 sensor
#define SENSOR_TIRE_TEMP_FL1 10 ///< id of the TIRE_TEMP_FL1 sensor
#define SENSOR_TIRE_TEMP_FL2 11 ///< id of the TIRE_TEMP_FL2 sensor
#define SENSOR_TIRE_TEMP_FL3 12 ///< id of the TIRE_TEMP_FL3 sensor
#define SENSOR_TIRE_TEMP_RR1 13 ///< id of the TIRE_TEMP_RR1 sensor
#define SENSOR_TIRE_TEMP_RR2 14 ///< id of the TIRE_TEMP_RR2 sensor
#define SENSOR_TIRE_TEMP_RR3 15 ///< id of the TIRE_TEMP_RR3 sensor
#define SENSOR_TIRE_TEMP_RL1 16 ///< id of the TIRE_TEMP_RL1 sensor
#define SENSOR_TIRE_TEMP_RL2 17 ///< id of the TIRE_TEMP_RL2 sensor
#define SENSOR_TIRE_TEMP_RL3 18 ///< id of the TIRE_TEMP_RL3 sensor
#define SENSOR_COOLING_FLOW_R 19 ///< id of the COOLING_FLOW_R sensor
#define SENSOR_COOLING_FLOW_L 20 ///< id of the COOLING_FLOW_L sensor
#define SENSOR_SD_BOTS 21 ///< id of the SD_BOTS sensor
#define SENSOR_SD_EM_CP 22 ///< id of the SD_EM_CP sensor
#define SENSOR_SD_EM_R 23 ///< id of the SD_EM_R sensor
#define SENSOR_SD_EM_L 24 ///< id of the SD_EM_L sensor
#define SENSOR_SD_INT_FR 25 ///< id of the SD_INT_FR sensor
#define SENSOR_SD_INT_FL 26 ///< id of the SD_INT_FL sensor
#define SENSOR_SD_INT_RR 27 ///< id of the SD_INT_RR sensor
#define SENSOR_SD_INT_RL 28 ///< id of the SD_INT_RL sensor
#define SENSOR_SD_INT_BAT 29 ///< id of the SD_INT_BAT sensor
#define SENSOR_SD_INERTIA 30 ///< id of the SD_INERTIA sensor
#define SENSOR_SD_HVD 31 ///< id of the SD_HVD sensor
#define SENSOR_SD_BSPD 32 ///< id of the SD_BSPD sensor
#define SENSOR_SD_TSMS 33 ///< id of the SD_TSMS sensor
#define SENSOR_SUS_FL 34 ///< id of the SUS_FL sensor
#define SENSOR_SUS_FR 35 ///< id of the SUS_FR sensor
#define SENSOR_SUS_RL 36 ///< id of the SUS_RL sensor
#define SENSOR_SUS_RR 37 ///< id of the SUS_RR sensor
#define SENSOR_SAS_ACC_FL 38 ///< id of the SAS_ACC_FL sensor
#define SENSOR_SAS_ACC_FR 39 ///< id of the SAS_ACC_FR sensor
#define SENSOR_SAS_ACC_RL 40 ///< id of the SAS_ACC_RL sensor
#define SENSOR_SAS_ACC_RR 41 ///< id of the SAS_ACC_RR sensor
#define SENSOR_SAS_P1_F 42 ///< id of the SAS_P1_F sensor
#define SENSOR_SAS_P1_R 43 ///< id of the SAS_P1_R sensor
#define SENSOR_SAS_P2_F 44 ///< id of the SAS_P2_F sensor
#define SENSOR_SAS_P2_R 45 ///< id of the SAS_P2_R sensor
#define SENSOR_SAS_TEMP_F 46 ///< id of the SAS_TEMP_F sensor
#define SENSOR_SAS_TEMP_R 47 ///< id of the SAS_TEMP_R sensor
#define SENSOR_SAS_VALVE_I_FR1 48 ///< id of the SAS_VALVE_I_FR1 sensor
#define SENSOR_SAS_VALVE_I_FR2 49 ///< id of the SAS_VALVE_I_FR2 sensor
#define SENSOR_SAS_VALVE_I_FL1 50 ///< id of the SAS_VALVE_I_FL1 sensor
#define SENSOR_SAS_VALVE_I_FL2 51 ///< id of the SAS_VALVE_I_FL2 sensor
#define SENSOR_SAS_VALVE_I_RR1 52 ///< id of the SAS_VALVE_I_RR1 sensor
#define SENSOR_SAS_VALVE_I_RR2 53 ///< id of the SAS_VALVE_I_RR2 sensor
#define SENSOR_SAS_VALVE_I_RL1 54 ///< id of the SAS_VALVE_I_RL1 sensor
#define SENSOR_SAS_VALVE_I_RL2 55 ///< id of the SAS_VALVE_I_RL2 sensor
#define SENSOR_CAR_STATE 56 ///< indicates in which FSM state the car is
#define SENSOR_BATT_LOWEST_CELL_TEMP 57 ///< id of the BATT_LOWEST_CELL_TEMP sensor
#define SENSOR_BATT_HIGHEST_CELL_TEMP 58 ///< id of the BATT_HIGHEST_CELL_TEMP
#define SENSOR_BATT_LOWEST_CELL_VOLTAGE 59 ///< id of the BATT_LOWEST_CELL_VOLTAGE
#define SENSOR_BATT_HIGHEST_CELL_VOLTAGE 60 ///< id of the BATT_HIGHEST_CELL_VOLTAGE
#define SENSOR_BATT_SOC 61 ///< id of the BATT_SOC
#define SENSOR_AMBIENT_TEMP 62 ///< id of the outside temperature
#define SENSOR_MOTOR_FL_TARGET_VEL 63 ///< id of the MOTOR_FL_TARGET_VEL
#define SENSOR_MOTOR_FL_TARGET_TORQUE 64 ///< id of the MOTOR_FL_TARGET_TORQUE
#define SENSOR_MOTOR_FL_VEL 65 ///< id of the MOTOR_FL_VEL
#define SENSOR_MOTOR_FL_TORQUE 66 ///< id of the MOTOR_FL_TORQUE
#define SENSOR_MOTOR_FL_TEMP 67 ///< id of the MOTOR_FL_TEMP
#define SENSOR_MOTOR_FR_TARGET_VEL 68 ///< id of the MOTOR_FR_TARGET_VEL
#define SENSOR_MOTOR_FR_TARGET_TORQUE 69 ///< id of the MOTOR_FR_TARGET_TORQUE
#define SENSOR_MOTOR_FR_VEL 70 ///< id of the MOTOR_FR_VEL
#define SENSOR_MOTOR_FR_TORQUE 71 ///< id of the MOTOR_FR_TORQUE
#define SENSOR_MOTOR_FR_TEMP 72 ///< id of the MOTOR_FR_TEMP
#define SENSOR_MOTOR_RL_TARGET_VEL 73 ///< id of the MOTOR_RL_TARGET_VEL
#define SENSOR_MOTOR_RL_TARGET_TORQUE 74 ///< id of the MOTOR_RL_TARGET_TORQUE
#define SENSOR_MOTOR_RL_VEL 75 ///< id of the MOTOR_RL_VEL
#define SENSOR_MOTOR_RL_TORQUE 76 ///< id of the MOTOR_RL_TORQUE
#define SENSOR_MOTOR_RL_TEMP 77 ///< id of the MOTOR_RL_TEMP
#define SENSOR_MOTOR_RR_TARGET_VEL 78 ///< id of the MOTOR_RR_TARGET_VEL
#define SENSOR_MOTOR_RR_TARGET_TORQUE 79 ///< id of the MOTOR_RR_TARGET_TORQUE
#define SENSOR_MOTOR_RR_VEL 80 ///< id of the MOTOR_RR_VEL
#define SENSOR_MOTOR_RR_TORQUE 81 ///< id of the MOTOR_RR_TORQUE
#define SENSOR_MOTOR_RR_TEMP 82 ///< id of the MOTOR_RR_TEMP
#define SENSOR_DRIVE_F_STATUS 83 ///< id of the DRIVE_F_STATUS
#define SENSOR_DRIVE_F_TEMP 84 ///< id of the DRIVE_F_TEMP
#define SENSOR_DRIVE_R_STATUS 85 ///< id of the DRIVE_R_STATUS
#define SENSOR_DRIVE_R_TEMP 86 ///< id of the DRIVE_R_TEMP
#define SENSOR_TEL_GPS_LAT_BYTES_3_2 87 ///< id of the TEL_GPS_LAT_BYTES_3_2 sensor
#define SENSOR_TEL_GPS_LAT_BYTES_1_0 88 ///< id of the TEL_GPS_LAT_BYTES_1_0
#define SENSOR_TEL_GPS_LONG_BYTES_3_2 89 ///< id of the TEL_GPS_LONG_BYTES_3_2
#define SENSOR_TEL_GPS_LONG_BYTES_1_0 90 ///< id of the TEL_GPS_LONG_BYTES_1_0
#define SENSOR_TEL_GPS_VELOCITY 91 ///< id of the TEL_GPS_VELOCITY
#define SENSOR_IMU_ACC_X 92 ///< id of the IMU_ACC_X
#define SENSOR_IMU_ACC_Y 93 ///< id of the IMU_ACC_Y
#define SENSOR_IMU_VEL_X 94 ///< id of the IMU_VEL_X
#define SENSOR_IMU_VEL_Y 95 ///< id of the IMU_VEL_Y
#define SENSOR_IMU_POS_LAT_HIGH 96 ///< id of the IMU_POS_LAT_HIGH
#define SENSOR_IMU_POS_LAT_LOW 97 ///< id of the IMU_POS_LAT_LOW
#define SENSOR_IMU_POS_LONG_HIGH 98 ///< id of the IMU_POS_LONG_HIGH
#define SENSOR_IMU_POS_LONG_LOW 99 ///< id of the IMU_POS_LONG_LOW
#define SENSOR_IMU_HEADING_ANGLE 100 ///< id of the IMU_HEADING_ANGLE
#define SENSOR_IMU_YAW_RATE 101 ///< id of the IMU_YAW_RATE
#define SENSOR_ECU_ACC_X 102 ///< id of the ECU_ACC_X
#define SENSOR_ECU_ACC_Y 103 ///< id of the ECU_ACC_Y sensor
#define SENSOR_ECU_GYRO_X 104 ///< id of the ECU_GYRO_X sensor
#define SENSOR_ECU_GYRO_Y 105 ///< id of the ECU_GYRO_Y sensor
#define SENSOR_LOGGING_ID 106 ///< id of the current logging session
#define SENSOR_WHEEL_SPEED_ENCODER_FL 107 ///< id of the WHEEL_SPEED_ENCODER_FL sensor
#define SENSOR_WHEEL_SPEED_ENCODER_FR 108 ///< id of the WHEEL_SPEED_ENCODER_FR sensor
#define SENSOR_EXTRA1 109 ///< id of the EXTRA1 sensor
#define SENSOR_EXTRA2 110 ///< id of the EXTRA2 sensor
#define SENSOR_EXTRA3 111 ///< id of the EXTRA3 sensor
#define SENSOR_EXTRA4 112 ///< id of the EXTRA4 sensor
#define SENSOR_EXTRA5 113 ///< id of the EXTRA5 sensor
#define SENSOR_EXTRA6 114 ///< id of the EXTRA6 sensor
#define SENSOR_EXTRA7 115 ///< id of the EXTRA7 sensor
#define SENSOR_EXTRA8 116 ///< id of the EXTRA8 sensor
#define SENSOR_EXTRA9 117 ///< id of the EXTRA9 sensor
#define SENSOR_EXTRA10 118 ///< id of the EXTRA10 sensor
#define SENSOR_ECU_VELOCITY 119 ///< id of the ECU_VELOCITY sensor
#define SENSOR_COOLING_FLOW_FL 120 ///< 
#define SENSOR_COOLING_FLOW_FR 121 ///< 
#define SENSOR_COOLING_FLOW_RL 122 ///< 
#define SENSOR_COOLING_FLOW_RR 123 ///< 
#define SENSOR_COOLING_TEMP1 124 ///< 
#define SENSOR_COOLING_TEMP2 125 ///< 
#define SENSOR_COOLING_TEMP3 126 ///< 
#define SENSOR_COOLING_TEMP4 127 ///< 
#define SENSOR_COOLING_TEMP5 128 ///< 
#define SENSOR_COOLING_TEMP6 129 ///< 
///@}

///@{
///@name ECU settings
///@brief all ecu settings

#define ECU_CONFIG_COUNT 6 ///< number of ecu settings (can be used for an array)
#define ECU_CONFIG_RPM 0 ///< id of the RPM setting
#define ECU_CONFIG_POWER_PERC 1 ///< id of the POWER_PERC setting
#define ECU_CONFIG_REGEN_PERC 2 ///< id of the REGEN_PERC setting
#define ECU_CONFIG_BALANCE_FRONT 3 ///< id of the BALANCE_FRONT setting
#define ECU_CONFIG_DYNAMIC_EVENT 4 ///< id of the DYNAMIC_EVENT setting
#define ECU_CONFIG_DRIVE_STATUS_FREQUENCY 5 ///< id of the DRIVE_STATUS_FREQUENCY setting
///@}

///@{
///@name ECU Constants
///@brief all dynamic event settings, the current dynamic event setting is saved in the DYNAMIC_EVENT setting in the ecu

#define ECU_COUNT 19 ///< number of dynamic events
#define ECU_DYN_EV_DEFAULT 0 ///< id of the default mode
#define ECU_DYN_EV_ACC 1 ///< id of the manual acceleration mode
#define ECU_DYN_EV_SKID 2 ///< id of the manual skidpad mode
#define ECU_DYN_EV_AUTOX1 3 ///< id of manual autocross mode 1
#define ECU_DYN_EV_AUTOX2 4 ///< id of manual autocross mode 2
#define ECU_DYN_EV_ENDURANCE1 5 ///< id of endurance mode 1
#define ECU_DYN_EV_ENDURANCE2 6 ///< id of endurance mode 2
#define ECU_DYN_DV_ACC 7 ///< id of the driverless acceleration mode
#define ECU_DYN_DV_SKID 8 ///< id of the driverless skidpad mode
#define ECU_DYN_DV_AUTOX1 9 ///< id of the driverless autocross mode
#define ECU_DYN_DV_TRACKDRIVE1 10 ///< id of the driverless trackdrive mode
#define ECU_FSM_LV_ON 0 ///< 
#define ECU_FSM_TURNING_HV_ON 1 ///< 
#define ECU_FSM_WAITING_FOR_BATTERY 2 ///< 
#define ECU_FSM_HV_ON 3 ///< 
#define ECU_FSM_SWITCH_DC_ON 4 ///< 
#define ECU_FSM_PREPARING_TO_DRIVE 5 ///< 
#define ECU_FSM_RUNNING 6 ///< 
#define ECU_FSM_ERROR_RECOVERY 7 ///< 
///@}

///@{
///@name Energymeter IDs
///@brief all constants related to the energymeter of formula

#define ENERGYMETER_COMMAND 1041 ///< complete id for a COMMAND message for the formula energymeter
#define ENERGYMETER_DEBUG 1296 ///< complete id for a DEBUG message for the formula energymeter
#define ENERGYMETER_RESPONSE 1297 ///< complete id for a RESPONSE message for the formula energymeter
#define ENERGYMETER_CURRENT 1313 ///< complete id for a CURRENT message for the formula energymeter
#define ENERGYMETER_VOLTAGE1 1314 ///< complete id for a VOLTAGE1 message for the formula energymeter
#define ENERGYMETER_VOLTAGE2 1315 ///< complete id for a VOLTAGE2 message for the formula energymeter
#define ENERGYMETER_VOLTAGE3 1316 ///< complete id for a VOLTAGE3 message for the formula energymeter
#define ENERGYMETER_TEMPERATURE 1317 ///< complete id for a TEMPERATURE message for the formula energymeter
#define ENERGYMETER_POWER 1318 ///< complete id for a POWER message for the formula energymeter
#define ENERGYMETER_CURRENT_COUNTER 1319 ///< complete id for a CURRENT_COUNTER message for the formula energymeter
#define ENERGYMETER_ENERGY 1320 ///< complete id for a ENERGY message for the formula energymeter
#define ENERGYMETER_RESTART 63 ///< data value for the RESTART command for the formula energymeter
///@}

///@{
///@name Eclipse specifics
///@brief defines used only by the Umicore Eclipse

#define ECLIPSE_INV_LF_V1_ID  648 ///< id of the CAN-message with the status, velocity and current of the motor FL
#define ECLIPSE_INV_LF_V2_ID 650 ///< id of the CAN-message with the temperature and error values for the motor FL
#define ECLIPSE_I1S1_ID 393 ///< id of the CAN-message sending the setpoints for the motor FL
#define ECLIPSE_INV_RR_V1_ID 647 ///< id of the CAN-message with the status, velocity and current of the motor RR
#define ECLIPSE_INV_RR_V2_ID 649 ///< id of the CAN-message with the temperature and error values for the motor RR
#define ECLIPSE_I3S1_ID 392 ///< id of the CAN-message sending the setpoints for the motor RR
#define ECLIPSE_INV_LR_V1_ID 643 ///< id of the CAN-message with the status, velocity and current of the motor RL
#define ECLIPSE_INV_LR_V2_ID 645 ///< id of the CAN-message with the temperature and error values for the motor  RL
#define ECLIPSE_I2S1_ID 388 ///< id of the CAN-message sending the setpoints for the motor LR
#define ECLIPSE_INV_RF_V1_ID 644 ///< id of the CAN-message with the status, velocity and current of the motor FR
#define ECLIPSE_INV_RF_V2_ID 646 ///< id of the CAN-message with the temperature and error values for the motor FR
#define ECLIPSE_I4S1_ID 389 ///< id of the CAN-message sending the setpoints for the motor FR
#define ECLIPSE_INV1 1 ///< not sure how it's used
#define ECLIPSE_INV2 2 ///< not sure how it's used
#define ECLIPSE_INV3 3 ///< not sure how it's used
#define ECLIPSE_INV4 4 ///< not sure how it's used
#define ECLIPSE_INV_FL 1 ///< not sure how it's used
#define ECLIPSE_INV_RL 2 ///< not sure how it's used
#define ECLIPSE_INV_RR 3 ///< not sure how it's used
#define ECLIPSE_INV_FR 4 ///< not sure how it's used
///@}

///@{
///@name Aurora specifics
///@brief defines used only by the Umicore aurora (more info https://drive.google.com/a/formulaelectric.be/file/d/14blvAJBdCPIJoc66KqUrZvYMPwNpgcf9/view?usp=sharing)

#define AURORA_DRIVE_STAT_INIT 0 ///< drive is still initializing
#define AURORA_DRIVE_STAT_READY 1 ///< drive is ready
#define AURORA_DRIVE_STAT_ERROR 2 ///< drive has an error
#define AURORA_DRIVE_STAT_PRECHARGING 3 ///< drive is precharging high voltage
#define AURORA_DRIVE_STAT_MOTORA_ERR 4 ///< drive has an error in motor A
#define AURORA_DRIVE_STAT_MOTORB_ERR 5 ///< drive has an error in motor B
#define AURORA_SCALE_MOTOR_VEL 0.002196172151 ///< speed (m/s) = speed (bin) * scale
#define AURORA_SCALE_MOTOR_TORQUE 0.2 ///< torque (Nm) = torque (bin) * scale
#define AURORA_SCALE_MOTOR_TEMP 1 ///< temp (°C) = temp (bin) * scale
#define AURORA_SCALE_DRIVE_VOLT 4 ///< voltage (V) = voltage (bin) * scale
#define AURORA_SCALE_DRIVE_POWER 5 ///< Power (W) = power (bin) * scale
#define AURORA_CAN_CTRL_DRIVE 0x18FF1000 ///< turn on/off both drives
#define AURORA_CAN_CTRL_MOTOR_FL 0x18FF1100 ///< control motor FL
#define AURORA_CAN_CTRL_MOTOR_FR 0x18FF1200 ///< control motor FR
#define AURORA_CAN_CTRL_MOTOR_RL 0x18FF2100 ///< control motor RL
#define AURORA_CAN_CTRL_MOTOR_RR 0x18FF2200 ///< control motor RR
#define AURORA_CAN_STAT_DRIVE_F 0x18FF00EA ///< get status of drive F
#define AURORA_CAN_STAT_MOTOR_FL 0x18FF01EA ///< get status of motor FL
#define AURORA_CAN_STAT_MOTOR_FR 0x18FF02EA ///< get status of motor FR
#define AURORA_CAN_STAT_DRIVE_R 0x18FF00EB ///< get status of drive R
#define AURORA_CAN_STAT_MOTOR_RL 0x18FF01EB ///< get status of motor RL
#define AURORA_CAN_STAT_MOTOR_RR 0x18FF02EB ///< get status of motor RR
///@}

///@{
///@name IMU
///@brief defines used to get data from the IMU

#define IMU_CAN_STATUS1 1 ///< id of the can message containing the general status of the IMU
#define IMU_CAN_STATUS3 2 ///< id of the can message containing the status of the IMU sensors
#define IMU_CAN_ACCEL 3 ///< id of the can message containing acceleration data
#define IMU_CAN_GYRO 4 ///< id of the can message containing angular rate data
#define IMU_CAN_EULER 5 ///< id of the can message containing roll, pitch, yaw data
#define IMU_CAN_EULER_ACC 6 ///< id of the can message containing roll, pitch, yaw accuracy
#define IMU_CAN_POS 7 ///< id of the can message containing latitude and longitude data
#define IMU_CAN_POS_ACC 8 ///< id of the can message containing the accuracy of the position
#define IMU_CAN_VEL 9 ///< id of the can message containing abolute (north, east, down) velocities
#define IMU_CAN_VEL_ACC 10 ///< id of the can message containing velocity accuracy
#define IMU_STAT_SOL_FILT 244 ///< expected value if there's no error in the EKF output
#define IMU_STAT_SOL_MASK 255 ///< mask for the solution status filter
#define IMU_STAT_SOL_POS 0 ///< position in CAN_STATUS3 message for the solution status
#define IMU_STAT_GEN_FILT 127 ///< expected value if there's no error in the IMU
#define IMU_STAT_GEN_MASK 255 ///< mask for the general status filter
#define IMU_STAT_GEN_POS 4 ///< position in CAN_STATUS1 message for the general status
#define IMU_ACCEL_X_POS 0 ///< position in the CAN_ACCEL message
#define IMU_ACCEL_Y_POS 2 ///< position in the CAN_ACCEL message
#define IMU_ACCEL_Z_POS 4 ///< position in the CAN_ACCEL message
#define IMU_ACCEL_SCALE 0.01 ///< real value = byte_val * scale (in m/s²)
#define IMU_ANG_RATE_X_POS 0 ///< position in the CAN_GYRO message
#define IMU_ANG_RATE_Y_POS 2 ///< position in the CAN_GYRO message
#define IMU_ANG_RATE_Z_POS 4 ///< position in the CAN_GYRO message
#define IMU_ANG_RATE_SCALE 0.001 ///< real value = byte_val * scale (in rad/s)
#define IMU_ANG_X_POS 0 ///< position in the CAN_EULER message
#define IMU_ANG_Y_POS 2 ///< position in the CAN_EULER message
#define IMU_ANG_Z_POS 4 ///< position in the CAN_EULER message
#define IMU_ANG_X_ACC_POS 0 ///< position in the CAN_EULER_ACC message
#define IMU_ANG_Y_ACC_POS 2 ///< position in the CAN_EULER_ACC message
#define IMU_ANG_Z_ACC_POS 4 ///< position in the CAN_EULER_ACC message
#define IMU_ANG_SCALE 0.0001 ///< real value = byte_val * scale (in rad)
#define IMU_POS_LAT_POS 0 ///< position in the CAN_POS message
#define IMU_POS_LON_POS 4 ///< position in the CAN_POS message
#define IMU_POS_SCALE 0.0000001 ///< real value = byte_val * scale (in °)
#define IMU_POS_LAT_ACC_POS 0 ///< position in the CAN_POS_ACC message
#define IMU_POS_LON_ACC_POS 2 ///< position in the CAN_POS_ACC message
#define IMU_POS_ACC_SCALE 0.01 ///< real value = byte_val * scale (in m)
#define IMU_VEL_N_POS 0 ///< position in the CAN_VEL message
#define IMU_VEL_E_POS 2 ///< position in the CAN_VEL message
#define IMU_VEL_D_POS 4 ///< position in the CAN_VEL message
#define IMU_VEL_N_ACC_POS 0 ///< position in the CAN_VEL_ACC message
#define IMU_VEL_E_ACC_POS 2 ///< position in the CAN_VEL_ACC message
#define IMU_VEL_D_ACC_POS 4 ///< position in the CAN_VEL_ACC message
#define IMU_VEL_SCALE 0.01 ///< real value = byte_val * scale (in m/s)
///@}

///@{
///@name Sensor node configurations
///@brief all settings in the sensor nodes that can be configured

#define SENSOR_NODE_CONFIG_COUNT 48 ///< number of sensor_node settings (can be used for an array)
#define SENSOR_NODE_CONFIG_SAVE 0 ///< save the current settings to EEPROM
#define SENSOR_NODE_CONFIG_ACTUATOR_AMOUNT 1 ///< number of actuators on this node
#define SENSOR_NODE_CONFIG_ACTUATOR1_ID 2 ///< Actuator 1 id
#define SENSOR_NODE_CONFIG_ACTUATOR2_ID 3 ///< Actuator 2 id
#define SENSOR_NODE_CONFIG_ACTUATOR3_ID 4 ///< Actuator 3 id
#define SENSOR_NODE_CONFIG_SENSOR_AMOUNT 5 ///< number of sensor on this node
#define SENSOR_NODE_CONFIG_SENSOR1_FREQUENCY 6 ///< Sensor 1 frequency
#define SENSOR_NODE_CONFIG_SENSOR1_CHANNEL 7 ///< Sensor 1 channel
#define SENSOR_NODE_CONFIG_SENSOR1_HASEVENT 8 ///< Sensor 1 hasevent
#define SENSOR_NODE_CONFIG_SENSOR1_EVENTID 9 ///< Sensor 1 eventid
#define SENSOR_NODE_CONFIG_SENSOR1_ID 10 ///< Sensor 1 id
#define SENSOR_NODE_CONFIG_SENSOR2_FREQUENCY 11 ///< Sensor 2 frequency
#define SENSOR_NODE_CONFIG_SENSOR2_CHANNEL 12 ///< Sensor 2 channel
#define SENSOR_NODE_CONFIG_SENSOR2_HASEVENT 13 ///< Sensor 2 hasevent
#define SENSOR_NODE_CONFIG_SENSOR2_EVENTID 14 ///< Sensor 2 eventid
#define SENSOR_NODE_CONFIG_SENSOR2_ID 15 ///< Sensor 2 id
#define SENSOR_NODE_CONFIG_SENSOR3_FREQUENCY 16 ///< Sensor 3 frequency
#define SENSOR_NODE_CONFIG_SENSOR3_CHANNEL 17 ///< Sensor 3 channel
#define SENSOR_NODE_CONFIG_SENSOR3_HASEVENT 18 ///< Sensor 3 hasevent
#define SENSOR_NODE_CONFIG_SENSOR3_EVENTID 19 ///< Sensor 3 eventid
#define SENSOR_NODE_CONFIG_SENSOR3_ID 20 ///< Sensor 3 id
#define SENSOR_NODE_CONFIG_SENSOR4_FREQUENCY 21 ///< Sensor 4 frequency
#define SENSOR_NODE_CONFIG_SENSOR4_CHANNEL 22 ///< Sensor 4 channel
#define SENSOR_NODE_CONFIG_SENSOR4_HASEVENT 23 ///< Sensor 4 hasevent
#define SENSOR_NODE_CONFIG_SENSOR4_EVENTID 24 ///< Sensor 4 eventid
#define SENSOR_NODE_CONFIG_SENSOR4_ID 25 ///< Sensor 4 id
#define SENSOR_NODE_CONFIG_SENSOR5_FREQUENCY 26 ///< Sensor 5 frequency
#define SENSOR_NODE_CONFIG_SENSOR5_CHANNEL 27 ///< Sensor 5 channel
#define SENSOR_NODE_CONFIG_SENSOR5_HASEVENT 28 ///< Sensor 5 hasevent
#define SENSOR_NODE_CONFIG_SENSOR5_EVENTID 29 ///< Sensor 5 eventid
#define SENSOR_NODE_CONFIG_SENSOR5_ID 30 ///< Sensor 5 id
#define SENSOR_NODE_CONFIG_SENSOR6_FREQUENCY 31 ///< Sensor 6 frequency
#define SENSOR_NODE_CONFIG_SENSOR6_CHANNEL 32 ///< Sensor 6 channel
#define SENSOR_NODE_CONFIG_SENSOR6_HASEVENT 33 ///< Sensor 6 hasevent
#define SENSOR_NODE_CONFIG_SENSOR6_EVENTID 34 ///< Sensor 6 eventid
#define SENSOR_NODE_CONFIG_SENSOR6_ID 35 ///< Sensor 6 id
#define SENSOR_NODE_CONFIG_SENSOR7_FREQUENCY 36 ///< Sensor 7 frequency
#define SENSOR_NODE_CONFIG_SENSOR7_CHANNEL 37 ///< Sensor 7 channel
#define SENSOR_NODE_CONFIG_SENSOR7_HASEVENT 38 ///< Sensor 7 hasevent
#define SENSOR_NODE_CONFIG_SENSOR7_EVENTID 39 ///< Sensor 7 eventid
#define SENSOR_NODE_CONFIG_SENSOR7_ID 40 ///< Sensor 7 id
#define SENSOR_NODE_CONFIG_SENSOR8_FREQUENCY 41 ///< Sensor 8 frequency
#define SENSOR_NODE_CONFIG_SENSOR8_CHANNEL 42 ///< Sensor 8 channel
#define SENSOR_NODE_CONFIG_SENSOR8_HASEVENT 43 ///< Sensor 8 hasevent
#define SENSOR_NODE_CONFIG_SENSOR8_EVENTID 44 ///< Sensor 8 eventid
#define SENSOR_NODE_CONFIG_SENSOR8_ID 45 ///< Sensor 8 id
#define SENSOR_NODE_CONFIG_LOAD_DEFAULTS 46 ///< reload the default (hard coded) settings
#define SENSOR_NODE_CONFIG_NODE_ID 47 ///< the id of the sensor node
///@}

///@{
///@name SAS node configurations
///@brief all settings and enums for configuring the SAS node

#define SAS_NODE_CONFIG_CURRENT_MODE_MA 0 ///< calculate the dutycycle value with the actuator value in desired current (mA)
#define SAS_NODE_CONFIG_CURRENT_MODE_DC 1 ///< put the received actuator value directly into the dutycycle register
#define SAS_NODE_CONFIG_CURRENT_MODE_PID 2 ///< use a PID to calculate the dutycycle. The actuator value is the current in mA
#define SAS_NODE_CONFIG_CURRENT_MODE 0 ///< change the current mode of the SAS node
#define SAS_NODE_CONFIG_KP 1 ///< Change the KP Value of the PID control
#define SAS_NODE_CONFIG_KD 2 ///< Change the KD Value of the PID control
#define SAS_NODE_CONFIG_KI 3 ///< Change the KI Value of the PID control
///@}

///@{
///@name 
///@brief 

#define SCS_EBS_FSM_STARTUP 0 ///< inital state, initialising peripherals
#define SCS_EBS_FSM_READY 1 ///< idle state
#define SCS_EBS_FSM_PREPARE 2 ///< closing safety loop
#define SCS_EBS_FSM_RELEASE 3 ///< releasing the EBS, for manual driving or pushing
#define SCS_EBS_FSM_DRIVE 4 ///< during driverless operations
#define SCS_EBS_FSM_REPORT 5 ///< after driving, without errors
#define SCS_EBS_FSM_FAULT 6 ///< when an error occurs
#define SCS_ASSI_AS_OFF 7 ///< lights off
#define SCS_ASSI_AS_READY 8 ///< yellow continuous
#define SCS_ASSI_AS_DRIVING 9 ///< yellow flashing
#define SCS_ASSI_AS_EMERGENCY 10 ///< blue flashing
#define SCS_ASSI_AS_FINISHED 11 ///< blue continuous
///@}

///@{
///@name CAN function macros
///@brief function that are universal for all nodes in the CAN-system

/// make a CAN-id with the function code and node id
#define make_id(node_id, function_code) ((function_code) << (NODE_ID_LENGTH) | (node_id))
/// retrieve a function code from a (right-justified) CAN-id
#define get_function_code(can_id) (((can_id) >> NODE_ID_LENGTH) & ((1 << FUNCTION_CODE_LENGTH) - 1))
/// retrieve a node id from a (right-justified) CAN-id
#define get_node_id(can_id) ((can_id) & ((1 << NODE_ID_LENGTH) - 1))
///@}

/** @}*///addtogroup can
#endif /* CAN_IDS_H_ */

