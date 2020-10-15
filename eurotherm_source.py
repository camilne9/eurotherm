#!/usr/bin/env python
# -*- coding: utf-8 -*-

import minimalmodbus
import time
import threading
import math

# # For guis
# from silx import sx
# import matplotlib.pyplot as plt
# import numpy
# from silx.gui import qt
# from silx.gui.plot import Plot1D
# qapp = qt.QApplication([])



class eurotherm2408(object):
    def __init__(self, serialPort, slaveAddress=1, baudrate=19200 ):

        # Keep this first in the class

        self._registers = {
            "setpoint1" : 24,
            "setpoint2" : 25,
            "setpoint3" :164,
            "setpoint4" :165,
            "setpoint5" :166,
            "setpoint6" :167,
            "setpoint7" :168,
            "setpoint8" :169,
            "setpoint9" :170,
            "setpoint10":171,
            "setpoint11":172,
            "setpoint12":173,
            "setpoint13":174,
            "setpoint14":175,
            "setpoint15":176,
            "setpoint16":177,
            "selectedSetpoint": 15,
            "Local_or_remote_setpoint_select" : 276,
            "Remote_setpoint": 485,
            "Remote_setpoint_trim":486,
            "Ratio_setpoint":61,
            "Local_setpoint_trim":27,
            "Setpoint_1_low_limit":112,
            "Setpoint_1_high_limit": 111,
            "Setpoint_2_low_limit":114,
            "Setpoint_2_high_limit":113,
            "Local_setpoint_trim_low_limit":67,
            "Local_setpoint_trim_high_limit": 66,
            "Setpoint_rate_limit":35,
            "Holdback_type_for_sp_rate_limit":70,
            "Holdback_value_for_srtpoint_rate_limit":65,
            "Dwell_Segment":62,
            "Goto":517,
            "Programmer_State_Write":57,
            "Programmer_state_Read":23,
            "Gain_scheduler_setpoint":153,
            "Current_PID_set":72,
            "Proportional_band_PID1":6,
            "Integral_time_PID1":8,
            "Derivative_time_PID1":9,
            "Manual_reset_PID1":28,
            "Cutback_high_PID1":18,
            "Cutback_low_PID1":17,
            "Relative_cool_gain_PID1":19,
            "Proportional_band_PID2":48,
            "Integral_time_PID2":49,
            "Derivative_time_PID2":51,
            "Manual_reset_PID2":50,
            "Cutback_high_PID2":118,
            "Cutback_low_PID2":117,
            "Relative_cool_gain_PID2":52,
            "Feedforward_proportional_band":97,
            "Feedforward_trim":98,
            "Feedforward_trim_limit":99,


            "Valve travel_time":21,
            "Valve_inertia_time":123,
            "Valve_backlash_time":124,
            "Minimum_pulse_time":54,
            "Bounded_sensor_break_strategy":128,
            "VP_Bounded_sensor_break":62,

            "Current_program_running__active_prog_no":22,
            "Program_status":23,
            "Programmer_setpoint":163,
            "Program_cycles_remaining":59,
            "Current_segment_number":56,
            "Current_segment_type":29,
            "Segment_time_remaining_in_secs":36,
            "Segment_time_remaining_in_mins":63,
            "Target_setpoint__current_segment":160,
            "Ramp_rate":161,
            "Program_time_remaining":58,
            "Fast_run":57,
            "Logic_1_output__current_program":464,
            "Logic_2_output__current_program":465,
            "Logic_3_output__current_program":466,
            "Logic_4_output__current_program":467,
            "Logic_5_output__current_program":468,
            "Logic_6_output__current_program":469,
            "Logic_7_output__current_program":470,
            "Logic_8_output__current_program":471,
            "A__Segment_synchronisation":488,
            "Flash_active_segment_in_lower_display":284,
            "Advance_Segment_Flag":149,
            "Skip_Segment_Flag":154,
            "Program_Logic_Status":162,
            "BBB_Alarm_1setpoint_value":13,
            "BBB_Alarm_2setpoint_value":14,
            "BBB_Alarm_3setpoint_value":81,
            "BBB_Alarm_4setpoint_value":82,
            "Alarm_1_hysteresis":47,
            "Alarm_2_hysteresis":68,
            "Alarm_3_hysteresis":69,
            "Alarm_4_hysteresis":71,
            "Loop_break_time":83,
            "Enable_diagnostic_messages":282,
            "Acknowledge_All_Alarms":274,
            "Autotune_enable":270,
            "Adaptive_tune_enable":271,
            "Adaptive_tune_trigger_level":100,
            "Automatic_droop_compensation__manual_reset":272,
            "Process_Variable":1,
            "Target_setpoint":2,
            "pc_Output_power":3,
            "Working_set_point":5,
            "Auto_man_select":273,
            "Pot_Position":317,
            "Valve_Posn__computed_by_VP_algorithm":53,
            "VP_Manual_Output__alterable_in_Man_only":60,
            "Heater_current__With_PDSIO_mode_2":80,
            "Customer_defined_identification_number":629,
            "Setpoint_Span":552,
            "Error__PV_SP":39,
            "Remote_Input_Value":26,
            "Input_1_filter_time_constant":101,
            "Input_2_filter_time_constant":103,
            "Select_input_1_or_input_2":288,
            "Derived_input_function_factor_1":292,
            "_Derived_input_function_factor_2":293,
            "Switchover_transition_region_high":286,
            "Switchover_transition_region_low":287,
            "Potentiometer_Calibration_Enable":310,
            "Potentiometer_Input_Calibration_Node":311,
            "Potentiometer_Calibration_Go":312,
            "Emmisivity":38,
            "Emmisivity_input_2":104,
            "User_calibration_enable":110,
            "Selected_calibration_point":102,
            "User_calibration_adjust_input_1":146,
            "User_calibration_adjust_input_2":148,
            "Input_1_calibration_offset":141,
            "Input_2_calibration_offset":142,
            "Input_1_measured_value":202,
            "Input_2_measured_value":208,
            "Input_1_cold_junction_temp__reading":215,
            "Input_2_cold_junction_temp__reading":216,
            "Input_1_linearised_value":289,
            "Input_2_linearised_value":290,
            "Currently_selected_setpoint":291,
            "Low_power_limit":31,
            "High_power_limit":30,
            "Remote_low_power_limit":33,
            "Remote_high_power_limit":32,
            "Output_rate_limit":37,
            "Forced_output_level":84,
            "Heat_cycle_time":10,
            "Heat_hysteresis__on_off_output":86,
            "Heat_output_minimum_on_time":45,
            "Cool_cycle_time":20,
            "Cool_hysteresis__on_off_output":88,
            "Cool_output_minimum_on_time":89,
            "Heat_cool_deadband__on_off_op":16,
            "Power_in_end_segment":64,
            "Sensor_break_output_power":34,
            "On_Off_Sensor_Break_Output":40,

            "Configuration_of_lower_readout_display":106,
            "PV_minimum":134,
            "PV_maximum":133,
            "PV_mean_value":135,
            "Time_PV_above_threshold_level":139,
            "PV_threshold_for_timer_log":138,
            "Logging_reset":140,
            "Maximum_Control_Task_Time__Processor_utilisation_factor":201,
            "Working_output":4,
            "PDSIO_SSR_status":79,
            "Feedforward_component_of_output":209,
            "Proportional_component_of_output":214,
            "Integral_component_of_output":55,
            "Derivative_component_of_output":116,
            "VP_motor_calibration_state":210,

            "DC_Output_1A_Telemetry":12694,
           "DC_Output_2A_Telemetry":12758,
            "DC_Output_3A_Telemetry":12822,
            "BCD_Input_Value":96,
            "Instrument_Mode":199,
            "Instrument_Version_Number":107,
            "Instrument_Ident":122,
            "Slave_Instrument_Target_Setpoint":92,
            "Slave_Instrument_Ramp_Rate":93,
            "Slave_Instrument_Sync":94,
            "Remote_SRL_Hold":95,
            "CNOMO_Manufacturers_ID":121,
            "Remote_Parameter":151,
            "Error_Logged_Flag":73,
            "Ramp_Rate_Disable":78,
            "Maximum_Input_Value":548,
            "Minimum_Input_Value":549,
            "Holdback_Disable":278,
            "All_User_Interface_Keys_Disable":279,

            "Control_type":512,
            "Control_Action":7,
            "Type_of_cooling":524,
            "Integral_and_Derivative_time_units":529,
            "Derivative_action":550,
            "Front_panel_Auto_Manual_button":530,
            "Front_panel_Run_Hold_button":564,
            "Power_feedback_enable":565,
            "Feed_forward_type":532,
            "Manual_Auto_transfer_PD_control":555,
            "_5D6_Sensor_break_output":553,
            "Forced_manual_output":556,
            "BCD_input_function":522,
            "Gain_schedule_enable":567,
            "Custom_linearisation_input_1":601,
            "Display_value_corresponding_to_input_1":621,
            "Custom_linearisation_input_2":602,
            "Display_value_corresponding_to_input_2":622,
            "Custom_linearisation_input_3":603,
            "Display_value_corresponding_to_input_3":623,
            "Custom_linearisation_input_4":604,
            "Display_value_corresponding_to_input_4":624,
            "Custom_linearisation_input_5":605,
            "Display_value_corresponding_to_input_5":625,
            "Custom_linearisation_input_6":606,
            "Display_value_corresponding_to_input_6":626,
            "Custom_linearisation_input_7":607,
            "Display_value_corresponding_to_input_7":627,
            "Custom_linearisation_input_8":608,
            "Display_value_corresponding_to_input_8":628,
            "Instrument_units":516,
            "Decimal_places_in_displayed_value":525,
            "Setpoint_Min___Low_range_limit":11,
            "Setpoint_Max___High_range_limit":12,

            "Input_type":12290,
            "Cold_junction_compensation":12291,
            "Sensor_break_impedance":12301,
            "Input_value_low":12307,
            "Input_value_high":12306,
            "Displayed_reading_low":12303,
            "Displayed_reading_high":12302,
            "Number_of_setpoints":521,
            "Remote_tracking":526,
            "Manual_tracking":527,
            "Programmer_tracking":528,
            "Setpoint_rate_limit_units":531,
            "Remote_setpoint_configuration":535,

            "Alarm_1_type":536,
            "Alarm_1_Latching":540,
            "Alarm_1_Blocking":544,

            "Alarm_2_type":537,
            "Alarm_2_Latching":541,
            "Alarm_2_Blocking":545,
            "Alarm_3_type":538,
            "Alarm_3_Latching":542,
            "Alarm_3_Blocking":546,
            "Alarm_4_type":539,
            "Alarm_4_Latching":543,
            "Alarm_4_Blocking":547,

            "Programmer_type":517,
            "Holdback":559,
            "Power_fail_recovery":518,
            "Servo":520,
            "Programmable_event_outputs":558,
            "Synchronisation_of_programs":557,
            "Maximum_Number_Of_Segments":211,

            "DI1_Logic":12352,
            "DI1_Input_functions":12355,

            "DI2_Identity:":12416,
            "DI2_Input_functions":12419,
            "DI2_Low_scalar":12431,
            "DI2_High_scalar":12430,

            "AA_Module_identity":12480,
            "AA_Module_function":12483,
            "AA_Sense_of_output":12489,
            "AA_Summary_of_AA_configuration":12486,
            "AA_Program_summary_OP_AA_configuration":12503,
            "AA_Comms_Resolution":12550,
            "AA_Module_Identity":12608,
            "AA_Retransmitted_Low_Scalar":12623,
            "AA_Retransmitted_High_Scalar":12622,

            "_1A_Module_identity":12672,
            "_1A_Module_function":12675,
            "_1A_PID_or_Retran_value_giving_min__o_p":12687,
            "_1A_PID_or_Retran_value_giving_max__o_p":12686,
            "_1A_Units":12684,
            "_1A_Minimum_electrical_output":12689,
            "_1A_Maximum_electrical_output":12688,
            "_1A_Sense_of_output":12681,
            "_1A_Summary_output_1A_configuration":12678,
            "_1A_DC_output_1A_telemetry_parameter":12694,
            "_1A_Program_summary_output_1A_config":12695,

            "_1B_Module_1B_identity":12673,
            "_1B_Module_1B_function":12676,
            "_1B_Sense_of_output__nor_inv_as1A":12682,
            "_1B_Summary_of_1B_configuration":12679,
            "_1B_Summary_program_O_P_1B_config":12696,

            "_1C_Module_1C_identity":12674,
            "_1C_Module_1C_function":12677,
            "_1C_Module_1C_value_giving_min_output":12699,
            "_1C_Module_1C_value_giving_max_output":12698,
            "_1C_Module_1C_Minimum_electrical_output":12701,
            "_1C_Module_1C_Maximum_electrical":12700,
            "_1C_Sense_of_output__nor_inv_as_1A":12683,
            "_1C_Summary_of_1C_configuration":12680,
            "_1C_Summary_program_O_P_1C_config":12697,

            "_2A_Module_identity":12736,
            "_2A_Module_function":12739,
            "_2A_PID_or_Retran_low_value":12751,
            "_2A_Potentiometer_input_low_scalar":12763,
            "_2A_PID_or_Retran_high_value":12750,
            "_2A_Potentiometer_input_high_scalar":12762,
            "_2A_Units":12748,
            "_2A_Minimum_electrical_output":12753,
            "_2A_Maximum_electrical_output":12752,
            "_2A_Sense_of_output":12745,
            "_2A_Summary_output_2A_configuration":12742,
            "_2A_Program_summary_output_2A_conf":12759,

            "_2B_Module_2B_identity":12737,
            "_2B_Module_2B_function":12740,
            "_2B_Sense_of_output__nor_inv_as_2A":12746,
            "_2B_Summary_of_2B_configuration":12743,
            "_2B_Summary_program_O_P_2B_config":12760,

            "_2C_Module_2C_identity":12738,
            "_2C_Module_2C_function":12741,
            "_2C_Sense_of_output__nor_inv_as_2A":12747,
            "_2C_Summary_of_2C_configuration":12744,
            "_2C_Summary_program_O_P_2C_config":12761,

            "_3A_Module_identity":12800,
            "_3A_Module_function":12803,
            "_3A_input_type__input_2":12830,
            "_3A_Cold_junction_compensation__ip_2":12831,
            "_3A_Sensor_break_impedance__input_2":12813,
            "_3A_Input_value_low":12819,
            "_3A_Input_value_high":12818,
            "_3A_Input_module_3A_low_value":12829,
            "_3A_Input_module_3A_high_value":12828,
            "_3A_Module_3A_low_value":12815,
            "_3A_Potentiometer_input_3A_low_scalar":12827,
            "_3A_Module_3A_high_value":12814,
            "_3A_Potentiometer_input_3A_high_scalar":12826,
            "_3A_Units_3A":12812,
            "_3A_Minimum_electrical_output":12817,
            "_3A_Maximum_electrical_output":12816,
            "_3A_Sense_of_output":12809,
            "_3A_Summary_output_3A_configuration":12806,
            "_3A_Program_summary_output_3A_config":12823,

            "_3B_Module_3B_identity":12801,
            "_3B_Module_3B_function":12804,
            "_3B_Sense_of_output__nor_inv_as_3A":12810,
            "_3B_Summary_of_3B_configuration":12807,
            "_3B_Summary_program_O_P_3B_config":12824,

            "_3C_Module_3C_identity":12802,
            "_3C_Module_3C_function":12805,
            "_3C_Sense_of_output__nor_inv_as_3A":12811,
            "_3C_Summary_of_3C_configuration":12808,
            "_3C_Summary_program_O_P_3C_config":12825,

            "_4A_Module_identity":12864,
            "_4A_Module_function":12867,
            "_4A_Input_module_4A_low_value":12879,
            "_4A_Input_module_4A_high_value":12878,
            "_4A_Minimum_electrical_output":12881,
            "_4A_Maximum_electrical_output":12880,
            "_4A_Sense_of_output__nor_inv_as_3A":12873,
            "_4A_Summary_output_4A_configuration":12870,
            "_4A_Program_summary_output_4A_config":12887,
            "Access_Mode_Password":514,
            "Configuration_Level_Password":515,
            }

        self._register_to_be_read_as_float = {
            "Process_Variable":1,
            "Target_setpoint":2,
            "pc_Output_power":3,
            "Working_set_point":5,
            "Ramp_rate":161,
            "Proportional_component_of_output":0,
            "Integral_component_of_output":0,
            "Derivative_component_of_output":0,
            "Input_1_measured_value":0,
#            "Output_rate_limit":37
#            "_1A_Minimum_electrical_output":12689,
#            "_1A_Maximum_electrical_output":12688,
        }


        self.myMutex = threading.Lock()



        self.debugPrint = False

        self.serialPort=serialPort
        self.slaveAddress = slaveAddress
        self.baudrate = baudrate
        self.instrument = None
        self.maxRetryReadRegister = 10
        self.reconnect()

    def reconnect(self):
        if self.instrument != None:
            self.instrument.serial.close()
            time.sleep(1)
            self.instrument.serial.open()

        self.instrument = minimalmodbus.Instrument(self.serialPort , self.slaveAddress)
        self.instrument.serial.baudrate = self.baudrate

        #See manual "FLOATING POINT DATA FORMATS"
        # http://wikiserv.esrf.fr/sample_env/index.php/Eurotherm_2408#The_decimal_place_is_wrong_.28x10_or_.2F10.29
        # Keep the following line, as calling self.resolution will set the value of sself.floatingPointDataFormat crucial in decoding the values.
        # print("Guessing equipment to be {reso} resolution format.".format(reso=self.resolution))
        self.floatingPointDataFormat = None
        self._resolutionUpdate_()


    #    def __dir__(self):
    #        return super().__dir__()+self._registers.keys()

    def _resolutionUpdate_(self):
        if self.debugPrint: print("Guessing of the floating point data format")
        self.floatingPointDataFormat= self.instrument.read_register(self._registers["AA_Comms_Resolution"], 0)



    def __getattr__(self, name):
        try:
            if name in self._registers:
                val = self._readRegister(self._registers[name])
                if self.debugPrint : print("reading {val} from device at register {reg} ".format(val=str(val), reg = str(self._registers[name])))
                return val
            else:
                return object.__getattribute__(self, name)
        except KeyError:
            raise AttributeError("Attribute inexisting during getting {name} ".format(name=name))
        except Exception as e :
            if self.debugPrint: print(str(e))

    def __setattr__(self, name, value):
        notDone = True
        while notDone:
            try:
                dict.__setattr__(self, name, value)
                if(name in self._registers):
                    if self.debugPrint : print("writing {val} to device at register {reg} ".format(val=str(value), reg = str(self._registers[name])))
                    self._writeRegister(self._registers[name], value)
                    dict.__delattr__(self, name)
                else:
                    object.__setattr__(self, name, value)
                notDone = False

            except KeyError:
                 raise AttributeError("Attribute inexisting during setting {name}, could be a read only attribute ".format(name=name))
                 time.sleep(0.2)
            except Exception as e :
                if self.debugPrint: print(str(e))
                time.sleep(0.2)
                #self.__delattr__(name)


    def _readRegister(self, register):
        if self.floatingPointDataFormat == None:
            self._resolutionUpdate_()

        res = None
        trial = 0
        while trial<self.maxRetryReadRegister:
            try:
                if( register in self._register_to_be_read_as_float.values()):
                    register32bits = int(str((2*register+8000)),16)
                    self.myMutex.acquire()
                    res = self.instrument.read_float(register32bits)
                    self.myMutex.release()

                else:
                    self.myMutex.acquire()
                    res = self.instrument.read_register(register, self.floatingPointDataFormat)
                    self.myMutex.release()

            except Exception as e:
                if self.debugPrint : print("Exception while reading register "+str(e))
                self.myMutex.release()
                self.reconnect()

                # if type(res) == float:
                #     res = res / (math.pow(10,self.decimalsStored))

            if res  != None : return res
            trial+=1
        return None

    def _writeRegister(self, register, value):

        if self.floatingPointDataFormat == None:
            self._resolutionUpdate_()

        # if type(value) == float:
        #     value = value *  (math.pow(10,self.decimalsStored))

        if type(value) != int and type(value) != float:
            value = float(value)
            if self.debugPrint : print("Value forced to be a float (was neither int or float) : "+str(value))

        if( register in self._register_to_be_read_as_float.values()):
            floating = self.floatingPointDataFormat
            try:
                register32bits = int(str((2*register+8000)),16)
                self.myMutex.acquire()
                self.instrument.write_float(register32bits, value)
                self.myMutex.release()
            except Exception as e:
                if self.debugPrint : print("Exception while writing register {reg} with {val} with a decimals of {dec}  \n Exception: {exc}".format(exc=str(e), reg=register, val=value, dec=floating))
                self.myMutex.release()
                self.reconnect()

        else:
            floating = self.floatingPointDataFormat

            try:
                self.myMutex.acquire()
                self.instrument.write_register(register, value, floating)
                self.myMutex.release()
            except Exception as e:
                if self.debugPrint : print("Exception while writing register {reg} with {val} with a decimals of {dec}  \n Exception: {exc}".format(exc=str(e), reg=register, val=value, dec=floating))
                self.myMutex.release()
                self.reconnect()


    def dumpAll(self):
        res = {}
        for key in self._registers.keys():
            res[key] = self.__getattr__(key)
        return res






    #  All class attribute are already available, but for commodity, I'm exposing here some of the most used parameters.
    # All the other attributes can directly be called (read and written ) by instance.param




    @property
    def temperature(self):
        return self.Process_Variable

    @property
    def remoteSetpoint(self):
        if self.Local_or_remote_setpoint_select == 1.0:
            return True
        else:
            return False

    @property
    def setpoint(self):
        return self.Target_setpoint

    @setpoint.setter
    def setpoint(self, value):

        if self.Setpoint_Max___High_range_limit < value:
            #self.Instrument_Mode = 2
            print ("You need to set the Setpoint_Max___High_range_limit Higher in conf mode" )
            self.Setpoint_Max___High_range_limit = value

        if self.Setpoint_1_high_limit < value:
            print("Forcing High limit")
            self.Setpoint_1_high_limit = value


        self.Target_setpoint=float(value)

    @property
    def P(self):
        return self.Proportional_band_PID1

    @P.setter
    def P(self, value):
        self.Proportional_band_PID1=value

    @property
    def I(self):
        return self.Integral_time_PID1

    @I.setter
    def I(self, value):
        self.Integral_time_PID1=value


    @property
    def D(self):
        return self.Derivative_time_PID1

    @D.setter
    def D(self, value):
        self.Derivative_time_PID1=value




    @property
    def P2(self):
        return self.Proportional_band_PID2

    @P2.setter
    def P2(self, value):
        self.Proportional_band_PID2=value

    @property
    def I2(self):
        return self.Integral_time_PID2

    @I2.setter
    def I2(self, value):
        self.Integral_time_PID2=value


    @property
    def D2(self):
        return self.Derivative_time_PID2

    @D2.setter
    def D2(self, value):
        self.Derivative_time_PID2=value



    @property
    def power(self):
        return self.pc_Output_power
    @power.setter
    def power(self, value):
        self.pc_Output_power = value


    @property
    def workingSetpoint(self):
        return self.Working_set_point

    @property
    def manual(self):
        if self.Auto_man_select == 1:
            return True
        else:
            return False

    @manual.setter
    def manual(self, value):
        if value:
            self.Auto_man_select = 1
        else:
            self.Auto_man_select = 0

    @property
    def pid(self):
        return (self.P, self.I, self.D)

    @pid.setter
    def pid(self,pid):
        (self.P, self.I, self.D) = pid

    @property
    def pid2(self):
        return (self.P2, self.I2, self.D2)

    @pid2.setter
    def pid2(self,pid2):
        (self.P2, self.I2, self.D2) = pid2

    @property
    def cutbackHigh(self):
        return self.Cutback_high_PID1

    @cutbackHigh.setter
    def cutbackHigh(self,value):
        self.Cutback_high_PID1 = value

    @property
    def cutbackLow(self):
        return self.Cutback_low_PID1

    @cutbackLow.setter
    def cutbackLow(self,value):
        self.Cutback_low_PID1 = value

    @property
    def automatic(self):
        return not self.manual

    @automatic.setter
    def automatic(self,value):
        self.manual = not value

    @property
    def rampRate(self):
        #return self.Ramp_rate
        return self.Setpoint_rate_limit

    @rampRate.setter
    def rampRate(self, value):
        if self.Ramp_Rate_Disable == 1: print("Warning, ramp rate is disabled. myEuro.Ramp_Rate_Disable = 0 to reactivate.")
        #if self.Setpoint_rate_limit < value : print("Warning, Setpoint_rate_limit is lower than desired ramp rate.")
        self.Setpoint_rate_limit = value
        self.Ramp_rate = value   # do not do anything


    @property
    def resolution(self):
        if self.AA_Comms_Resolution == 0:
            self.floatingPointDataFormat = 0
            return "Full"
        else:
            self.floatingPointDataFormat = 1
            return "Integer"

    @property
    def temperatureSensor(self):
        type=self.Input_type
        if type == 0: return "J"
        if type == 1: return "K"
        if type == 2: return "L"
        if type == 3: return "R"
        if type == 4: return "B"
        if type == 5: return "N"
        if type == 6: return "T"
        if type == 7: return "S"
        if type == 8: return "PL 2"
        if type == 9: return "Custom (factory)"
        if type == 10: return "RTD"
        if type == 11: return "Linear mV (+/- 100mV)"
        if type == 12: return "Linear V (0-10V)"
        if type == 13: return "Linear mA"
        if type == 14: return "Square root V"
        if type == 15: return "Square root mA"
        if type == 16: return "Custom mV "
        if type == 17: return "Custom V"
        if type == 18: return "Custom mA"

#5.7 CONFIGURATION MODE PARAMETERS
#To write parameters in this group, it is first necessary to set the instrument mode parameter (Bisynch ‘IM’,
#Modbus 199) to the value 2 to set the controller into configuration mode. Note this will disable all normal
#control action and the controller outputs will be switched to a safe state.
#To exit from configuration mode, simply write 0 to instrument mode. This will reset the controller, a process that
#takes around 5 seconds. During this period it will not be possible to communicate with the controller.

    @temperatureSensor.setter
    def temperatureSensor(self, value):

        if(value != self.temperatureSensor):
            self.Instrument_Mode = 2

            if value ==  "J"                        : self.Input_type =    0
            if value ==  "K"                        : self.Input_type =    1
            if value ==  "L"                        : self.Input_type =    2
            if value ==  "R"                        : self.Input_type =    3
            if value ==  "B"                        : self.Input_type =    4
            if value ==  "N"                        : self.Input_type =    5
            if value ==  "T"                        : self.Input_type =    6
            if value ==  "S"                        : self.Input_type =    7
            if value ==  "PL 2"                     : self.Input_type =    8
            if value ==  "Custom (factory)"         : self.Input_type =    9
            if value ==   "RTD"                     : self.Input_type =    10
            if value ==   "Pt100"                   : self.Input_type =    10
            if value ==   "PT100"                   : self.Input_type =    10
            if value ==   "pt100"                   : self.Input_type =    10
            if value ==   "Linear mV (+/- 100mV)"   : self.Input_type =    11
            if value ==   "Linear V (0-10V)"        : self.Input_type =    12
            if value ==   "0-10V"                   : self.Input_type =    12
            if value ==   "Linear mA"               : self.Input_type =    13
            if value ==   "Square root V"           : self.Input_type =    14
            if value ==   "Square root mA"          : self.Input_type =    15
            if value ==   "Custom mV "              : self.Input_type =    16
            if value ==   "Custom V"                : self.Input_type =    17
            if value ==   "Custom mA"               : self.Input_type =    18

            #time.sleep(1)
            self.Instrument_Mode = 0
            time.sleep(5)
            self.reconnect()



    @property
    def rampUnit(self):
        units = self.Setpoint_rate_limit_units
        if units == 0 : return "Seconds"
        if units == 1 : return "Minutes"
        if units == 2 : return "Hours"


    @rampUnit.setter
    def rampUnit(self, value):

        if value == "Seconds": value = 0
        if value == "Minutes": value = 1
        if value == "Hours": value = 2

        if value == "sec": value = 0
        if value == "min": value = 1
        if value == "hr": value = 2

        if(value != self.Setpoint_rate_limit_units):
            self.Instrument_Mode = 2
            self.Setpoint_rate_limit_units = value
            self.Instrument_Mode = 0
            time.sleep(5)
            self.reconnect()

    @property
    def timeUnits(self):
        units = self.Integral_and_Derivative_time_units
        if units == 0 : return "Seconds"
        if units == 1 : return "Minutes"
        if units == 2 : return "Hours"



    @timeUnits.setter
    def timeUnits(self, value):

        if value == "Seconds": value = 0
        if value == "Minutes": value = 1
        if value == "Hours": value = 2

        if value == "sec": value = 0
        if value == "min": value = 1
        if value == "hr": value = 2

        if(value != self.Integral_and_Derivative_time_units):
            self.Instrument_Mode = 2
            self.Integral_and_Derivative_time_units = value
            self.Instrument_Mode = 0
            time.sleep(5)
            self.reconnect()

    @property
    def tensionRange(self):
        return (self._1A_Minimum_electrical_output, self._1A_Maximum_electrical_output)

    @tensionRange.setter
    def tensionRange(self, values):
        print("Previous: "+str((self._1A_Minimum_electrical_output, self._1A_Maximum_electrical_output)))
        self.Instrument_Mode = 2
        self._1A_Minimum_electrical_output=values[0]
        self._1A_Maximum_electrical_output = values[1]
        self.Instrument_Mode = 0
        time.sleep(5)
        self.reconnect()
        print("Actual: "+str((self._1A_Minimum_electrical_output, self._1A_Maximum_electrical_output)))


    @property
    def decimalsDisp(self):
        return self.Decimal_places_in_displayed_value

    @decimalsDisp.setter
    def decimalsDisp(self,value):
            self.Instrument_Mode = 2
            self.Decimal_places_in_displayed_value=value
            self.Instrument_Mode = 0
            time.sleep(5)
            self.reconnect()


    # @property
    # def highResolution(self):
    #     if self.AA_Comms_Resolution == 0:
    #         return False
    #     else:
    #         return True
    # @highResolution.setter
    # def highResolution(self,value):
    #     if self.highResolution != value:
    #             self.Instrument_Mode = 2
    #             if value:
    #                 self.AA_Comms_Resolution = 1
    #             else:
    #                 self.AA_Comms_Resolution = 0
    #             self.Instrument_Mode = 0
    #             time.sleep(5)
    #             self.reconnect()


#     def gui(self, updateTime=0.1, lightmemory=False):
#         from silx.gui import qt
#         from silx.gui.plot import Plot1D

#         global app
#         app = qt.QApplication([])

#         time.sleep(1)
#         global mySilxEurotherm
#         self.silx = silxEurotherm(self,lightmemory=lightmemory)
#         time.sleep(1)

#         # Create the thread that calls ThreadSafePlot1D.addCurveThreadSafe
#         global updateThread
#         self.updateThread = UpdateThread(self.silx, updateTime,lightmemory=lightmemory )
#         time.sleep(1)
#         # open silx window
#         self.silx.show()
#         time.sleep(1)
#         self.updateThread.start()  # Start updating the plot
#         time.sleep(1)
#         self.updateThread.pidPlot()

#     def refreshGui(self):
#         self.updateThread.reset()


# class silxEurotherm(Plot1D):

#     _sigAddCurve = qt.Signal(tuple, dict)
#     """Signal used to perform addCurve in the main thread.

#     It takes args and kwargs as arguments.
#     """

#     def __init__(self, myEuro, myEuro2=None, parent=None, lightmemory=False):

#         super(silxEurotherm, self).__init__(parent)
#         # Connect the signal to the method actually calling addCurve
#         self._sigAddCurve.connect(self.__addCurve)

#         self.lightmemory=lightmemory
#         self.myEuro = myEuro
#         self.myEuro2 = myEuro2

#         self.mutex = threading.Lock()

#     def __addCurve(self, args, kwargs):
#         """Private method calling addCurve from _sigAddCurve"""
#         self.addCurve(*args, **kwargs)

#     def addCurveThreadSafe(self, *args, **kwargs):
#         """Thread-safe version of :meth:`silx.gui.plot.Plot.addCurve`

#         This method takes the same arguments as Plot.addCurve.

#         WARNING: This method does not return a value as opposed to Plot.addCurve
#         """
#         self._sigAddCurve.emit(args, kwargs)

# class UpdateThread(threading.Thread):
#     """Thread updating the curve of a :class:`keithley_2510_temperature`

#     :param plot1d: The ThreadSafePlot1D to update."""

#     def __init__(self, plot1d, updateTime=1,lightmemory=False):
#         self.plot1d = plot1d
#         self.running = False
#         self.updateTime = updateTime
#         self.debugPrint = False
#         self.lightmemory=lightmemory
#         super(UpdateThread, self).__init__()

#     def start(self):
#         """Start the update thread"""
#         self.running = True
#         super(UpdateThread, self).start()

#     def run(self):
#         """Method implementing thread loop that updates the plot"""
#         startingTime = time.time()

#         self.history = []
#         self.temperatures = []
#         self.targetSP = []
#         self.power = []

#         if not self.lightmemory:
#             self.setpoints = []
#             self.powerP = []
#             self.powerI = []
#             self.powerD = []

#         self.resetLock = threading.Lock()
#         previousPid = self.plot1d.myEuro.pid
#         previousPid2 = self.plot1d.myEuro.pid2
#         previousRamp = self.plot1d.myEuro.rampRate
#         every = 30

#         while self.running:
#             time.sleep(self.updateTime)
#             with self.resetLock :
#                 try:
#                     temp = float(self.plot1d.myEuro.temperature)

#                     curset = float(self.plot1d.myEuro.workingSetpoint)
#                     power = float(self.plot1d.myEuro.power)

#                     if not self.lightmemory:
#                         setp = float(self.plot1d.myEuro.setpoint)

#                         powerp = float(self.plot1d.myEuro.Proportional_component_of_output)
#                         poweri = float(self.plot1d.myEuro.Integral_component_of_output)
#                         powerd = float(self.plot1d.myEuro.Derivative_component_of_output)

#                         if powerp > power : powerp=power; #print("Silly P")
#                         if poweri > power : poweri=power; #print("Silly I")
#                         if powerd > power : powerd=power; #print("Silly D")

#                     if(every <= 0):
#                         every = 30
#                         tmp1 = self.plot1d.myEuro.pid
#                         tmp3 = self.plot1d.myEuro.pid2
#                         tmp2 = self.plot1d.myEuro.rampRate
#                         if tmp1 != previousPid:
#                             self.pidPlot("pid={pid}".format(pid=str(tmp1)))
#                             previousPid = tmp1
#                         if tmp3 != previousPid2:
#                             self.pidPlot("pid2={pid}".format(pid=str(tmp3)))
#                             previousPid2 = tmp3
#                         if tmp2 != previousRamp:
#                             self.pidPlot("rampRate={ramp}".format(ramp=str(tmp2)))
#                             previousRamp = tmp2
#                     else:
#                         every -=1

#                     if self.lightmemory and type(temp) == float and type(power) == float and type(curset) == float:


#                         self.history.append( time.time()-startingTime )

#                         self.temperatures.append(temp)

#                         self.targetSP.append(curset)
#                         self.power.append(power)


#                         if (len(self.history) == len(self.temperatures) ):

#                             self.plot1d.addCurveThreadSafe(
#                                 x=numpy.asarray(self.history), y=numpy.asarray(self.temperatures), resetzoom=True, legend="Temperature", linewidth=4, yaxis="left")
#                             self.plot1d.addCurveThreadSafe(
#                                 x=numpy.asarray(self.history), y=numpy.asarray(self.targetSP), resetzoom=True, legend="Target", linewidth=1.5, linestyle=":", yaxis="left")
#                             self.plot1d.addCurveThreadSafe(
#                                 x=numpy.asarray(self.history), y=numpy.asarray(self.power), resetzoom=True, legend="Power", linewidth=3, yaxis="right")

#                     else:

#                         if(type(temp) == float and type(setp) == float and type(curset) == float and type(power) == float):


#                             self.history.append( time.time()-startingTime )
#                             self.temperatures.append(temp)
#                             self.setpoints.append(setp)
#                             self.targetSP.append(curset)
#                             self.power.append(power)

#                             self.powerP.append(powerp)
#                             self.powerI.append(poweri)
#                             self.powerD.append(powerd)
#                         #self.tensions.append(self.plot1d.tension)
#                     #self.currents.append(self.plot1d.current)

#                     #self.plot1d.addCurveThreadSafe(
#                     #    numpy.arange(1000), numpy.random.random(1000), resetzoom=False)



#                         if (len(self.history) == len(self.temperatures) ):
#                             self.plot1d.addCurveThreadSafe(
#                                 x=numpy.asarray(self.history), y=numpy.asarray(self.temperatures), resetzoom=True, legend="Temperature", linewidth=4, yaxis="left")
#                             self.plot1d.addCurveThreadSafe(
#                                 x=numpy.asarray(self.history), y=numpy.asarray(self.setpoints), resetzoom=True, legend="Setpoint", linestyle="--",linewidth=4, yaxis="left")
#                             self.plot1d.addCurveThreadSafe(
#                                 x=numpy.asarray(self.history), y=numpy.asarray(self.targetSP), resetzoom=True, legend="Target", linewidth=1.5, linestyle=":", yaxis="left")
#                             self.plot1d.addCurveThreadSafe(
#                                 x=numpy.asarray(self.history), y=numpy.asarray(self.power), resetzoom=True, legend="Power", linewidth=3, yaxis="right")
#                             self.plot1d.addCurveThreadSafe(
#                                 x=numpy.asarray(self.history), y=numpy.asarray(self.powerP), resetzoom=True, legend="Power P", yaxis="right", symbol="+")
#                             self.plot1d.addCurveThreadSafe(
#                                 x=numpy.asarray(self.history), y=numpy.asarray(self.powerI), resetzoom=True, legend="Power I", yaxis="right", symbol="x")
#                             self.plot1d.addCurveThreadSafe(
#                                 x=numpy.asarray(self.history), y=numpy.asarray(self.powerD), resetzoom=True, legend="Power D", yaxis="right", symbol="d")
#                             # self.plot1d.addCurveThreadSafe(
#                             #     x=numpy.asarray(self.history), y=numpy.asarray(self.currents), resetzoom=True, legend="Peltier Current", yaxis="right")

#                         else:
#                             if self.debugPrint : print("Error on length", len(self.history), len(self.temperatures))

#                 except Exception as e:
#                     if self.debugPrint: print(str(e))

#     def stop(self):
#         """Stop the update thread"""
#         self.running = False
#         self.join(2)

#     def reset(self):
#         with self.resetLock :
#             self.history = []
#             self.temperatures = []
#             if not self.lightmemory:
#                 self.setpoints = []
#                 self.targetSP = []
#                 self.power = []
#                 self.powerP = []
#                 self.powerI = []
#                 self.powerD = []




#     def pidPlot(self, text=None):

#         try:
#             if(text==None):text=str(self.plot1d.myEuro.pid)
#             #with self.resetLock :
#             self.plot1d.addMarker(
#                 x=self.history[-1],
#                 y=self.temperatures[-1],
#                 text=text,
#                 draggable=True
#                 )
#         except Exception as e:
#             print("In pidPlot"+str(e))















import argparse
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Use eurotherm on a modbus serial line.')

    parser.add_argument('serial_line', help='Which serial line is the eurotherm connected on, something like /dev/tty? /dev/ttyRP2 ...')


    args = parser.parse_args()

    if not args.serial_line:
        args.serial_line = "/dev/ttyAMA0"

    myEuro = eurotherm2408(args.serial_line,baudrate=9600)

    # Minimal modbus debug mode :
    myEuro.instrument.debug = False


    # Check serial line param : stty -F /dev/ttyAMA0 -a
    # reset default param: stty -F /dev/ttyAMA0 sane; stty -F /dev/ttyAMA0 -echo -echoe -echok

    ## If max setpoint (like 100):
    # value = 999;
    # myEuro.Instrument_Mode = 2;
    # myEuro.Setpoint_Max___High_range_limit = value;
    # myEuro.Setpoint_1_high_limit = value
    # myEuro.Instrument_Mode = 0

    # find a value in the eurotherm
    # dumped = myEuro.dumpAll()
    # {k:v for k,v in dumped.iteritems() if v==100}


    # Difference entre deux dumped:
    #{k:v for k,v in dumped.iteritems() if v!=dumped2[k]}
