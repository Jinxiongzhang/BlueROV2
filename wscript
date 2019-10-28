#!/usr/bin/env python
# encoding: utf-8

def build(bld):
    vehicle = bld.path.name
    bld.ap_stlib(
        name=vehicle + '_libs',
        ap_vehicle=vehicle,
        ap_libraries=bld.ap_common_vehicle_libraries() + [
            'AC_AttitudeControl',
            'AC_Fence',
            'AC_Avoidance',
            'AC_PID',
            'AC_WPNav',
            'AP_Camera',
            'AP_InertialNav',
            'AP_JSButton',
            'AP_LeakDetector',
            'AP_Motors',
            'AP_RCMapper',
            'AP_Beacon',
            'AP_TemperatureSensor',
            'AP_Arming'
        ],
    )

    bld.ap_program(
        program_name='ardusub',
        program_groups=['bin', 'sub'],
        use=vehicle + '_libs',
    )
