package org.firstinspires.ftc.teamcode.SubSystems;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Subsystem {
        void initialize(HardwareMap hardwareMap);
        void update(HardwareMap hardwareMap, Gamepad gamepadState1, Gamepad gamepadState2);
        void stopSubsystem(HardwareMap hardwareMap);
    }
