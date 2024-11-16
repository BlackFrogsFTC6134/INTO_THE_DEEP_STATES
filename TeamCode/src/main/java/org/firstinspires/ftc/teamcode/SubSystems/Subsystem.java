package org.firstinspires.ftc.teamcode.states.SubSystems;


import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Subsystem {
        void initialize(HardwareMap hardwareMap);
        void periodic();
        void stop();
    }
