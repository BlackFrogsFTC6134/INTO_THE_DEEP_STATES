package org.firstinspires.ftc.teamcode.states.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystems.Subsystem;

public interface HangingSubsystem extends Subsystem {
    void initialize(HardwareMap hardwareMap);

    void extend();
    void retract();
    boolean isExtended();
}
