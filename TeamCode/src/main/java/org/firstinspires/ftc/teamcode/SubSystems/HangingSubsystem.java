package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface HangingSubsystem extends Subsystem {
    void initialize(HardwareMap hardwareMap);

    void extend();
    void retract();
    boolean isExtended();
}
