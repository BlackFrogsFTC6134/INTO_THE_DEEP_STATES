package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface DrivetrainSubsystem {
    /** Drive Motor Proportional Gain */
    double DRIVE_P = 16.0;

    /** Drive Motor Integral Gain */
    double DRIVE_I = 6.0;

    void initialize(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException;
    void update(Telemetry telemetry);
    void stopSubsystem(Telemetry telemetry) throws InterruptedException;
}


