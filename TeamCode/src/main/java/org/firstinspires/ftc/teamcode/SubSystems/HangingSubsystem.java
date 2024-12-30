package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface HangingSubsystem extends Subsystem {
    void initialize(HardwareMap hardwareMap, Telemetry telemetry);
    void extend(Telemetry telemetry, double targetPosition);
    void retract(Telemetry telemetry, double targetPosition);
    boolean isExtended(Telemetry telemetry);
    void stopSubsystem(Telemetry telemetry);
}