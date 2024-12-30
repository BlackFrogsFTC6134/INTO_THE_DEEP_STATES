package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface ViperSlideSubsystem {
        void initialize(HardwareMap hardwareMap, Telemetry telemetry);

        void setSlidePosition(int targetPosition,Telemetry telemetry);
        int getSlidePosition(Telemetry telemetry);
        void setSlideSpeed(double slideSpeed, Telemetry telemetry);

}
