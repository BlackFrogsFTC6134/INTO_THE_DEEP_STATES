package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface ViperSlideSubsystem extends Subsystem {
        void initialize(HardwareMap hardwareMap);

        void setSlidePosition(int position);
        int getSlidePosition();
        void setSlideSpeed(double speed);

}
