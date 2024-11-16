package org.firstinspires.ftc.teamcode.states.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface DriveSubsystem {
    void initialize(HardwareMap hardwareMap);

    void drive(double forward, double strafe, double turn);
        void setDrivePowers(double leftFront, double leftBack, double rightFront, double rightBack);

    void periodic();

    void stop();
}
