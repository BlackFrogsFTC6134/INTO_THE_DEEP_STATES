package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface DriveSubsystem {
    void initialize(HardwareMap hardwareMap);

    void drive();

    void increaseDrivePower();

    void decreaseDrivePower();

    void makeRearOfRobotFront();

    void makeLeftOfRobotFront();

    void makeRightOfRobotFront();

    void setDrivePowers();

    void DriveForward(int ForwardDistance);

    void DriveRearward(int RearwardDistance);

    void TurnRight(int RightTurnDegrees);

    void LeftTurn(int LeftTurnDegrees);

    void testMotorDirections();

    void periodic();

    void stop();
}
