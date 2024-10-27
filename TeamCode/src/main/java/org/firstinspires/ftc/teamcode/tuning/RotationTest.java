package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Disabled
@TeleOp(name = "Rotation Test", group = "Test")
public class RotationTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        waitForStart();

        if (isStopRequested()) return;

        // Rotate 360 degrees
        //drive.turn(Math.toRadians(360));

        while (opModeIsActive()) {

        }
    }
}