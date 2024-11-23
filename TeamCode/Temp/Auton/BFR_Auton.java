package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

// Roadrunner 1.0 API documentation - https://acmerobotics.github.io/road-runner/core/0.4.2/javadoc/index.html?index-all.html

@Config
@Autonomous(name = "Template Auton", group = "BFR_Auton")
public class BFR_Auton extends LinearOpMode {

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        //DcMotor motor1 = hardwareMap.get(DcMotor.class,  "motor");

        // Delcare Trajectory as such

    }
}
