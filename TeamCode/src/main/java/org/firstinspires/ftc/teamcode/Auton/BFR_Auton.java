package org.firstinspires.ftc.teamcode.Auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
