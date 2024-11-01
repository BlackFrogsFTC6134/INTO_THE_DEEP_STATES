package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Autonomous(name = "Roadrunner Auton")
public class Roadrunner_Auton extends LinearOpMode {
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0,0), 0));

        waitForStart();
        Action TrajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToX(10)
                .build();

        Action TrajectoryAction2 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(5, 5), Math.toRadians(90))
                .build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        TrajectoryAction1,
                        TrajectoryAction2
                )
        );
    }
}
