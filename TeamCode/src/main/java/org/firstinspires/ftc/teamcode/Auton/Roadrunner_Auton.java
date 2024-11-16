package org.firstinspires.ftc.teamcode.states.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Autonomous(name = "Roadrunner Auton")
public class Roadrunner_Auton extends LinearOpMode {
    MecanumDrive drive;

    DcMotorEx leftFront = null;
    DcMotorEx leftBack = null;
    DcMotorEx rightFront = null;
    DcMotorEx rightBack = null;
    public static boolean RUN_USING_ENCODER = false;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0,0), 0));
/*
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        if (DriveConstants.RUN_USING_ENCODER) {
            leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } else {
            leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
*/
    waitForStart();
        Action TrajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToXConstantHeading(50)
                .splineTo(new Vector2d(45,25), Math.toRadians(0))
                .build();
/*
        Action TrajectoryAction2 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(5, 5), Math.toRadians(90))
                .build();

        Action TrajectoryAction2 = drive.actionBuilder(drive.pose)
                .lineToX(10)

                .splineTo()
                .build();
*/
       // drive.setDrivePowers(new PoseVelocity2d(new Vector2d(1.0,0.0),0.0));
        Actions.runBlocking(
                new SequentialAction(
                        TrajectoryAction1
                )
        );
    }
}