package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

// https://gist.github.com/TheOutcastVirus/fc0de0afdcb37808288904e308a67dc7

@TeleOp(name = "Straight Line Test", group = "Test")
public class StraightLineTest extends LinearOpMode {
    private Servo continuousIntakeServo1;
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,Math.toRadians(90))); //Blackfrog_Setting

        // Define the starting pose of the robot
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90)); //Blackfrog_Setting

        // Create a trajectory to move forward by 24 inches
        TrajectoryActionBuilder tab1 = drive.actionBuilder(drive.pose)
                .lineToY(24);

        // Delcare Trajectory as such
        Action TrajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToY(10)
                .build();
/*
        Action TrajectoryAction2 = drive.actionBuilder(new Pose2d(15,20,0))
                .splineTo(new Vector2d(15,20), Math.toRadians(90))
                .build();
*/
        // Initialize claws. Additional configuration needed.
        continuousIntakeServo1 = hardwareMap.get(Servo.class, "clawServo");

        // Set initial positions
        continuousIntakeServo1.setPosition(1.0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                    new SequentialAction(
                            TrajectoryAction1, // Example of a drive action

                            // Only that this action uses a Lambda expression to reduce complexity
                            (telemetryPacket) -> {
                                telemetry.addLine("Action 1");
                                telemetry.update();
                                return false; // Returning true causes the action to run again, returning false causes it to cease
                            },/*
                            new ParallelAction( // several actions being run in parallel
                                   // TrajectoryAction2, // Run second trajectory
                                    (telemetryPacket) -> { // Run some action
                                        continuousIntakeServo1.setPosition(0);
                                        continuousIntakeServo1.setPosition(1);
                                        continuousIntakeServo1.setPosition(0);
                                        return false;
                                    }
                            ),*/
                            drive.actionBuilder(new Pose2d(15,10,Math.toRadians(125))) // Another way of running a trajectory (not recommended because trajectories take time to build and will slow down your code, always try to build them beforehand)
                                    .splineTo(new Vector2d(25, 15), 0)
                                    .build()

                    )
            );
    }
}