package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();
            // Robot turns counterclockwise by the specified angle
            // This turn is in radians, so you must convert your degrees to radians using `Math.toRadians()`.
            // If you see `Math.PI`, it is already in radians, and does not need `Math.toRadians()`. Degrees from 0 to 360 need to be converted to radians.
            // To turn clockwise, use a negative angle.


            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //.turn(Math.PI) // Turns clockwise by `Math.PI / 6` degrees, ending at a heading of 0 degrees
                        //.turn(Math.PI / 2) // Turns counterclockwise by `Math.PI / 6` degrees, ending at the original heading
                        //.turn(-Math.PI)
                        /*.strafeToConstantHeading(new Vector2d(70,20 ))*/
                        .lineToX(70)

                        //.splineTo(new Vector2d(60, 0), Math.PI/2)
                        //.splineTo(new Vector2d(0, 0), 0)
                        .build());
        }
        }
    }

