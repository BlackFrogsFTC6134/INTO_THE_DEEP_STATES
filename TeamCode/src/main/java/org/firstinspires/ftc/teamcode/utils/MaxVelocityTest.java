package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// Source: https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit?tab=t.0

/*
The formula "F = 32767 / maxV" for calculating the feedforward (F) coefficient in a PIDF controller is not directly derived from first principles, but rather is a practical approximation used in specific motor control scenarios, particularly in FIRST Tech Challenge (FTC) robotics.
        Here's a breakdown of this formula:
        32767 is the maximum value that can be represented by a signed 16-bit integer. In many motor control systems, including those used in FTC, this represents the maximum motor power or speed.
maxV represents the maximum velocity of the motor, typically measured in encoder ticks per second.
The division 32767 / maxV essentially creates a scaling factor that maps the desired velocity to the appropriate motor power.
This formula is used as a starting point for tuning the feedforward coefficient, which helps predict the amount of power needed to achieve a desired velocity. It's important to note that:
This is a simplified approach and may not be optimal for all systems.
The actual value often needs fine-tuning based on empirical testing.
It assumes a linear relationship between power and velocity, which may not always be the case in real-world scenarios.
For more precise tuning, it's recommended to use methods like:
Empirical testing using the MaxVelocityTuner in RoadRunner.
Calculating based on motor specifications and system characteristics.
Iterative tuning processes that account for real-world factors like friction and inertia.
Remember that while this formula provides a reasonable starting point, optimal PIDF tuning often requires experimentation and adjustment based on your specific robot's behavior and performance requirements.
*/

@TeleOp(name = "MaxVelocityTest", group = "Utility")
public class MaxVelocityTest extends LinearOpMode {
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    double currentLeftFrontVelocity, currentLeftBackVelocity, currentRightFrontVelocity, currentRightBackVelocity;
    double maxLeftFrontVelocity, maxLeftBackVelocity, maxRightFrontVelocity, maxRightBackVelocity = 0.0;
    double leftFront_P, leftFront_I, leftFront_D, leftFront_F;
    double leftBack_P, leftBack_I, leftBack_D, leftBack_F;
    double rightFront_P, rightFront_I, rightFront_D, rightFront_F;
    double rightBack_P, rightBack_I, rightBack_D, rightBack_F;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "lfrtch0");
        leftBack = hardwareMap.get(DcMotorEx.class, "lrrech1");
        rightFront = hardwareMap.get(DcMotorEx.class, "rftch2");
        rightBack = hardwareMap.get(DcMotorEx.class, "rrrch3");

        waitForStart();
        while (opModeIsActive()) {
            currentLeftFrontVelocity = leftFront.getVelocity();
            currentLeftBackVelocity = leftBack.getVelocity();
            currentRightFrontVelocity = rightFront.getVelocity();
            currentRightBackVelocity = rightBack.getVelocity();

            if (currentLeftFrontVelocity > maxLeftFrontVelocity) {
                maxLeftFrontVelocity = currentLeftFrontVelocity;
            }

            if (currentLeftBackVelocity > maxLeftBackVelocity) {
                maxLeftBackVelocity = currentLeftBackVelocity;
            }

            if (currentRightFrontVelocity > maxRightFrontVelocity) {
                maxRightFrontVelocity = currentRightFrontVelocity;
            }

            if (currentRightBackVelocity > maxRightBackVelocity) {
                maxRightBackVelocity = currentRightBackVelocity;
            }

            telemetry.addData("currentLeftFrontVelocity", currentLeftFrontVelocity);
            telemetry.addData("currentLeftBackVelocity", currentLeftBackVelocity);
            telemetry.addData("currentRightFrontVelocity", currentRightFrontVelocity);
            telemetry.addData("currentRightBackVelocity", currentRightBackVelocity);
            telemetry.addLine();

            telemetry.addData("maxLeftFrontVelocity", maxLeftFrontVelocity);
            telemetry.addData("maxLeftBackVelocity", maxLeftBackVelocity);
            telemetry.addData("maxRightFrontVelocity", maxRightFrontVelocity);
            telemetry.addData("maxRightBackVelocity", maxRightBackVelocity);
            telemetry.addLine();

            leftFront_F = 32767 / maxLeftFrontVelocity;
            leftBack_F = 32767 / maxLeftBackVelocity;
            rightFront_F = 32767 / maxRightFrontVelocity;
            rightBack_F = 32767 / maxRightBackVelocity;

            telemetry.addData("LeftFront feedforward (F) coefficient", leftFront_F);
            telemetry.addData("LeftBack feedforward (F) coefficient", leftBack_F);
            telemetry.addData("RightFront feedforward (F) coefficient",rightFront_F);
            telemetry.addData("RightBack feedforward (F) coefficient", rightBack_F);
            telemetry.addLine();

            leftFront_P = 0.1 * leftFront_F;
            leftBack_P = 0.1 * leftBack_F;
            rightFront_P = 0.1 * rightFront_F;
            rightBack_P = 0.1 * rightBack_F;

            telemetry.addData("LeftFront feedforward (P) coefficient", leftFront_P);
            telemetry.addData("LeftBack feedforward (P) coefficient", leftBack_P);
            telemetry.addData("RightFront feedforward (P) coefficient", rightFront_P);
            telemetry.addData("RightBack feedforward (P) coefficient", rightBack_P);
            telemetry.addData("Regardless of your maximum velocity, you can set the position PIDF values to: " , 5.0);
            telemetry.addLine();

            leftFront_I = 0.1 * leftFront_P;
            leftBack_I = 0.1 * leftBack_P;
            rightFront_I = 0.1 * rightFront_P;
            rightBack_I = 0.1 * rightBack_P;

            telemetry.addData("LeftFront feedforward (I) coefficient", leftFront_I);
            telemetry.addData("LeftBack feedforward (I) coefficient", leftBack_I);
            telemetry.addData("RightFront feedforward (I) coefficient", rightFront_I);
            telemetry.addData("RightBack feedforward (I) coefficient", rightBack_I);
            telemetry.addLine();

            telemetry.addData("LeftFront feedforward (D) coefficient", 0);
            telemetry.addData("LeftBack feedforward (D) coefficient", 0);
            telemetry.addData("RightFront feedforward (D) coefficient", 0);
            telemetry.addData("RightBack feedforward (D) coefficient", 0);
            telemetry.addLine();

/*          motor.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
            motor.setPositionPIDFCoefficients(5.0)*/


            telemetry.update();
        }
    }
}
