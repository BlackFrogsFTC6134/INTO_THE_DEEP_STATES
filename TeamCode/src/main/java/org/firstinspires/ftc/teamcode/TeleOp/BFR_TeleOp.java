/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp", group="Linear OpMode")

public class BFR_TeleOp extends LinearOpMode {

    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx Linear_Actuator = null;
    private DcMotorEx Rotate_Actuator = null;
    private DcMotorEx Linear_Viper = null;
    private DcMotorEx Rotate_Viper = null;
    public boolean Use_Motor_Encoder = false; // true is using motor encoder. False if using odometry pods
    // Setup a variable for each drive wheel to save power level for telemetry
    double Linear_Actuator_Power;
    double Rotate_Actuator_Power;
    double Linear_Viper_Power;
    double Rotate_Viper_Power;

    public static final double FULL_FIELD_INCHES = 141.24;
    public static final double HALF_FIELD_INCHES = FULL_FIELD_INCHES / 2.0;
    public static final double FULL_TILE_INCHES = FULL_FIELD_INCHES / 6.0;

    public static final double ROBOT_LENGTH = 18.0;
    public static final double ROBOT_WIDTH = 18.0;

    public static final double DRIVE_SLOW_SCALE = 0.3;
    public static final double DRIVE_NORMAL_SCALE = 0.5;
    public static final double TURN_SLOW_SCALE = 0.3;
    public static final double TURN_NORMAL_SCALE = 0.6;

    public static final double LINEAR_ACTUATOR_SLOW_SCALE = 0.7;
    public static final double LINEAR_ACTUATOR_NORMAL_SCALE = 1.0;
    public static boolean LINEAR_ACTUATOR_FULL_POWER = true;

    private PIDController LINEAR_VIPER_pidController;

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.05;

    public static final double ROTATE_ACTUATOR_SLOW_SCALE = 0.7;
    public static final double ROTATE_ACTUATOR_NORMAL_SCALE = 1.0;
    public static final boolean ROTATE_ACTUATOR_FULL_POWER = true;

    public static final double LINEAR_VIPER_SLOW_SCALE = 0.7;
    public static final double LINEAR_VIPER_NORMAL_SCALE = 1.0;
    public static boolean LINEAR_VIPER_FULL_POWER = true;

    public static final double ROTATE_VIPER_SLOW_SCALE = 0.7;
    public static final double ROTATE_VIPER_NORMAL_SCALE = 1.0;
    public static final boolean ROTATE_VIPER_FULL_POWER = true;

    // Chassis Variables
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;

    double  X_movement;
    double  Y_movement;

    double drive1 = 0;
    double strafe = 0;
    double turn = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        Linear_Actuator = hardwareMap.get(DcMotorEx.class, "Linear_Actuator");
        Rotate_Actuator = hardwareMap.get(DcMotorEx.class, "Rotate_Actuator");
        Linear_Viper = hardwareMap.get(DcMotorEx.class, "Linear_Viper");
        Rotate_Viper = hardwareMap.get(DcMotorEx.class, "Rotate_Viper");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        Linear_Actuator.setDirection(DcMotorEx.Direction.FORWARD);
        Rotate_Actuator.setDirection(DcMotorEx.Direction.FORWARD);
        Linear_Viper.setDirection(DcMotorEx.Direction.FORWARD);
        Rotate_Viper.setDirection(DcMotorEx.Direction.FORWARD);

        Linear_Actuator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Rotate_Actuator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Linear_Viper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Rotate_Viper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        Linear_Actuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Rotate_Actuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Linear_Viper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Rotate_Viper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        LINEAR_VIPER_pidController = new PIDController(kP, kI, kD);

        if (Use_Motor_Encoder) {
            leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("Status", "TeleOp Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            X_movement = -gamepad1.left_stick_y * DRIVE_NORMAL_SCALE;
            Y_movement = -gamepad1.left_stick_x * DRIVE_NORMAL_SCALE;
            drive1 = -gamepad1.left_stick_y * DRIVE_NORMAL_SCALE;  // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x * TURN_NORMAL_SCALE;  // Reduce strafe rate to 50%.
            turn = -gamepad1.right_stick_x * TURN_NORMAL_SCALE;
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
                telemetry.addData("Mecanum wheels driving", "");

                waitForStart();

                while (opModeIsActive()) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y * DRIVE_NORMAL_SCALE,
                                    -gamepad1.left_stick_x * DRIVE_NORMAL_SCALE
                            ),
                            -gamepad1.right_stick_x * TURN_NORMAL_SCALE
                    ));


                    if (LINEAR_ACTUATOR_FULL_POWER) {
                        Linear_Actuator_Power = -gamepad2.left_stick_y * LINEAR_ACTUATOR_NORMAL_SCALE;
                    } else {
                        Linear_Actuator_Power = -gamepad2.left_stick_y * LINEAR_ACTUATOR_SLOW_SCALE;
                    }

                    if (ROTATE_ACTUATOR_FULL_POWER) {
                        Rotate_Actuator_Power = gamepad2.left_stick_x * ROTATE_ACTUATOR_NORMAL_SCALE;
                    } else {
                        Rotate_Actuator_Power = gamepad2.left_stick_x * ROTATE_ACTUATOR_SLOW_SCALE;
                    }

                    if (LINEAR_VIPER_FULL_POWER) {
                        Linear_Viper_Power = -gamepad2.right_stick_y * LINEAR_VIPER_NORMAL_SCALE;
                    } else {
                        Linear_Viper_Power = gamepad2.right_stick_y * LINEAR_VIPER_SLOW_SCALE;
                    }

                    if (ROTATE_VIPER_FULL_POWER) {
                        Rotate_Viper_Power = gamepad2.right_stick_x * ROTATE_VIPER_NORMAL_SCALE;
                    } else {
                        Rotate_Viper_Power = gamepad2.right_stick_x * ROTATE_VIPER_SLOW_SCALE;
                    }

                    if (gamepad2.dpad_up) {
                        LINEAR_ACTUATOR_FULL_POWER = true;
                    }

                    if (gamepad2.dpad_down) {
                        LINEAR_ACTUATOR_FULL_POWER = false;
                    }

                    if (gamepad2.dpad_left) {
                        LINEAR_VIPER_FULL_POWER = true;
                    }

                    if (gamepad2.dpad_right) {
                        LINEAR_VIPER_FULL_POWER = false;
                    }

                    // Send calculated power to wheels
                    Linear_Actuator.setPower(Linear_Actuator_Power);
                    Rotate_Actuator.setPower(Rotate_Actuator_Power);
                    Rotate_Viper.setPower(Rotate_Viper_Power);
                    Linear_Viper.setPower(Linear_Viper_Power);

                    // Show the elapsed game time and wheel power.
                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.addData("Linear_Actuator_Power ", "(%.2f), Rotate_Actuator_Power (%.2f)", Linear_Actuator_Power, Rotate_Actuator_Power);
                    telemetry.addData("Linear_Viper_Power", "(%.2f), Rotate_Viper_Power (%.2f)", Linear_Viper_Power, Rotate_Viper_Power);
                    //telemetry.addData("Meacanum_Front", " left_Front (%.2f), right_Front (%.2f)", leftFront, rightFront);
                    //telemetry.addData("Meacanum_Back", " left_Back (%.2f), right_Back (%.2f)", leftBack, rightBack);
                    telemetry.update();
                }
            }
        }
    }
}