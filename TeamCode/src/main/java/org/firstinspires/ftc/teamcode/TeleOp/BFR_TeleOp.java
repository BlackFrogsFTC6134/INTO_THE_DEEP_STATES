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
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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

@TeleOp(name = "TeleOp", group = "Linear OpMode")

public class BFR_TeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx linearActuator = null;
    private DcMotorEx rotateActuator = null;
    private DcMotorEx viperSlideMotor = null;
    private DcMotorEx rotateSlideMotor = null;

    // Setup a variable for each drive wheel to save power level for telemetry
    double linearActuatorPower;
    double rotateActuatorPower;
    double viperSlideMotorPower;
    double rotateSlideMotorPower;

    private DistanceSensor sensorDistance;
    Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

    public static final double FULL_FIELD_INCHES = 141.24;
    public static final double HALF_FIELD_INCHES = FULL_FIELD_INCHES / 2.0;
    public static final double FULL_TILE_INCHES = FULL_FIELD_INCHES / 6.0;

    public static final double ROBOT_LENGTH = 18.0;
    public static final double ROBOT_WIDTH = 18.0;

    public static final double DRIVE_SLOW_SCALE = 0.3;
    public static final double DRIVE_NORMAL_SCALE = 1.0;
    public static final double TURN_SLOW_SCALE = 0.3;
    public static final double TURN_NORMAL_SCALE = 1.0;

    private PIDController LINEAR_VIPER_pidController;

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.05;

    public static final double LINEAR_ACTUATOR_SLOW_SCALE = 0.7;
    public static final double LINEAR_ACTUATOR_NORMAL_SCALE = 1.0;
    public static boolean LINEAR_ACTUATOR_FULL_POWER = true;

    public static final double ROTATE_ACTUATOR_SLOW_SCALE = 0.7;
    public static final double ROTATE_ACTUATOR_NORMAL_SCALE = 1.0;
    public static final boolean ROTATE_ACTUATOR_FULL_POWER = true;

    public static final double LINEAR_VIPER_SLOW_SCALE = 0.3;
    public static final double LINEAR_VIPER_NORMAL_SCALE = 0.6;
    public static boolean LINEAR_VIPER_FULL_POWER = true;

    public static final double ROTATE_VIPER_SLOW_SCALE = 0.7;
    public static final double ROTATE_VIPER_NORMAL_SCALE = 1.0;
    public static final boolean ROTATE_VIPER_FULL_POWER = true;

    // Chassis Variables
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;

    // Variables to track button states
    private boolean lastAState = false;
    private boolean lastBState = false;

    // Define a dead zone threshold to overcome stick drift
    private static final double DEAD_ZONE_THRESHOLD = 0.1;
    private double drive1;
    private double strafe;
    private double turn;

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    // Define the target position and time limit
    private static final int LINEAR_ACTUATOR_TARGET_POSITION_UP = 10; // Target position for moving up (in encoder ticks)
    private static final int LINEAR_ACTUATOR_TARGET_POSITION_DOWN = 0;   // Target position for moving down (in encoder ticks)
    private static final int LINEAR_VIPER_TARGET_POSITION_UP = 50; // Target position for moving up (in encoder ticks)
    private static final int LINEAR_VIPER_TARGET_POSITION_DOWN = 0;   // Target position for moving down (in encoder ticks)

    private static final int ROTATE_VIPER_TARGET_POSITION_UP = 100; // Target position for moving up (in encoder ticks)
    private static final int ROTATE_VIPER_TARGET_POSITION_DOWN = 0;   // Target position for moving down (in encoder ticks)

    private static final long SLIDE_TIME_UP  = 2000 ; // Time to move up in milliseconds
    private static final long SLIDE_TIME_DOWN  = 2000; // Time to move down in milliseconds

    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(45, 45);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(90);

    // Section to init claw and intake
    // Declare the servos
    private Servo clawServo;    // For opening and closing the claw
    private Servo rotateServo;  // For rotating the claw
    private Servo hangServo; //For the hooks

    // Servo positions
    private static final double CLAW_OPEN_POSITION = 1.0;   // Fully open position, (ready to pick sample)
    private static final double CLAW_CLOSE_POSITION = 0.0;  // Fully closed position, (holding sample)
    private static final double RETRACT_HOOK_DOWN_POSITION = -2.0;   // Fully open position, (ready to pick sample)
    private static final double RAISE_HOOK_UP_POSITION = 2.0;  // Fully closed position, (holding sample)
    //private static final double CLAW_ROTATE_LEFT_POSITION = 0.0;  // Rotate claw left
    //private static final double CLAW_ROTATE_RIGHT_POSITION = 1.0; // Rotate claw right
    //private static final double CLAW_ROTATE_CENTER_POSITION = 0.5; // Center position

    private double integralClaw = 0;
    private double previousErrorClaw = 0;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        initializeChassis();
        initializeSubsystems();
        initializeSubsystemsUsingEncoderBeforeStart();
        updateTelemetryBeforeStart();
        //     initializeSubsystemsWithoutEncoder();
        //     initializeSubsystemsUsingPosition();

        telemetry.addData("Status", "TeleOp Initialized");
        telemetry.update();

        // Go go BlackFrog robot! (driver presses START)
        waitForStart();
        runtime.reset();
        initializeSubsystemsUsingEncoderAfterStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            updateTelemetryAfterStart();
            handleGamePad1Inputs();
            handleGamepad2Inputs();
            updateMotorPowers();

        }
    }

    // Function to initialize chassis
    private void initializeChassis() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        /*
        Zero Power Behavior: Set the zero power behavior of the motors to BRAKE or FLOAT
        depending on your design requirements. BRAKE will hold the motors in place when no power is applied,
        while FLOAT allows the motors to spin freely.
        */

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        if (DriveConstants.RUN_USING_ENCODER) {
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

        // you can use this as a regular DistanceSensor.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.

    }

    // Function to initialize subsystems (Arms & vipers)
    private void initializeSubsystems() {
        initializeActuator();
        initializeViper();
        initializeClaw();
        //initializeIntake();
        initializeHang();
        //initializeHook();
    }

    private void initializeActuator() {
        linearActuator = hardwareMap.get(DcMotorEx.class, "Linear_Actuator");
        rotateActuator = hardwareMap.get(DcMotorEx.class, "Rotate_Actuator");

        linearActuator.setDirection(DcMotorEx.Direction.FORWARD);
        rotateActuator.setDirection(DcMotorEx.Direction.FORWARD);

        linearActuator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rotateActuator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        linearActuator.setPower(0);
        rotateActuator.setPower(0);
    }

    private void initializeViper() {
        viperSlideMotor = hardwareMap.get(DcMotorEx.class, "Linear_Viper");
        rotateSlideMotor = hardwareMap.get(DcMotorEx.class, "Rotate_Viper");

        viperSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rotateSlideMotor.setDirection(DcMotorEx.Direction.FORWARD);

        viperSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rotateSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        viperSlideMotor.setPower(0);
        rotateSlideMotor.setPower(0);
    }

    private void initializeClaw() {
        // Initialize claws. Additional configuration needed.
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // Set initial positions
        clawServo.setPosition(CLAW_OPEN_POSITION);
        telemetry.addLine(">> clawServo: Initialized");
        telemetry.addData("clawServo Position: ", CLAW_OPEN_POSITION);
        telemetry.update();
    }

    private void initializeIntake() {

        // Initialize claws. Additional configuration needed.

    }

    private void initializeHang() {
        // Initialize Hang. Additional configuration needed.
        // Initialize claws. Additional configuration needed.
        hangServo = hardwareMap.get(Servo.class, "hangServo");

        // Set initial positions
        hangServo.setPosition(RETRACT_HOOK_DOWN_POSITION);
        telemetry.addLine(">> hangServo: Initialized");
        telemetry.addData("hangServo Position: ", RETRACT_HOOK_DOWN_POSITION);
        telemetry.update();
    }

    private void initializeHook() {

        // Initialize Hang. Additional configuration needed.

    }

        // Function to initialize subsystems Without Encoder
    private void initializeSubsystemsWithoutEncoder() {
        linearActuator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rotateActuator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        viperSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rotateSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Function to initialize subsystems UsingEncoder during initialization
    private void initializeSubsystemsUsingEncoderBeforeStart() {
        linearActuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rotateActuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rotateSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Function to initialize subsystems UsingEncoder after start
    private void initializeSubsystemsUsingEncoderAfterStart() {
        linearActuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rotateActuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        viperSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rotateSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // Function to initialize subsystems UsingEncoder
    private void initializeSubsystemsUsingPosition() {
        linearActuator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rotateActuator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        viperSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rotateSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

    }

    // Function to handle drive input from gamepad1
    private void handleGamePad1Inputs() {
        drive1 = applyDeadZone(-gamepad1.left_stick_y * DRIVE_NORMAL_SCALE);
        strafe = applyDeadZone(-gamepad1.left_stick_x * DRIVE_NORMAL_SCALE);
        turn = applyDeadZone(-gamepad1.right_stick_x * TURN_NORMAL_SCALE);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            drive1,
                            strafe
                    ),
                    turn
            ));
        }

        if (gamepad1.b) {
            raiseHook();
            telemetry.addData("Raise hook", "");
        } else if (gamepad1.x) {
            retractHook();
            telemetry.addData("Retract hook", "");
        }
    }

    // Function to handle manipulator input from gamepad2
    private void handleGamepad2Inputs() {
        rotateActuatorPower = applyDeadZone(-gamepad2.left_stick_y * (ROTATE_ACTUATOR_FULL_POWER ? ROTATE_ACTUATOR_NORMAL_SCALE: ROTATE_ACTUATOR_SLOW_SCALE));
        linearActuatorPower = applyDeadZone(gamepad2.left_stick_x * ( LINEAR_ACTUATOR_FULL_POWER? LINEAR_ACTUATOR_NORMAL_SCALE : LINEAR_ACTUATOR_SLOW_SCALE));

        rotateSlideMotorPower = applyDeadZone(-gamepad2.right_stick_y * (ROTATE_VIPER_FULL_POWER ? ROTATE_VIPER_NORMAL_SCALE : ROTATE_VIPER_SLOW_SCALE));
        viperSlideMotorPower = applyDeadZone(gamepad2.right_stick_x * (LINEAR_VIPER_FULL_POWER ? LINEAR_VIPER_NORMAL_SCALE : LINEAR_VIPER_SLOW_SCALE));

        if (gamepad2.dpad_up) {
            LINEAR_ACTUATOR_FULL_POWER = true;
            telemetry.addData("Setting linear viper power to ", "FULL POWER");
            telemetry.update();
        } else if (gamepad2.dpad_down) {
            LINEAR_ACTUATOR_FULL_POWER = false;
            telemetry.addData("Setting linear viper power to ", "REDUCED POWER");
            telemetry.update();
        }

        if (gamepad2.dpad_left) {
            LINEAR_VIPER_FULL_POWER = true;
            telemetry.addData("Setting viper power to ", "FULL POWER");
            telemetry.update();
        } else if (gamepad2.dpad_right) {
            LINEAR_VIPER_FULL_POWER = false;
            telemetry.addData("Setting viper power to ", "REDUCED POWER");
            telemetry.update();
        }

        // Control to move the linear actuator down (select either Position control or time control)
        if (gamepad2.b) {
            moveViperToPosition(LINEAR_VIPER_TARGET_POSITION_DOWN);
            //     moveForSpecifiedTime(SLIDE_TIME_DOWN);
        }
        // Control to move the slide down
        if (gamepad2.x) {
            moveViperToPosition(LINEAR_VIPER_TARGET_POSITION_UP);
            //     moveForSpecifiedTime(SLIDE_TIME_DOWN);
        }

        // Control to move the linear actuator up (select either Position control or time control)
        if (gamepad2.a) {
            moveLinearActuatorToPosition(LINEAR_ACTUATOR_TARGET_POSITION_DOWN);
            //     moveForSpecifiedTime(SLIDE_TIME_DOWN);
        }

        if (gamepad2.y) {
            moveLinearActuatorToPosition(LINEAR_ACTUATOR_TARGET_POSITION_UP);
            //  moveForSpecifiedTime(SLIDE_TIME_UP)
        }

        if (gamepad2.right_bumper) {
            closeClaw();
        } else if (gamepad2.left_bumper) {
            openClaw();
        }


        if (gamepad2.back) {
            operateClaw();
        }
    }

    // Function to update motor power values
    private void updateMotorPowers() {
        linearActuator.setPower(linearActuatorPower);
        rotateActuator.setPower(rotateActuatorPower);
        rotateSlideMotor.setPower(rotateSlideMotorPower);
        viperSlideMotor.setPower(viperSlideMotorPower);
    }

    private void updateTelemetryBeforeStart() {
        telemetry.addLine("Robot Status after start: ")
                .addData(">> RUN_USING_ENCODER?: ", String.valueOf(DriveConstants.RUN_USING_ENCODER));

        telemetry.addLine(">> Robot Arms Status: ");
        telemetry.addData(">> Linear actuator power: ", "%.2f", linearActuatorPower);
        telemetry.addData(">> Linear actuator position (ticks/rev): ", linearActuator.getCurrentPosition());
        telemetry.addData(">> Linear actuator UP target: ", LINEAR_ACTUATOR_TARGET_POSITION_UP);
        telemetry.addData(">> Linear actuator DOWN target: ", LINEAR_ACTUATOR_TARGET_POSITION_DOWN);

        telemetry.addLine("***************");
        telemetry.addData(">> Rotate actuator power: ", "%.2f", rotateActuatorPower);
        telemetry.addData(">> Rotate actuator position (ticks/rev): ", rotateActuator.getCurrentPosition());

        telemetry.addLine("***************");
        telemetry.addData(">> Linear viper power: ", "%.2f", viperSlideMotorPower);
        telemetry.addData(">> Linear viper position (ticks/rev): ", viperSlideMotor.getCurrentPosition());

        telemetry.addLine("***************");
        telemetry.addData(">> Linear viper UP target: ", LINEAR_VIPER_TARGET_POSITION_UP);
        telemetry.addData(">> Linear viper DOWN target: ", LINEAR_VIPER_TARGET_POSITION_DOWN);

        telemetry.addLine("***************");
        telemetry.addData(">> Rotate viper power: ", "%.2f", rotateSlideMotorPower);
        telemetry.addData(">> Rotate viper position (ticks/rev):",rotateSlideMotor.getCurrentPosition());
        telemetry.addData(">> Rotate viper UP target: ", ROTATE_VIPER_TARGET_POSITION_UP);
        telemetry.addData(">> Rotate viper DOWN target: ", ROTATE_VIPER_TARGET_POSITION_DOWN);

        telemetry.update();
    }
    // Function to update telemetry data
    private void updateTelemetryAfterStart() {
        telemetry.addLine("Robot Status after start: ");
        telemetry.addData("Run Time: ", runtime.toString());

        telemetry.addData(">> drive1: ", "%.2f", drive1);
        telemetry.addData(">> Strafe: ", "%.2f", strafe);
        telemetry.addData(">> Turn: ", "%.2f", turn);

        telemetry.addLine("***************");
        telemetry.addData(">> Robot Arms Status", ":");
        telemetry.addData(">> Linear actuator power: ", "%.2f", linearActuatorPower);
        telemetry.addData(">> Linear actuator position (ticks/rev): ", linearActuator.getCurrentPosition());

        telemetry.addLine("***************");
        telemetry.addData(">> Rotate actuator power: ", "%.2f", rotateActuatorPower);
        telemetry.addData(">> Rotate actuator position (ticks/rev): ", rotateActuator.getCurrentPosition());

        telemetry.addLine("***************");
        telemetry.addData(">> Linear viper power: ", "%.2f", viperSlideMotorPower);
        telemetry.addData(">> Linear viper position(ticks/rev) : ", viperSlideMotor.getCurrentPosition());

        telemetry.addLine("***************");
        telemetry.addData(">> Rotate viper power: ", "%.2f", rotateSlideMotorPower);
        telemetry.addData(">> Rotate viper position (ticks/rev): ", rotateSlideMotor.getCurrentPosition());

        // Show joystick information as some other illustrative data
        telemetry.addLine("left joystick | ")
                .addData("x", gamepad1.left_stick_x)
                .addData("y", gamepad1.left_stick_y);
        telemetry.addLine("right joystick | ")
                .addData("x", gamepad2.right_stick_x)
                .addData("y", gamepad2.right_stick_y);

        telemetry.addLine("***************");
        telemetry.addData("deviceName", sensorDistance.getDeviceName() );
        telemetry.addData("range", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));
        telemetry.update();
        //telemetry.addLine("***************");
        /* Rev2mDistanceSensor specific methods.
        telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
        telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

        */

    }

    // Function to move the linear actuator to the specified position
    private void moveLinearActuatorToPosition(int targetPosition) {
        // Set power to the motor
        linearActuator.setPower(LINEAR_VIPER_FULL_POWER ? LINEAR_VIPER_NORMAL_SCALE : LINEAR_VIPER_SLOW_SCALE); // Set power to full (you can adjust this as needed)
        linearActuator.setTargetPosition(targetPosition);
        linearActuator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        linearActuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        linearActuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // Function to move the viper to the specified position
    private void moveViperToPosition(int targetPosition) {
        //viperSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setTargetPosition(targetPosition);
        viperSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        // Set power to the motor
        viperSlideMotor.setPower(LINEAR_VIPER_FULL_POWER ? LINEAR_VIPER_NORMAL_SCALE : LINEAR_VIPER_SLOW_SCALE); // Set power to full (you can adjust this as needed)
        //viperSlideMotor.setPower(viperSlideMotorPower); // Set power to full (you can adjust this as needed)

        //viperSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        // Set back to RUN_USING_ENCODER mode
      //  viperSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // Function to move the slide within the specified time
    private void moveLinearActuatorForSpecifiedTime(long targetTime) {

        // Set power to the motor
        viperSlideMotor.setPower(LINEAR_VIPER_FULL_POWER ? LINEAR_VIPER_NORMAL_SCALE : LINEAR_VIPER_SLOW_SCALE); // Set power to full (you can adjust this as needed)

        telemetry.addData(">> Moving viper until specified time", targetTime);
        telemetry.addData(">> Current viper Position", viperSlideMotor.getCurrentPosition());
        telemetry.update();

        sleep(targetTime);

        // Stop the motor
        viperSlideMotor.setPower(0);
    }

    private void openClaw() {
        setClawPosition(CLAW_OPEN_POSITION);
    }

    private void closeClaw() {
        setClawPosition(CLAW_CLOSE_POSITION);
    }

    private void raiseHook() {
        setHookPosition(RAISE_HOOK_UP_POSITION);
    }

    private void retractHook() {
        setHookPosition(RETRACT_HOOK_DOWN_POSITION);
    }

    private void setClawPosition(double position) {
        clawServo.setPosition(position);
       // rightClawServo.setPosition(1 - position); // Assuming right servo is mirrored
    }

    private void setHookPosition(double position) {
        hangServo.setPosition(position);
        // rightClawServo.setPosition(1 - position); // Assuming right servo is mirrored
    }

    public void operateClaw() {
        /*if (gamepad2.right_bumper) {
            closeClaw();
        } else if (gamepad2.left_bumper) {
            openClaw();
        }
*/
        // Use right trigger for fine control of claw position
        double targetPosition = gamepad2.right_trigger * (CLAW_OPEN_POSITION - CLAW_CLOSE_POSITION) + CLAW_CLOSE_POSITION;
        setClawPositionWithPID(targetPosition);
    }

    private void setClawPositionWithPID(double targetPosition) {
        double currentPosition = clawServo.getPosition();
        double error = targetPosition - currentPosition;

        double dt = timer.seconds();
        timer.reset();

        integralClaw += error * dt;
        double derivative = (error - previousErrorClaw) / dt;

        double output = kP * error + kI * integralClaw + kD * derivative;

        setClawPosition(currentPosition + output);

        previousErrorClaw = error;
    }
    // Function to apply dead zone filtering
    private double applyDeadZone(double value) {
        // If the joystick input is smaller than the dead zone threshold, set it to 0
        if (Math.abs(value) < DEAD_ZONE_THRESHOLD) {
            return 0.0;
        }
        // Otherwise, return the original value
        return value;
    }
}
