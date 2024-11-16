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

package org.firstinspires.ftc.teamcode.states.util;

import android.view.View;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
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

@TeleOp(name = "TeleOp", group = "TeleOp")

public class BFR_TeleOp_states extends LinearOpMode {

    private static double LINEAR_VIPER_STARTING_POSITION;
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx linearActuator = null;
    private DcMotorEx rotateActuator = null;
    private DcMotorEx linearViper = null;
    private DcMotorEx rotateViper = null;

    // Setup a variable for each drive wheel to save power level for telemetry
    double linearActuatorPower = 0;
    double rotateActuatorPower = 0;
    double linearViperPower = 0;
    double rotateViperPower = 0;

    private DistanceSensor sensorDistance = null;
    double rotateViperDistance = 0;
    public double DISTANCE_SENSOR_CONSTANT = 3.5;
    public double DISTANCE_SENSOR_MOUNT_HEIGHT = 1.0;
    public double SHAFT_HEIGHT = 0.0;
    public double TAN_B = 0;
    double result = 0;
    double result1 = 0;
    double resultDegrees = 0;

    TouchSensor touchSensor1 = null;  // // Touch sensor 1 Object
    TouchSensor touchSensor2 = null;  // // Touch sensor 2 Object
    TouchSensor touchSensor3 = null;  // // Touch sensor 3 Object
/** The colorSensor field will contain a reference to our color sensor hardware object */
    NormalizedColorSensor colorSensor;
    View relativeLayout;

    public static final double FULL_FIELD_INCHES = 141.24;
    public static final double HALF_FIELD_INCHES = FULL_FIELD_INCHES / 2.0;
    public static final double FULL_TILE_INCHES = FULL_FIELD_INCHES / 6.0;

    public static final double ROBOT_LENGTH = 17.5;
    public static final double ROBOT_WIDTH = 18.0;

    public static final double DRIVE_SLOW_SCALE = 0.4;
    public static final double DRIVE_NORMAL_SCALE = 0.6;
    public static final double TURN_SLOW_SCALE = 0.4;
    public static final double TURN_NORMAL_SCALE = 0.6;

    private PIDController LINEAR_VIPER_pidController;

    public static final double LINEAR_ACTUATOR_SLOW_SCALE = 0.7;
    public static final double LINEAR_ACTUATOR_NORMAL_SCALE = 1.0;
    public static boolean LINEAR_ACTUATOR_FULL_POWER = true;

    public static final double ROTATE_ACTUATOR_SLOW_SCALE = 0.7;
    public static final double ROTATE_ACTUATOR_NORMAL_SCALE = 1.0;
    public static final boolean ROTATE_ACTUATOR_FULL_POWER = true;

    public static final double LINEAR_VIPER_SLOW_SCALE = 0.9;
    public static final double LINEAR_VIPER_NORMAL_SCALE = 1.0;
    public static boolean LINEAR_VIPER_FULL_POWER = true;

    public static final double ROTATE_VIPER_SLOW_SCALE = 0.8;
    public static final double ROTATE_VIPER_NORMAL_SCALE = 1.0;
    public static final boolean ROTATE_VIPER_FULL_POWER = true;

    private boolean driveCloseToPerimeter = false;
    private static double SAFE_DISTANCE_FROM_PERIMETER = 2.0; //Inches

    // Chassis Variables
    DcMotorEx leftFront = null;
    DcMotorEx leftBack = null;
    DcMotorEx rightFront = null;
    DcMotorEx rightBack = null;

    // Variables to track button states
    private boolean lastAState = false;
    private boolean lastBState = false;

    // Define a dead zone threshold to overcome stick drift
    private static final double DEAD_ZONE_THRESHOLD = 0.01;
    private double DRIVE_SPEED = 0;
    private double TURN_SPEED = 0;
    private double STRAFE_SPEED = 0;

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    enum teleOpMode {
        SCORING,
        HANGING
    }

    // Define the target position and time limit
    private static int LINEAR_ACTUATOR_TARGET_POSITION_UP = 12000; // Target position for moving up (in encoder ticks)
    private static int LINEAR_ACTUATOR_TARGET_POSITION_DOWN = 0;   // Target position for moving down (in encoder ticks)

    private static int ROTATE_ACTUATOR_TARGET_POSITION_UP = 100; // Target position for moving up (in encoder ticks)
    private static int ROTATE_ACTUATOR_TARGET_POSITION_DOWN = 0;   // Target position for moving down (in encoder ticks)

    private int LINEAR_VIPER_TARGET_POSITION_UP; // Target position for moving up (in encoder ticks)
    private int LINEAR_VIPER_TARGET_POSITION_UP_LIMITED; // Target position for moving up (in encoder ticks)
    private int LINEAR_VIPER_TARGET_POSITION_DOWN;   // Target position for moving down (in encoder ticks)

    private static int ROTATE_VIPER_TARGET_POSITION_UP = 300; // Target position for moving up (in encoder ticks)
    private static int ROTATE_VIPER_TARGET_POSITION_DOWN = 0;   // Target position for moving down (in encoder ticks)

    Mode currentMode = Mode.DRIVER_CONTROL;
    teleOpMode currentteleOp = teleOpMode.SCORING;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(0, 0);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(90);

    // Section to init claw and intake
    // Declare the servos
    private Servo clawServo = null;    // For opening and closing the claw
    private Servo rotateServo = null;  // For rotating the claw
    private Servo hangServo = null; //For the hooks
    private CRServo continuousIntakeServo1 = null; //For intake claw
    private CRServo continuousIntakeServo2 = null; //For intake claw
    private Servo intakeRotationServo = null;

    // Servo positions
    private PIDController pidController;
    private static final double CLAW_OPEN_POSITION = 1.0;   // Fully open position, (ready to pick sample)
    private static final double CLAW_CLOSED_POSITION = 0.0;  // Fully closed position, (holding sample)
    private static final double RETRACT_HOOK_DOWN_POSITION = -1.0;   // Fully open position, (ready to pick sample)
    private static final double RAISE_HOOK_UP_POSITION = 1.0;  // Fully closed position, (holding sample)
    private static final double CONTINUOUS_INTAKE_SERVO_FORWARD_POSITION = 1.0;  // Full speed forward
    private static final double CONTINUOUS_INTAKE_SERVO_REVERSE_POSITION = 0.0;  // Full speed reverse
    private static final double CONTINUOUS_INTAKE_SERVO_CENTER_POSITION = 0.5;  // Center position
    double intakeRotationServoPosition = 0;

    //    private static final double RAISE_HOOK_UP_POSITION = 2.0;  // Fully closed position, (holding sample)
    //private static final double CLAW_ROTATE_LEFT_POSITION = 0.0;  // Rotate claw left
    //private static final double CLAW_ROTATE_RIGHT_POSITION = 1.0; // Rotate claw right
    //private static final double CLAW_ROTATE_CENTER_POSITION = 0.5; // Center position

    private static final double kP_Claw = 0.1;
    private static final double kI_Claw = 0.01;
    private static final double kD_Claw = 0.05;

    private double integralClaw = 0;
    private double previousErrorClaw = 0;
    private ElapsedTime timer_Claw = new ElapsedTime();

    private static final double kP_Viper = 0.01;
    private static final double kI_Viper = 0.001;
    private static final double kD_Viper = 0.0005;

    double previousViperErrorViper = 0;
    double integralViper = 0;
    private ElapsedTime timer_Viper = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        initializeChassis();
        initializeSubsystems();

        while (!isStarted()){
            updateTelemetryBeforeStart();
        }

        telemetry.addData("Status", "TeleOp Initialized");
        telemetry.update();

        // Go go BlackFrog robot! (driver presses START)
        waitForStart();

        runtime.reset();
        if (isStopRequested()) return;

        LINEAR_VIPER_STARTING_POSITION = linearViper.getCurrentPosition();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            updateTelemetryAfterStart();
            //handleDistanceSensors();
            //handleDigitalInputs();
            handleGamePad1Inputs();
            handleGamepad2Inputs();
            updateSubsystemsMotorPower();
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

            leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
/*
            leftFront.setVelocity(100);
            leftBack.setVelocity(100);
            rightFront.setVelocity(100);
            rightBack.setVelocity(100);
*/
            /* To determine Optimal SetVelocity
            Know your motor specifications:
            Find the maximum RPM (rotations per minute) of your motor.
            Determine the encoder resolution (ticks per revolution) for your motor.
            Calculate the theoretical maximum ticks per second:
            Max ticks/second = (Max RPM / 60 seconds) * Ticks per revolution
            Begin with about 50-70% of the theoretical maximum. */

        } else {
            leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // Function to initialize subsystems (Arms & vipers)
    private void initializeSubsystems() {
        initializeActuator();
        initializeViper();
        initializeClaw();
        initializeHang();
        initializeDistanceSensors();
        initializeDigitalInputs();
        initializeContinuousIntakeServo();
    }

    private void initializeActuator() {
        linearActuator = hardwareMap.get(DcMotorEx.class, "Linear_Actuator");
        rotateActuator = hardwareMap.get(DcMotorEx.class, "Rotate_Actuator");

        linearActuator.setPower(linearActuatorPower);
        rotateActuator.setPower(rotateActuatorPower);

        linearActuator.setDirection(DcMotorEx.Direction.FORWARD);
        rotateActuator.setDirection(DcMotorEx.Direction.FORWARD);

        linearActuator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rotateActuator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        linearActuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rotateActuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        linearActuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rotateActuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine(">> linear & rotate actuators: Initialized");
        telemetry.update();
    }

    private void initializeViper() {
        linearViper = hardwareMap.get(DcMotorEx.class, "Linear_Viper");
        rotateViper = hardwareMap.get(DcMotorEx.class, "Rotate_Viper");

        linearViper.setDirection(DcMotorEx.Direction.REVERSE);
        rotateViper.setDirection(DcMotorEx.Direction.FORWARD);

        linearViper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rotateViper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        linearViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rotateViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //linearViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        linearViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rotateViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        linearViper.setPower(linearViperPower);
        rotateViper.setPower(rotateViperPower);

        //LINEAR_VIPER_TARGET_POSITION_DOWN = linearViper.getCurrentPosition();
        //ROTATE_VIPER_TARGET_POSITION_DOWN = rotateViper.getCurrentPosition();

        //telemetry.addData(">> Linear viper position (ticks/rev) ", linearViper.getCurrentPosition());
        //telemetry.addData(">> Linear viper position (ticks/rev) ", rotateViper.getCurrentPosition());

        telemetry.addLine(">> linear & rotate vipers: Initialized");
        telemetry.update();
    }

    private void initializeClaw() {
        // Initialize claws. Additional configuration needed.
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // Set initial positions
        clawServo.setPosition(CLAW_CLOSED_POSITION);
        telemetry.addLine(">> clawServo: Initialized");
        telemetry.addData("clawServo Position: ", clawServo.getPosition());
        telemetry.update();
    }

    private void initializeContinuousIntakeServo() {
        // Initialize continuous intake (Servo should be Speed servo, not torque).
        continuousIntakeServo1 = hardwareMap.get(CRServo.class, "continuousIntakeServo1");
        continuousIntakeServo2 = hardwareMap.get(CRServo.class, "continuousIntakeServo2");
        intakeRotationServo =    hardwareMap.get(Servo.class, "intakeRotation");

        telemetry.addData("Status", "Servos initialized");
        telemetry.update();
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

    private void initializeDistanceSensors(){
        // you can use this as a regular DistanceSensor.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        rotateViperDistance = sensorDistance.getDistance((DistanceUnit.INCH));
    }

    private void initializeDigitalInputs(){
         /* A REV Robotics Touch Sensor must be configured on digital port number 1, 3, 5, or 7.
                * A Magnetic Limit Switch can be configured on any digital port.*/
        // get a reference to our touchSensor object.
        touchSensor1 = hardwareMap.get(TouchSensor.class, "sensor_touch1");
        touchSensor2 = hardwareMap.get(TouchSensor.class, "sensor_touch2");
        touchSensor3 = hardwareMap.get(TouchSensor.class, "sensor_touch3");
    }

    // Function to handle drive input from gamepad1
    private void handleGamePad1Inputs() {
        if (gamepad1.dpad_left) {
            DRIVE_SPEED = applyDeadZone(gamepad1.left_stick_y * DRIVE_SLOW_SCALE);
            STRAFE_SPEED = applyDeadZone(gamepad1.left_stick_x * DRIVE_SLOW_SCALE);
            TURN_SPEED = applyDeadZone(gamepad1.right_stick_x * TURN_SLOW_SCALE);

            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                DRIVE_SPEED,
                                STRAFE_SPEED
                        ),
                        TURN_SPEED
                ));
            }
        }

        else {
            DRIVE_SPEED = applyDeadZone(gamepad1.left_stick_y * DRIVE_NORMAL_SCALE);
            STRAFE_SPEED = applyDeadZone(gamepad1.left_stick_x * DRIVE_NORMAL_SCALE);
            TURN_SPEED = applyDeadZone(gamepad1.right_stick_x * TURN_NORMAL_SCALE);

            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                DRIVE_SPEED,
                                STRAFE_SPEED
                        ),
                        TURN_SPEED
                ));
            }
        }

        if (gamepad1.x) {

        }

        if (gamepad1.left_bumper) {
            currentteleOp = teleOpMode.HANGING;
        }

        if (gamepad1.right_bumper) {
            currentteleOp = teleOpMode.SCORING;

        }
    }

    // Function to handle manipulator input from gamepad2
    private void handleGamepad2Inputs() {

        //linearViperPower = applyDeadZone(-gamepad2.right_stick_y * (LINEAR_VIPER_FULL_POWER ? LINEAR_VIPER_NORMAL_SCALE : LINEAR_VIPER_SLOW_SCALE));
        if (currentteleOp == teleOpMode.SCORING) {
            rotateViperDistance = sensorDistance.getDistance((DistanceUnit.INCH));
            rotateViperPower = applyDeadZone(-gamepad2.right_stick_x * (ROTATE_VIPER_FULL_POWER ? ROTATE_VIPER_NORMAL_SCALE : ROTATE_VIPER_SLOW_SCALE));

            if (rotateViperDistance > 4.1 && rotateViperDistance < 7.9) {
                if (gamepad2.b) {
                    moveLinearViperToPosition(LINEAR_VIPER_TARGET_POSITION_DOWN);
                }

                if (gamepad2.x) {
                    moveLinearViperToPosition(LINEAR_VIPER_TARGET_POSITION_UP_LIMITED);
                }
            }
            rotateViperDistance = sensorDistance.getDistance((DistanceUnit.INCH));
            if (rotateViperDistance > 0.5 && rotateViperDistance < 3.7) {
                if (gamepad2.x) {
                        moveLinearViperToPosition(LINEAR_VIPER_TARGET_POSITION_UP);
                    } else if (gamepad2.b) {
                        moveLinearViperToPosition(LINEAR_VIPER_TARGET_POSITION_DOWN);

                    }

            }



    /*
             if (touchSensor1.isPressed()) {
                 rotateViperPower = applyDeadZone(-gamepad2.right_stick_x * (ROTATE_VIPER_FULL_POWER ? ROTATE_VIPER_NORMAL_SCALE: ROTATE_VIPER_SLOW_SCALE));
                if (gamepad2.x) {
                    moveLinearViperToPosition(LINEAR_VIPER_TARGET_POSITION_UP);

                }
                if (gamepad2.b) {
                    moveLinearViperToPosition(LINEAR_VIPER_TARGET_POSITION_DOWN);

                }

             } */
            /* if (!touchSensor1.isPressed()){
                rotateViper.setPower(0); //TODO
                linearViper.setPower(0);
            } */

/*
            if (gamepad2.x) {
                if (linearActuator.getCurrentPosition() > 50) {
                    //rotateViperPower = 0;
                    rotateViper.setPower(0);
                    moveLinearViperToPosition(0);
                    telemetry.addLine("We are limiting the viper slide");
                } else if (gamepad2.b) {
                    //rotateViperPower = 0;
                    rotateViper.setPower(0);
                    moveLinearViperToPosition(LINEAR_VIPER_TARGET_POSITION_DOWN);
                }
            } else if (linearActuator.getCurrentPosition() < 45) {
                rotateViperPower = applyDeadZone(-gamepad2.right_stick_x * (ROTATE_VIPER_FULL_POWER ? ROTATE_VIPER_NORMAL_SCALE : ROTATE_VIPER_SLOW_SCALE));
                rotateViper.setPower(rotateViperPower);

                if (gamepad2.x) {
                    moveLinearViperToPosition(LINEAR_VIPER_TARGET_POSITION_UP);
                }
                if (gamepad2.b) {
                    moveLinearViperToPosition(LINEAR_VIPER_TARGET_POSITION_DOWN);
                }

            }
*/

            linearViper.setPower(linearViperPower);
            rotateViper.setPower(rotateViperPower);
            //rotateViper.setPower(0);

            if (gamepad2.dpad_left) {
                //rotateActuatorPower = applyDeadZone(-gamepad2.left_stick_y * (ROTATE_ACTUATOR_FULL_POWER ? ROTATE_ACTUATOR_NORMAL_SCALE: ROTATE_ACTUATOR_SLOW_SCALE));
                //rotateActuatorToPosition(ROTATE_ACTUATOR_TARGET_POSITION_UP);
            }

            if (gamepad2.dpad_right) {
                //rotateActuatorToPosition(ROTATE_ACTUATOR_TARGET_POSITION_DOWN);
            }

            // Control to move the linear actuator up (select either Position control or time control)
            if (gamepad2.a) {

            }

            if (gamepad2.y) {

            }

            if (gamepad2.right_bumper) {
                moveClawToPosition(CLAW_OPEN_POSITION);
                //continuousIntakeServo1.setPower(1.0);
                //continuousIntakeServo2.setPower(-1.0);
            }

            if (gamepad2.left_bumper) {
                moveClawToPosition(CLAW_CLOSED_POSITION);
                //continuousIntakeServo1.setPower(-1.0);
                //continuousIntakeServo2.setPower(1.0);
            }

            //continuousIntakeServo1.setPower(0.0);
            //continuousIntakeServo2.setPower(0.0);

            // Optionally, use triggers for fine control
            //double triggerPosition = gamepad2.right_trigger * (CLAW_OPEN_POSITION - CLAW_CLOSED_POSITION) + CLAW_CLOSED_POSITION;
            //clawServo.setPosition(triggerPosition);

            // Optionally, use triggers for fine control
            //intakeRotationServoPosition = gamepad2.right_stick_x;
            //intakeRotationServo.setPosition(intakeRotationServoPosition);
        } else if (currentteleOp == teleOpMode.HANGING) {
            rotateActuatorPower = applyDeadZone(-gamepad2.left_stick_y * (ROTATE_ACTUATOR_FULL_POWER ? ROTATE_ACTUATOR_NORMAL_SCALE : ROTATE_ACTUATOR_SLOW_SCALE));
            linearActuatorPower = applyDeadZone(gamepad2.right_stick_x * (LINEAR_ACTUATOR_FULL_POWER ? LINEAR_ACTUATOR_NORMAL_SCALE : LINEAR_ACTUATOR_SLOW_SCALE));

            rotateActuator.setPower(rotateActuatorPower);
            linearActuator.setPower(linearActuatorPower);
        }
    }

    public void handlePID() {
            double viperSlideError= LINEAR_VIPER_TARGET_POSITION_UP - (linearViper.getCurrentPosition() - LINEAR_VIPER_STARTING_POSITION) * 360/384.5;

            double slide = 0.05 * viperSlideError;

            if (slide > 1) {
                slide = 1;
            }
            if (slide < -1){
                slide = -1;
            }
    }

    //linearViper.setPower(1.0);

    private void handleDistanceSensors(){
 /*       if (sensorDistance.getDistance(DistanceUnit.INCH) < SAFE_DISTANCE_FROM_PERIMETER){
            driveCloseToPerimeter = true;
        } else {
            driveCloseToPerimeter = false;
        }
  */
       /* if (rotateViperDistance > 3.0 && rotateViperDistance < 4.0) {
            LINEAR_VIPER_TARGET_POSITION_UP = 1500;
        }
        else {
            LINEAR_VIPER_TARGET_POSITION_UP = 1500;
        }
        */
    //TAN_B = (sensorDistance.getDistance(DistanceUnit.INCH) - DISTANCE_SENSOR_CONSTANT) / (DISTANCE_SENSOR_MOUNT_HEIGHT - SHAFT_HEIGHT);
    //result = Math.atan(TAN_B);
    //resultDegrees = Math.toDegrees(result);
    //result1 = Math.atan(resultDegrees);
    }


    private void handleDigitalInputs(){
        // button is pressed if value returned is LOW or false.

            //  rotateViper.setPower(0); //TODO

        //if (touchSensor3.isPressed()){
        //TODO
        //}
    }

    // Function to update motor power values
    private void updateSubsystemsMotorPower() {
        linearActuator.setPower(linearActuatorPower);
        rotateActuator.setPower(rotateActuatorPower);
        rotateViper.setPower(rotateViperPower);
        linearViper.setPower(linearViperPower);
        telemetry.addData("Linear Actuator power", linearActuatorPower );
        telemetry.addData("Rotate Actuator power", rotateActuatorPower );
        telemetry.addData("Linear Viper power", rotateViperPower );
        telemetry.addData("Rotate Viper power", linearViperPower );

    }

    private void updateTelemetryBeforeStart() {
        telemetry.addLine("Robot Status after start ")
                .addData(">> RUN_USING_ENCODER? ", String.valueOf(DriveConstants.RUN_USING_ENCODER));

        telemetry.addLine(">> Robot Arms Status ");
        telemetry.addData(">> Linear actuator power ", "%.2f", linearActuatorPower);
        //telemetry.addData(">> Linear actuator position (ticks/rev) ", linearActuator.getCurrentPosition());
        telemetry.addData(">> Linear actuator UP target ", LINEAR_ACTUATOR_TARGET_POSITION_UP);
        telemetry.addData(">> Linear actuator DOWN target ", LINEAR_ACTUATOR_TARGET_POSITION_DOWN);

        telemetry.addLine("***************");
        telemetry.addData(">> Rotate actuator power ", "%.2f", rotateActuatorPower);
        //telemetry.addData(">> Rotate actuator position (ticks/rev) ", rotateActuator.getCurrentPosition());

        telemetry.addLine("***************");
        telemetry.addData(">> Linear viper power ", "%.2f", linearViperPower);
        //telemetry.addData(">> Linear viper position (ticks/rev) ", linearViper.getCurrentPosition());

        telemetry.addLine("***************");
        telemetry.addData(">> Linear viper UP target ", LINEAR_VIPER_TARGET_POSITION_UP);
        telemetry.addData(">> Linear viper DOWN target ", LINEAR_VIPER_TARGET_POSITION_DOWN);

        telemetry.addLine("***************");
        telemetry.addData(">> Rotate viper power ", "%.2f", rotateViperPower);
       // telemetry.addData(">> Rotate viper position (ticks/rev) ", rotateViper.getCurrentPosition());
        telemetry.addData(">> Rotate viper UP target ", ROTATE_VIPER_TARGET_POSITION_UP);
        telemetry.addData(">> Rotate viper DOWN target ", ROTATE_VIPER_TARGET_POSITION_DOWN);

        telemetry.update();
    }

    // Function to update telemetry data
    private void updateTelemetryAfterStart() {
        telemetry.addLine("Robot Status after start ");
        telemetry.addData("Run Time ", runtime.toString());

        telemetry.addData(">> drive1 ", "%.2f", DRIVE_SPEED);
        telemetry.addData(">> Strafe ", "%.2f", STRAFE_SPEED);
        telemetry.addData(">> Turn:", "%.2f", TURN_SPEED);
        telemetry.addData(">> currentteleOp:",currentteleOp);

        telemetry.addLine("***************");
        telemetry.addData(">> Robot Arms Status", ":");
        telemetry.addData(">> Linear actuator power ", "%.2f", linearActuatorPower);
       // telemetry.addData(">> Linear actuator position (ticks/rev) ", linearActuator.getCurrentPosition());

        telemetry.addLine("***************");
        telemetry.addData(">> Rotate actuator power ", "%.2f", rotateActuatorPower);
        //telemetry.addData(">> Rotate actuator position (ticks/rev) ", rotateActuator.getCurrentPosition());

        telemetry.addLine("***************");
        telemetry.addData(">> Linear viper power ", "%.2f", linearViperPower);
        //telemetry.addData(">> Linear viper position(ticks/rev) ", linearViper.getCurrentPosition());

        telemetry.addLine("***************");
        telemetry.addData(">> Rotate viper power ", "%.2f", rotateViperPower);
       // telemetry.addData(">> Rotate viper position (ticks/rev) ", rotateViper.getCurrentPosition());

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
        telemetry.addData("DISTANCE_SENSOR_CONSTANT ", DISTANCE_SENSOR_CONSTANT);
        telemetry.addData("DISTANCE_SENSOR_MOUNT_HEIGHT ", DISTANCE_SENSOR_MOUNT_HEIGHT);
        telemetry.addData("SHAFT_HEIGHT ", SHAFT_HEIGHT);
        telemetry.addData("TAN_B ", TAN_B);
        telemetry.addData("result ", result);
        telemetry.addData("result1 ", result1);
        telemetry.addData("resultDegrees ", resultDegrees);
        telemetry.update();
    }

    // Function to move the linear actuator to the specified position
    private void moveLinearActuatorToPosition(int targetPosition) {
        linearActuator.setTargetPosition(targetPosition);
        linearActuator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        linearActuator.setPower(LINEAR_ACTUATOR_FULL_POWER ? LINEAR_ACTUATOR_NORMAL_SCALE : LINEAR_ACTUATOR_SLOW_SCALE); // Set power to full (you can adjust this as needed)

        while (opModeIsActive() && linearActuator.isBusy()) {
            sleep(10);
            telemetry.addData(">> Linear actuator power ", "%.2f", linearActuatorPower);
            telemetry.addData(">> Linear actuator ticks ",  linearActuator.getCurrentPosition());

            //telemetry.addData(">> Linear actuator position (ticks/rev) ", linearActuator.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion
        linearActuator.setPower(0);

        if(targetPosition == LINEAR_ACTUATOR_TARGET_POSITION_DOWN){
            linearActuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Reset motor mode
        linearActuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // Function to move the rotate actuator to the specified position
    private void rotateActuatorToPosition(int targetPosition) {
        rotateActuator.setTargetPosition(targetPosition);
        rotateActuator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rotateActuator.setPower(ROTATE_ACTUATOR_FULL_POWER ? ROTATE_ACTUATOR_NORMAL_SCALE : ROTATE_ACTUATOR_SLOW_SCALE); // Set power to full (you can adjust this as needed)

        while (opModeIsActive() && rotateActuator.isBusy()) {
            sleep(10);
            telemetry.addData(">> Rotate actuator power ", "%.2f", rotateActuatorPower);
            telemetry.addData(">> Rotate actuator ticks ",  rotateActuator.getCurrentPosition());

            telemetry.update();
        }
        // Stop all motion
        rotateActuator.setPower(0);

        // Reset motor mode
        rotateActuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void moveRotateViperToPosition(int targetPosition) {
        rotateViper.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rotateViper.setTargetPosition(targetPosition);
        rotateViper.setPower(ROTATE_VIPER_FULL_POWER ? ROTATE_VIPER_NORMAL_SCALE : ROTATE_VIPER_SLOW_SCALE); // Set power to full (you can adjust this as needed)

        while (opModeIsActive() && rotateViper.isBusy()) {
            sleep(10);
            telemetry.addData(">> Rotate viper power ", "%.2f", rotateViperPower);
            telemetry.addData(">> Rotate viper ticks ",  rotateViper.getCurrentPosition());

            //telemetry.addData(">> Linear actuator position (ticks/rev) ", linearActuator.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion
        rotateViper.setPower(0);
        rotateViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // Function to move the viper to the specified position
    private void moveLinearViperToPosition(int targetPosition) {
        linearViper.setTargetPosition(targetPosition);
        linearViper.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        linearViper.setPower(LINEAR_VIPER_FULL_POWER ? LINEAR_VIPER_NORMAL_SCALE : LINEAR_VIPER_SLOW_SCALE); // Set power to full (you can adjust this as needed)

        while (opModeIsActive() && linearViper.isBusy()) {
            sleep(10);
            telemetry.addData(">> Linear viper power ", "%.2f", linearViperPower);
            telemetry.addData(">> Linear viper position (ticks/rev) ", linearViper.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion
        linearViper.setPower(0);

        // Reset motor mode
        linearViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }


    // Function to move the slide within the specified time
    private void moveLinearViperForSpecifiedTime(long targetTime) {
        // Set power to the motor
        linearViper.setPower(LINEAR_VIPER_FULL_POWER ? LINEAR_VIPER_NORMAL_SCALE : LINEAR_VIPER_SLOW_SCALE); // Set power to full (you can adjust this as needed)

        telemetry.addData(">> Moving viper until specified time", targetTime);
        //telemetry.addData(">> Current viper Position", linearViper.getCurrentPosition());
        telemetry.update();

        while (opModeIsActive() && linearViper.isBusy()) {
            idle();
        }

        // Stop the motor
        linearViper.setPower(0);
    }

    private void moveClawToPosition(double targetClawPosition) {
        /*
        double currentClawPosition = clawServo.getPosition();
        double error_Claw = targetClawPosition - currentClawPosition;

        double dt_Claw = timer_Claw.seconds();

        integralClaw += error_Claw * dt_Claw;
        double derivative_Claw = (error_Claw - previousErrorClaw) / dt_Claw;

        double output = kP_Claw * error_Claw + kI_Claw * integralClaw + kD_Claw * derivative_Claw;

        clawServo.setPosition(currentClawPosition + output);

        previousErrorClaw = error_Claw;
        timer_Claw.reset();
*/
        //To move claw without PID enable below code and disable above code
        clawServo.setPosition(targetClawPosition);
        // rightClawServo.setPosition(1 - targetClawPosition); // Assuming right servo is mirrored
    }

    private void calculateViperPID(double targetViperPosition) {
        int currentViperPosition = linearViper.getCurrentPosition();
        double viperError = targetViperPosition - currentViperPosition;
        double dt_Viper = timer_Viper.seconds();

        integralViper += viperError * dt_Viper;
        double viperDerivative = (viperError - previousViperErrorViper) / dt_Viper;

        linearViperPower = kP_Viper * viperError + kI_Viper * integralViper + kD_Viper * viperDerivative;
        previousViperErrorViper = viperError;

        linearViper.setPower(linearViperPower);

        // Reset timer for next cycle
        runtime.reset();
    }

    private void raiseHook() {
        setHookPosition(RAISE_HOOK_UP_POSITION);
    }

    private void retractHook() {
        setHookPosition(RETRACT_HOOK_DOWN_POSITION);
    }

    private void setHookPosition(double position) {
        hangServo.setPosition(position);
        // rightClawServo.setPosition(1 - position); // Assuming right servo is mirrored
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

