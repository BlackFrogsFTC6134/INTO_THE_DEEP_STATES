package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "_6gr1 (Blocks to Java)")
public class _6gr1 extends LinearOpMode {

    private DcMotor lrrech1;
    private DcMotor lftch0;
    private DcMotor rrrch3;
    private DcMotor rftch2;
    private Servo clawreh0;
    private Servo clawleh1;
    private Servo pivotreh2;
    private Servo pivotleh3;
    private Servo ClimberHookEh4;
    private DcMotor extensioneh0;
    private DcMotor _2extensionEh2;
    private DcMotor pivoteh1;
    private RevBlinkinLedDriver Blinkin12;
    private DistanceSensor LeftFrontDistanceCH12Cbus33;
    private DistanceSensor RightFrontDistanceEH12Cbus11;

    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;

    /**
     * This OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
     * This code will work with either a Mecanum-Drive or an X-Drive train.
     * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
     *
     * Also note that it is critical to set the correct rotation direction for each motor. See details below.
     *
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * Each motion axis is controlled by one Joystick axis.
     *
     * 1) Axial -- Driving forward and backward -- Left-joystick Forward/Backward
     * 2) Lateral -- Strafing right and left -- Left-joystick Right and Left
     * 3) Yaw -- Rotating Clockwise and counter clockwise -- Right-joystick Right and Left
     *
     * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
     * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
     * the direction of all 4 motors (see code below).
     */
    @Override
    public void runOpMode() {
        ElapsedTime runtime;
        double PowerReduction;
        double PowerIncrease;
        double axial;
        double lateral;
        double yaw;
        double max;

        lrrech1 = hardwareMap.get(DcMotor.class, "lrrech1");
        lftch0 = hardwareMap.get(DcMotor.class, "lftch0");
        rrrch3 = hardwareMap.get(DcMotor.class, "rrrch3");
        rftch2 = hardwareMap.get(DcMotor.class, "rftch2");
        clawreh0 = hardwareMap.get(Servo.class, "clawreh0");
        clawleh1 = hardwareMap.get(Servo.class, "clawleh1");
        pivotreh2 = hardwareMap.get(Servo.class, "pivotreh2");
        pivotleh3 = hardwareMap.get(Servo.class, "pivotleh3");
        ClimberHookEh4 = hardwareMap.get(Servo.class, "ClimberHookEh4");
        extensioneh0 = hardwareMap.get(DcMotor.class, "extensioneh0");
        _2extensionEh2 = hardwareMap.get(DcMotor.class, "2extensionEh2");
        pivoteh1 = hardwareMap.get(DcMotor.class, "pivoteh1");
        Blinkin12 = hardwareMap.get(RevBlinkinLedDriver.class, "Blinkin12");
        LeftFrontDistanceCH12Cbus33 = hardwareMap.get(DistanceSensor.class, "LeftFrontDistanceCH12Cbus3-3");
        RightFrontDistanceEH12Cbus11 = hardwareMap.get(DistanceSensor.class, "RightFrontDistanceEH12Cbus1-1");

        runtime = new ElapsedTime();
        // ########################################################################################
        // !!! IMPORTANT Drive Information. Test your motor directions. !!!!!
        // ########################################################################################
        //
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot
        // (the wheels turn the same direction as the motor shaft).
        //
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction. So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        //
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward.
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        // <--- Click blue icon to see important note re. testing motor directions.
        lrrech1.setDirection(DcMotor.Direction.REVERSE);
        lftch0.setDirection(DcMotor.Direction.REVERSE);
        rrrch3.setDirection(DcMotor.Direction.FORWARD);
        rftch2.setDirection(DcMotor.Direction.FORWARD);
        // Servo
        clawreh0.setDirection(Servo.Direction.FORWARD);
        clawleh1.setDirection(Servo.Direction.REVERSE);
        pivotreh2.setDirection(Servo.Direction.FORWARD);
        pivotleh3.setDirection(Servo.Direction.REVERSE);
        pivotreh2.setDirection(Servo.Direction.FORWARD);
        pivotleh3.setDirection(Servo.Direction.REVERSE);
        ClimberHookEh4.setDirection(Servo.Direction.FORWARD);
        // Expansion Motors
        extensioneh0.setDirection(DcMotor.Direction.FORWARD);
        _2extensionEh2.setDirection(DcMotor.Direction.FORWARD);
        pivoteh1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensioneh0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _2extensionEh2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivoteh1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensioneh0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
            if (gamepad2.right_stick_button) {
                ClimberHookEh4.setPosition(0);
            }
            if (gamepad2.left_stick_button) {
                ClimberHookEh4.setPosition(0.1);
            }
            // Viper Slide  Full Up and Mid way
            // 400= 1 rotation Full up is 6 rotations
            if (gamepad2.start && gamepad2.dpad_up) {
                Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1865));
                extensioneh0.setTargetPosition(400 * 6);
                extensioneh0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extensioneh0.setPower(1);
                sleep(4500);
                Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1845));
                sleep(1000);
            } else if (gamepad2.start && gamepad2.dpad_right) {
                Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1795));
                extensioneh0.setTargetPosition(400 * 3);
                extensioneh0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extensioneh0.setPower(1);
                sleep(2000);
                Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1845));
                sleep(2000);
            } else if (gamepad2.start && gamepad2.dpad_down) {
                // Full down = 0
                Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1805));
                extensioneh0.setTargetPosition(0);
                extensioneh0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extensioneh0.setPower(1);
                sleep(500);
                Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1845));
                sleep(500);
            } else if (gamepad2.start && gamepad2.dpad_left) {
                // Reset 0 (home set)
                Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1815));
                extensioneh0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sleep(500);
            } else {
                extensioneh0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extensioneh0.setPower(-gamepad2.left_stick_y);
            }
            if (gamepad2.back && gamepad2.dpad_up) {
                Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_RAINBOW_PALETTE);
                pivoteh1.setTargetPosition((int) (400 * 19.2));
                pivoteh1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivoteh1.setPower(1);
                sleep(4000);
                Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1845));
                sleep(500);
            } else if (gamepad2.back && gamepad2.dpad_right) {
                Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_PARTY_PALETTE);
                pivoteh1.setTargetPosition(400 * 10);
                pivoteh1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivoteh1.setPower(1);
                sleep(3000);
                Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1845));
                sleep(500);
            } else if (gamepad2.back && gamepad2.dpad_down) {
                // Full down = 0
                Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_PARTY_PALETTE);
                pivoteh1.setTargetPosition(0);
                pivoteh1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivoteh1.setPower(1);
                sleep(3000);
                Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1845));
                sleep(500);
            } else if (gamepad2.back && gamepad2.dpad_left) {
                // Reset 0 (home set)
                pivoteh1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else {
                pivoteh1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pivoteh1.setPower(-gamepad2.right_stick_y);
            }
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Note: pushing stick forward gives negative value
            // Setting power reduction number Multiplying by (0.8) reduces the Maximum speed of the robot for more control
            PowerReduction = 0.4;
            // Setting power Increase number Multiplying by (0.2) Increases the Maximum speed of the robot for more control
            PowerIncrease = 0.6;
            if (gamepad1.left_bumper) {
                axial = -(gamepad1.left_stick_y * PowerIncrease);
                lateral = -(gamepad1.left_stick_x * PowerIncrease);
                yaw = gamepad1.right_stick_x * PowerIncrease;
            } else {
                axial = -(gamepad1.left_stick_y * PowerReduction);
                lateral = -(gamepad1.left_stick_x * PowerReduction);
                yaw = gamepad1.right_stick_x * PowerReduction;
            }
            // Directional Flipping Section
            if (gamepad1.a) {
                // When Holding A  button ALONE rear of robot becomes front
                axial = gamepad1.left_stick_y * PowerReduction;
                lateral = gamepad1.left_stick_x * PowerReduction;
                yaw = gamepad1.right_stick_x * PowerReduction;
            }
            if (gamepad1.b) {
                // When Holding B button ALONE Right Side of robot becomes front
                axial = -(gamepad1.left_stick_y * PowerReduction);
                lateral = gamepad1.left_stick_x * PowerReduction;
                yaw = gamepad1.right_stick_x * PowerReduction;
            }
            if (gamepad1.x) {
                // When Holding X button ALONE Left Side of robot becomes front
                axial = -(gamepad1.left_stick_y * PowerReduction);
                lateral = -(gamepad1.left_stick_x * PowerReduction);
                yaw = gamepad1.right_stick_x * PowerReduction;
            }
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = (axial - lateral) - yaw;
            leftBackPower = (axial - lateral) + yaw;
            rightBackPower = (axial + lateral) - yaw;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
            if (max > 1) {
                leftFrontPower = leftFrontPower / max;
                rightFrontPower = rightFrontPower / max;
                leftBackPower = leftBackPower / max;
                rightBackPower = rightBackPower / max;
            }
            // Send calculated power to wheels.
            lrrech1.setPower(leftFrontPower);
            rrrch3.setPower(rightFrontPower);
            lftch0.setPower(leftBackPower);
            rftch2.setPower(rightBackPower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", JavaUtil.formatNumber(leftFrontPower, 4, 2) + ", " + JavaUtil.formatNumber(rightFrontPower, 4, 2));
            telemetry.addData("Back  left/Right", JavaUtil.formatNumber(leftBackPower, 4, 2) + ", " + JavaUtil.formatNumber(rightBackPower, 4, 2));
            telemetry.update();
            // SPEC OPS Pressing A Alone opens and closes claw
            if (gamepad2.a) {
                clawreh0.setPosition(0);
                clawleh1.setPosition(0);
            }
            // SPEC OPS Pressing B Alone opens and closes claw
            if (gamepad2.b) {
                clawreh0.setPosition(0.5);
                clawleh1.setPosition(0.5);
            }
            // SPEC OPS Pressing RightBumper ALONE Moves Pivot UP
            // SPEC OPS Pressing LeftBumper ALONE Moves Pivot Down
            if (gamepad2.right_bumper) {
                pivoteh1.setPower(1);
            } else if (gamepad2.left_bumper) {
                pivoteh1.setPower(-1);
            } else {
                pivoteh1.setPower(0);
            }
            // SPEC OPS Pressing Right Bumper ALONE Moves Elevator Up
            // SPEC OPS Pressing Left Bumper ALONE Moves Elevator Down
            // SPEC OPS Pressing X ALONE Moves Wrist Up
            if (gamepad2.x) {
                pivotreh2.setPosition(0.26);
                pivotleh3.setPosition(0.26);
            }
            // SPEC OPS Pressing Y ALONE Moves Wrist Down
            if (gamepad2.y) {
                pivotreh2.setPosition(0.1);
                pivotleh3.setPosition(0.1);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void DriveForward(int RightFrontPWR, int RightRearPWR, int LeftFrontPWR, int LeftRearPWR, int ForwardDistance) {
        Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));
        lftch0.setDirection(DcMotor.Direction.REVERSE);
        lrrech1.setDirection(DcMotor.Direction.REVERSE);
        rftch2.setDirection(DcMotor.Direction.REVERSE);
        rrrch3.setDirection(DcMotor.Direction.REVERSE);
        lftch0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lrrech1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rftch2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrrch3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // 38.4615385 Ticks Per Inch
        lftch0.setTargetPosition((int) (38.4615385 * ForwardDistance));
        lrrech1.setTargetPosition((int) (38.4615385 * ForwardDistance));
        rftch2.setTargetPosition((int) (38.4615385 * ForwardDistance));
        rrrch3.setTargetPosition((int) (38.4615385 * ForwardDistance));
        lftch0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lrrech1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rftch2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rrrch3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lftch0.setPower(LeftFrontPWR);
        lrrech1.setPower(LeftRearPWR);
        rftch2.setPower(RightFrontPWR);
        rrrch3.setPower(RightRearPWR);
        while (opModeIsActive() && lftch0.isBusy() && lrrech1.isBusy() && rftch2.isBusy() && rrrch3.isBusy()) {
            idle();
        }
    }

    /**
     * Describe this function...
     */
    private void DriveRearward(int RightFrontPWR, int RightRearPWR, int LeftFrontPWR, int LeftRearPWR, int RearwardDistance) {
        Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1805));
        lftch0.setDirection(DcMotor.Direction.FORWARD);
        lrrech1.setDirection(DcMotor.Direction.FORWARD);
        rftch2.setDirection(DcMotor.Direction.FORWARD);
        rrrch3.setDirection(DcMotor.Direction.FORWARD);
        lftch0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lrrech1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rftch2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrrch3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // 38.4615385 Ticks Per Inch
        lftch0.setTargetPosition((int) (38.4615385 * RearwardDistance));
        lrrech1.setTargetPosition((int) (38.4615385 * RearwardDistance));
        rftch2.setTargetPosition((int) (38.4615385 * RearwardDistance));
        rrrch3.setTargetPosition((int) (38.4615385 * RearwardDistance));
        lftch0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lrrech1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rftch2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rrrch3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lftch0.setPower(LeftFrontPWR);
        lrrech1.setPower(LeftRearPWR);
        rftch2.setPower(RightFrontPWR);
        rrrch3.setPower(RightRearPWR);
        while (opModeIsActive() && lftch0.isBusy() && lrrech1.isBusy() && rftch2.isBusy() && rrrch3.isBusy()) {
            idle();
        }
    }

    /**
     * Describe this function...
     */
    private void TurnRight(double RightFrontPWR, double RightRearPWR, double LeftFrontPWR, double LeftRearPWR, int RightTurnDegrees) {
        Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1825));
        // ToPivotRightSideMotorValuesReversed
        lftch0.setDirection(DcMotor.Direction.FORWARD);
        lrrech1.setDirection(DcMotor.Direction.FORWARD);
        rftch2.setDirection(DcMotor.Direction.REVERSE);
        rrrch3.setDirection(DcMotor.Direction.REVERSE);
        lftch0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lrrech1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rftch2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrrch3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // 38.4615385 Ticks Per Inch multiplied by .25 equals 1 degree of rotation
        // Adjust degrees value up or down to increase or decrease number of degrees witnessed in testing
        lftch0.setTargetPosition((int) (38.4615385 * 0.25555556 * RightTurnDegrees));
        lrrech1.setTargetPosition((int) (38.4615385 * 0.25555556 * RightTurnDegrees));
        rftch2.setTargetPosition((int) (38.4615385 * 0.25555556 * RightTurnDegrees));
        rrrch3.setTargetPosition((int) (38.4615385 * 0.25555556 * RightTurnDegrees));
        lftch0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lrrech1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rftch2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rrrch3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lftch0.setPower(LeftFrontPWR);
        lrrech1.setPower(LeftRearPWR);
        rftch2.setPower(RightFrontPWR);
        rrrch3.setPower(RightRearPWR);
        while (opModeIsActive() && lftch0.isBusy() && lrrech1.isBusy() && rftch2.isBusy() && rrrch3.isBusy()) {
            idle();
        }
    }

    /**
     * Describe this function...
     */
    private void LeftTurn(double RightFrontPWR, double RightRearPWR, double LeftFrontPWR, double LeftRearPWR, int LeftTurnDegrees) {
        Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1845));
        // ToPivotRightSideMotorValuesReversed
        lftch0.setDirection(DcMotor.Direction.REVERSE);
        lrrech1.setDirection(DcMotor.Direction.REVERSE);
        rftch2.setDirection(DcMotor.Direction.FORWARD);
        rrrch3.setDirection(DcMotor.Direction.FORWARD);
        lftch0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lrrech1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rftch2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrrch3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // 38.4615385 Ticks Per Inch multiplied by .25 equals 1 degree of rotation
        // Adjust degrees value up or down to increase or decrease number of degrees witnessed in testing
        lftch0.setTargetPosition((int) (38.4615385 * 0.25555556 * LeftTurnDegrees));
        lrrech1.setTargetPosition((int) (38.4615385 * 0.25555556 * LeftTurnDegrees));
        rftch2.setTargetPosition((int) (38.4615385 * 0.25555556 * LeftTurnDegrees));
        rrrch3.setTargetPosition((int) (38.4615385 * 0.25555556 * LeftTurnDegrees));
        lftch0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lrrech1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rftch2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rrrch3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lftch0.setPower(LeftFrontPWR);
        lrrech1.setPower(LeftRearPWR);
        rftch2.setPower(RightFrontPWR);
        rrrch3.setPower(RightRearPWR);
        while (opModeIsActive() && lftch0.isBusy() && lrrech1.isBusy() && rftch2.isBusy() && rrrch3.isBusy()) {
            idle();
        }
    }

    /**
     * Describe this function...
     */
    private void RaisePivotToHighPosition(int PivotFullUp, int PivotPower, int FunctionTimingSeconds) {
        pivoteh1.setTargetPosition(PivotFullUp);
        pivoteh1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _2extensionEh2.setPower(PivotPower);
        sleep(FunctionTimingSeconds * 1000);
    }

    /**
     * Describe this function...
     */
    private void RaisePivotToMiddlePosition(int PivotMiddlePosition, int PivotPower, int FunctionTimingSeconds) {
        pivoteh1.setTargetPosition(PivotMiddlePosition);
        pivoteh1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _2extensionEh2.setPower(PivotPower);
        sleep(FunctionTimingSeconds * 1000);
    }

    /**
     * Describe this function...
     */
    private void RaisePivotToLowPositio(int PivotLowPosition, int PivotPower, int FunctionTimingSeconds) {
        pivoteh1.setTargetPosition(PivotLowPosition);
        pivoteh1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _2extensionEh2.setPower(PivotPower);
        sleep(FunctionTimingSeconds * 1000);
    }

    /**
     * Describe this function...
     */
    private void WristLowPosition(double WristLowerValue) {
        pivotleh3.setPosition(WristLowerValue);
        pivotreh2.setPosition(WristLowerValue);
    }

    /**
     * Describe this function...
     */
    private void WristMidPosition(double WristMiddleValue) {
        pivotleh3.setPosition(WristMiddleValue);
        pivotreh2.setPosition(WristMiddleValue);
    }

    /**
     * Describe this function...
     */
    private void WristHighPosition(double WristHighValue) {
        pivotleh3.setPosition(WristHighValue);
        pivotreh2.setPosition(WristHighValue);
    }

    /**
     * Describe this function...
     */
    private void ClawClosedPosition(int ClawClosedPosition2) {
        clawleh1.setPosition(ClawClosedPosition2);
        clawreh0.setPosition(ClawClosedPosition2);
    }

    /**
     * Describe this function...
     */
    private void ClawOpenPosition(double ClawOpenPosition2) {
        clawleh1.setPosition(ClawOpenPosition2);
        clawreh0.setPosition(ClawOpenPosition2);
    }

    /**
     * Describe this function...
     */
    private void RaiseElevatorToMiddlePosition(double ElevatorMiddlePosition, int ElevatorPower, int FunctionTimingSeconds) {
        // 3 rotations is mid condition since full extension is 6
    }

    /**
     * Describe this function...
     */
    private void RaiseElevatorToHighPosition(double ElevatorFullUp, int ElevatorPower, int FunctionTimingSeconds) {
        extensioneh0.setTargetPosition((int) ElevatorFullUp);
        extensioneh0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensioneh0.setPower(ElevatorPower);
        while (extensioneh0.isBusy()) {
            _2extensionEh2.setPower(ElevatorPower);
        }
        sleep(FunctionTimingSeconds * 1000);
    }

    /**
     * This function is used to test your motor directions.
     *
     * Each button should make the corresponding motor run FORWARD.
     *
     *   1) First get all the motors to take to correct positions on the robot
     *      by adjusting your Robot Configuration if necessary.
     *
     *   2) Then make sure they run in the correct direction by modifying the
     *      the setDirection() calls above.
     */
    private void testMotorDirections() {
        leftFrontPower = gamepad1.x ? 1 : 0;
        leftBackPower = gamepad1.a ? 1 : 0;
        rightFrontPower = gamepad1.y ? 1 : 0;
        rightBackPower = gamepad1.b ? 1 : 0;
    }

    /**
     * Describe this function...
     */
    private void RaiseElevatorToLowPosition(int ElevatorLowPosition, int ElevatorPower, int FunctionTimingSeconds) {
        for (int count7 = 0; count7 < 1; count7++) {
            extensioneh0.setTargetPosition(ElevatorLowPosition);
            extensioneh0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extensioneh0.setPower(ElevatorPower);
            sleep(FunctionTimingSeconds * 1000);
        }
        extensioneh0.setPower(0);
    }
}