package org.firstinspires.ftc.teamcode.SubSystems;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.drive.OpModeEx;

public class DrivetrainSubsystem extends LinearOpMode implements Subsystem {
    public DcMotorEx leftFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightBack = null;

    private OpModeEx opMode;

    double leftFrontPower = 0;
    double leftBackPower = 0;
    double rightFrontPower = 0;
    double rightBackPower = 0;

    double axial = 0;
    double lateral = 0;
    double yaw = 0;
    double max = 0;

    // Setting power reduction number Multiplying by (0.8) reduces the Maximum speed of the robot for more control
    double PowerReduction = 0.5;
    // Setting power Increase number Multiplying by (0.2) Increases the Maximum speed of the robot for more control
    double PowerIncrease = 0.9;
    double NormalDrivetrainPower = 0.8;

    private static final double TICKS_PER_REV = 384.5; // For GoBILDA 5202 motors
    private static final double FRICTION_COMPENSATION = 19.5; // For GoBILDA 5202 motors 19.5

    private static final double WHEEL_DIAMETER_INCHES = 4.09449;
    private static final double GEAR_RATIO = 1;

    private static final double TICKS_PER_INCH = ((TICKS_PER_REV - FRICTION_COMPENSATION) * GEAR_RATIO / (WHEEL_DIAMETER_INCHES * Math.PI));

    private LEDSubsystem led;
    private DistanceSensorSubsystem distanceSensor1;
    private DistanceSensorSubsystem distanceSensor2;

    private double LeftFrontDistanceSensor;
    private double RightFrontDistanceSensor;

    public DrivetrainSubsystem(LEDSubsystem led, DistanceSensorSubsystem distanceSensor1, DistanceSensorSubsystem distanceSensor2) {
        this.led = led;
        this.distanceSensor1 = distanceSensor1;
        this.distanceSensor2 = distanceSensor2;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {

        led = new LEDSubsystem();
        led.initialize(hardwareMap);

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        leftFront = hardwareMap.get(DcMotorEx.class, "lftch0");
        leftBack = hardwareMap.get(DcMotorEx.class, "lrrech1");
        rightFront = hardwareMap.get(DcMotorEx.class, "rftch2");
        rightBack = hardwareMap.get(DcMotorEx.class, "rrrch3");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
/*
        telemetry.addData("DrivetrainSubsystem initialized: ", "successfully");
        telemetry.update();*/
    }

    public void drive(HardwareMap hardwareMap, double axial, double lateral, double yaw) {
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
        sleep(250);
        leftFrontPower = (axial + lateral + yaw)*NormalDrivetrainPower;
        rightFrontPower = ((axial - lateral) - yaw)*NormalDrivetrainPower;
        leftBackPower = ((axial - lateral) + yaw)*NormalDrivetrainPower;
        rightBackPower = ((axial + lateral) - yaw)*NormalDrivetrainPower;
        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
        if (max > 1) {
            leftFrontPower = leftFrontPower / max;
            rightFrontPower = rightFrontPower / max;
            leftBackPower = leftBackPower / max;
            rightBackPower = rightBackPower / max;
        }
        setDrivePowers(hardwareMap, leftFrontPower, leftBackPower,rightFrontPower, rightBackPower);

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
        sleep(250);
    }

    public void setDrivePowers(HardwareMap hardwareMap, double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower ) {
        // Send calculated power to wheels.
        this.leftFront.setPower(leftFrontPower);
        this.leftBack.setPower(leftBackPower);
        this.rightFront.setPower(rightFrontPower);
        this.rightBack.setPower(rightBackPower);

        telemetry.addData("leftFrontPower", leftFrontPower);
        telemetry.addData("leftBackPower", leftBackPower);
        telemetry.addData("rightFrontPower", rightFrontPower);
        telemetry.addData("rightBackPower", rightBackPower);
    }

    public void increaseDrivePower(HardwareMap hardwareMap, GamepadState gamepadState) {
        axial = -(gamepadState.leftStickY1 * PowerIncrease);
        lateral = -(gamepadState.leftStickX1 * PowerIncrease);
        yaw = gamepadState.rightStickX1 * PowerIncrease;
        drive(hardwareMap, axial, lateral, yaw);
/*
        telemetry.addData("increaseDrivePower", "");
        telemetry.addData("axial", axial);
        telemetry.addData("lateral", lateral);
        telemetry.addData("yaw", yaw);
        telemetry.update();*/
    }

    public void decreaseDrivePower(HardwareMap hardwareMap, GamepadState gamepadState) {
        axial = -(gamepadState.leftStickY1 * PowerReduction);
        lateral = -(gamepadState.leftStickX1 * PowerReduction);
        yaw = gamepadState.rightStickX1 * PowerReduction;
        drive(hardwareMap, axial, lateral, yaw);
/*
        telemetry.addData("decreaseDrivePower", "");
        telemetry.addData("axial", axial);
        telemetry.addData("lateral", lateral);
        telemetry.addData("yaw", yaw);
        telemetry.update();*/
    }

    public void makeRearOfRobotFront(HardwareMap hardwareMap, GamepadState gamepadState) {
        // When Holding A  button ALONE rear of robot becomes front
        axial = gamepadState.leftStickY1 * PowerReduction;
        lateral = gamepadState.leftStickX1 * PowerReduction;
        yaw = gamepadState.rightStickX1 * PowerReduction;
        drive(hardwareMap, axial, lateral, yaw);
    }

    public void makeLeftOfRobotFront(HardwareMap hardwareMap, GamepadState gamepadState) {
        axial = -(gamepadState.leftStickY1 * PowerReduction);
        lateral = -(gamepadState.leftStickX1* PowerReduction);
        yaw = gamepadState.rightStickX1 * PowerReduction;
        drive(hardwareMap, axial, lateral, yaw);
    }

    public void makeRightOfRobotFront(HardwareMap hardwareMap, GamepadState gamepadState) {
        axial = -(gamepadState.leftStickY1 * PowerReduction);
        lateral = gamepadState.leftStickX1 * PowerReduction;
        yaw = gamepadState.rightStickX1 * PowerReduction;
        drive(hardwareMap, axial, lateral, yaw);
    }

    public void driveSetTargetPosition(double inches) {

        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        int ticks = (int)(inches * TICKS_PER_INCH );

        leftFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightFront.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);

        setRunMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sleep(100);
        double drivetrainPower = 0.3;
        setPower(drivetrainPower);

        //Initialize the drivetrain to what it was set during initialization
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);

        while (opModeIsActive() && isBusy()) {
            //sleep(100);
        }
    }

    public void DriveForward(double inches) {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));
        driveSetTargetPosition(inches);
    }

    public void DriveBackward(double inches) {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        driveSetTargetPosition(inches);
    }

    public void strafeDriveLeft(double inches) {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));

        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        driveSetTargetPosition(inches);
    }

    public void strafeDriveRight(double inches) {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        driveSetTargetPosition(inches);
    }

    public void turnDriveLeft(double inches) {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));

        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        driveSetTargetPosition(inches);
    }

    public void turnDriveRight(double inches) {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        driveSetTargetPosition(inches);
    }

    public boolean isBusy() {
        return leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy();
    }

    public double getCurrentDistance() {
        int averageTicks = (leftFront.getCurrentPosition() + leftBack.getCurrentPosition() +
                rightFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 4;
        return averageTicks / TICKS_PER_INCH;
    }

    private void setRunMode(DcMotorEx.RunMode runMode) {
        leftFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightFront.setMode(runMode);
        rightBack.setMode(runMode);
    }

    private void setPower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    private void resetPower() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void DriveRearward(HardwareMap hardwareMap, int RearwardDistance) {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1805));
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // 38.4615385 Ticks Per Inch
        leftFront.setTargetPosition((int) (38.4615385 * RearwardDistance));
        leftBack.setTargetPosition((int) (38.4615385 * RearwardDistance));
        rightFront.setTargetPosition((int) (38.4615385 * RearwardDistance));
        rightBack.setTargetPosition((int) (38.4615385 * RearwardDistance));
        leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        setDrivePowers(hardwareMap, leftFrontPower, leftBackPower,rightFrontPower, rightBackPower);
    }

    public void TurnRight(HardwareMap hardwareMap, int RightTurnDegrees) {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1825));
        // ToPivotRightSideMotorValuesReversed
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // 38.4615385 Ticks Per Inch multiplied by .25 equals 1 degree of rotation
        // Adjust degrees value up or down to increase or decrease number of degrees witnessed in testing
        leftFront.setTargetPosition((int) (38.4615385 * 0.25555556 * RightTurnDegrees));
        leftBack.setTargetPosition((int) (38.4615385 * 0.25555556 * RightTurnDegrees));
        rightFront.setTargetPosition((int) (38.4615385 * 0.25555556 * RightTurnDegrees));
        rightBack.setTargetPosition((int) (38.4615385 * 0.25555556 * RightTurnDegrees));
        leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        setDrivePowers(hardwareMap, leftFrontPower, leftBackPower,rightFrontPower, rightBackPower);
    }

    public void LeftTurn(HardwareMap hardwareMap, int LeftTurnDegrees) {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1845));
        // ToPivotRightSideMotorValuesReversed
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // 38.4615385 Ticks Per Inch multiplied by .25 equals 1 degree of rotation
        // Adjust degrees value up or down to increase or decrease number of degrees witnessed in testing
        leftFront.setTargetPosition((int) (38.4615385 * 0.25555556 * LeftTurnDegrees));
        leftBack.setTargetPosition((int) (38.4615385 * 0.25555556 * LeftTurnDegrees));
        rightFront.setTargetPosition((int) (38.4615385 * 0.25555556 * LeftTurnDegrees));
        rightBack.setTargetPosition((int) (38.4615385 * 0.25555556 * LeftTurnDegrees));
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePowers(hardwareMap, leftFrontPower, leftBackPower,rightFrontPower, rightBackPower);
    }

    @Override
    public void update(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        // Implement any periodic checks or updates
        /*
        telemetry.addData("leftFrontPower", leftFrontPower);
        telemetry.addData("leftBackPower", leftBackPower);
        telemetry.addData("rightFrontPower", rightFrontPower);
        telemetry.addData("rightBackPower", rightBackPower);

        telemetry.addData("axial", axial);
        telemetry.addData("lateral", lateral);
        telemetry.addData("yaw", yaw);

        telemetry.update();*/
    }

    @Override
    public void stopSubsystem(HardwareMap hardwareMap) {
        resetPower();
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void testMotorDirections() {
        /**
         * This function is used to test your motor directions. Each button should make the
         * corresponding motor run FORWARD. 1) First get all the motors to take to correct
         * positions on the robot by adjusting your Robot Configuration if necessary. 2) Then make
         * sure they run in the correct direction by modifying the the setDirection() calls above.
         */

        leftFrontPower = gamepad1.x ? 1 : 0;
        leftBackPower = gamepad1.a ? 1 : 0;
        rightFrontPower = gamepad1.y ? 1 : 0;
        rightBackPower = gamepad1.b ? 1 : 0;

        setDrivePowers(hardwareMap, leftFrontPower, leftBackPower,rightFrontPower, rightBackPower);
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}

