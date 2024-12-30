package org.firstinspires.ftc.teamcode.SubSystems;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain implements DrivetrainSubsystem {
    public DcMotorEx leftFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightBack = null;

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
    double PowerIncrease = 0.8;
    double NormalDrivetrainPower = 0.65;

    private static final double TICKS_PER_REV = 384.5; // For GoBILDA 5202 motors
    private static final double FRICTION_COMPENSATION = 19.5; // For GoBILDA 5202 motors 19.5

    private static final double WHEEL_DIAMETER_INCHES = 4.09449;
    private static final double GEAR_RATIO = 1;

    private static final double TICKS_PER_INCH = ((TICKS_PER_REV - FRICTION_COMPENSATION) * GEAR_RATIO / (WHEEL_DIAMETER_INCHES * Math.PI));

    private LED led;
    private DistanceSensor distanceSensor1;
    private DistanceSensor distanceSensor2;

    private double LeftFrontDistanceSensor;
    private double RightFrontDistanceSensor;

    int sleepTime = 500;
    Gamepad gamepad1, gamepad2;

    private final Telemetry telemetry;

    public Drivetrain(HardwareMap hardwareMap, LED led, DistanceSensor distanceSensor1, DistanceSensor distanceSensor2, Telemetry telemetry) {
        this.led = led;
        this.distanceSensor1 = distanceSensor1;
        this.distanceSensor2 = distanceSensor2;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException {
        led = new LED(hardwareMap, telemetry);
        led.initialize(hardwareMap, telemetry);

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        leftFront = hardwareMap.get(DcMotorEx.class, "lfrtch0");
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

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);

        telemetry.addData("DrivetrainSubsystem initialized: ", "successfully");
        telemetry.update();
    }

    public void drive(double axial, double lateral, double yaw) {
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        /*led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
        sleep(sleepTime); */

        leftFrontPower = (axial + lateral + yaw)*NormalDrivetrainPower;
        rightFrontPower = ((axial - lateral) - yaw)*NormalDrivetrainPower;
        leftBackPower = ((axial - lateral) + yaw)*NormalDrivetrainPower;
        rightBackPower = ((axial + lateral) - yaw)*NormalDrivetrainPower;


        telemetry.addData("axial 1", axial);
        telemetry.addData("lateral", lateral);
        telemetry.addData("yaw", yaw);

        telemetry.addData("leftFrontPower", leftFrontPower);
        telemetry.addData("leftBackPower", leftBackPower);
        telemetry.addData("rightFrontPower", rightFrontPower);
        telemetry.addData("rightBackPower", rightBackPower);
        telemetry.update();

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
       /*
        if (max > 1) {
            leftFrontPower = leftFrontPower / max;
            rightFrontPower = rightFrontPower / max;
            leftBackPower = leftBackPower / max;
            rightBackPower = rightBackPower / max;
        } */

        setDrivePowers(leftFrontPower, leftBackPower,rightFrontPower, rightBackPower);
/*
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
        sleep(sleepTime);*/
    }

    public void setDrivePowers(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower ) {
        // Send calculated power to wheels.
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    public void increaseDrivePower() {
        axial = -(gamepad1.left_stick_y * PowerIncrease);
        lateral = -(gamepad1.left_stick_x * PowerIncrease);
        yaw = gamepad1.right_stick_x * PowerIncrease;
        drive(axial, lateral, yaw);

        telemetry.addData("increaseDrivePower", "");
        telemetry.addData("axial 2", axial);
        telemetry.addData("lateral", lateral);
        telemetry.addData("yaw", yaw);
        telemetry.update();
    }

    public void decreaseDrivePower() {
        axial = -(gamepad1.left_stick_y * PowerReduction);
        lateral = -(gamepad1.left_stick_x  * PowerReduction);
        yaw = gamepad1.right_stick_x * PowerReduction;
        drive(axial, lateral, yaw);

        telemetry.addData("decreaseDrivePower", "");
        telemetry.addData("axial 3", axial);
        telemetry.addData("lateral", lateral);
        telemetry.addData("yaw", yaw);
        telemetry.update();
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

        setPower(NormalDrivetrainPower);
  //      led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);

        while (isBusy()) {
            telemetry.addData("Moving forward, current position: ", leftFront.getCurrentPosition());
        }

        //Initialize the drivetrain to what it was set during initialization
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
    }

    public void DriveForward(double inches) {
        driveSetTargetPosition(inches);
    }

    public void DriveBackward(double inches) throws InterruptedException {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        driveSetTargetPosition(inches);
    }

    public void strafeDriveLeft(double inches) throws InterruptedException {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));

        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        driveSetTargetPosition(inches);
    }

    public void strafeDriveRight(double inches) throws InterruptedException {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        driveSetTargetPosition(inches);
    }

    public void turnDriveLeft(double inches) throws InterruptedException {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));

        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        driveSetTargetPosition(inches);
    }

    public void turnDriveRight(double inches) throws InterruptedException {
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

    public void DriveRearward(int RearwardDistance) throws InterruptedException {
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
        setDrivePowers(leftFrontPower, leftBackPower,rightFrontPower, rightBackPower);
    }

    public void TurnRight(int RightTurnDegrees) throws InterruptedException {
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
        setDrivePowers(leftFrontPower, leftBackPower,rightFrontPower, rightBackPower);
    }

    public void LeftTurn(int LeftTurnDegrees) throws InterruptedException {
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
        leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        setDrivePowers(leftFrontPower, leftBackPower,rightFrontPower, rightBackPower);
    }

    @Override
    public void update(Telemetry telemetry) {
        // Implement any periodic checks or updates

    }

    @Override
    public void stopSubsystem(Telemetry telemetry) throws InterruptedException {
        resetPower();
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void testMotorDirections() throws InterruptedException {
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

        setDrivePowers(leftFrontPower, leftBackPower,rightFrontPower, rightBackPower);
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT);
    }
}

