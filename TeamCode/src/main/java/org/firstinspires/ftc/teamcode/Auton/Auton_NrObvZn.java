package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "Auton_NrObvZn(Blocks to Java)")
public class Auton_NrObvZn extends LinearOpMode {

    // private RevBlinkinLedDriver Blinkin12;
    private DcMotor lftch0;
    private DcMotor lrrech1;
    private DcMotor rftch2;
    private DcMotor rrrch3;
    private Servo clawreh0;
    private Servo clawleh1;
    private Servo pivotreh2;
    private Servo pivotleh3;
    private Servo ClimberHookEh4;
    private DcMotor extensioneh0;
    private DcMotor _2extensionEh2;
    private DcMotor pivoteh1;
    private DistanceSensor LeftFrontDistanceSensor;
    private DistanceSensor RightFrontDistanceSensor;
    private DistanceSensor LeftRearDistanceSensor;
    private DistanceSensor RightRearDistanceSensor;

    private static final double TICKS_PER_REV = 384.5; // For GoBILDA 5202 motors
    private static final double FRICTION_COMPENSATION = 19.5; // For GoBILDA 5202 motors 19.5

    private static final double WHEEL_DIAMETER_INCHES = 4.09449;
    private static final double GEAR_RATIO = 1;
    private static final double TICKS_PER_INCH = ((TICKS_PER_REV - FRICTION_COMPENSATION) * GEAR_RATIO / (WHEEL_DIAMETER_INCHES * Math.PI));
    public int IntialPivotPos;
    public double LeftFrontDistance;
    public double RightFrontDistance;
    public double LeftRearDistance;
    public double RightRearDistance;

    /**
     * This is the Auton code using the Mecanum wheel Encoders for Drive train navigation
     * */

    @Override
    public void runOpMode() {
        ElapsedTime runtime;

        lftch0 = hardwareMap.get(DcMotor.class, "lftch0");
        lrrech1 = hardwareMap.get(DcMotor.class, "lrrech1");
        rftch2 = hardwareMap.get(DcMotor.class, "rftch2");
        rrrch3 = hardwareMap.get(DcMotor.class, "rrrch3");
        clawreh0 = hardwareMap.get(Servo.class, "clawreh0");
        clawleh1 = hardwareMap.get(Servo.class, "clawleh1");
        pivotreh2 = hardwareMap.get(Servo.class, "pivotreh2");
        pivotleh3 = hardwareMap.get(Servo.class, "pivotleh3");
        ClimberHookEh4 = hardwareMap.get(Servo.class, "ClimberHookEh4");
        extensioneh0 = hardwareMap.get(DcMotor.class, "extensioneh0");
        _2extensionEh2 = hardwareMap.get(DcMotor.class, "2extensionEh2");
        pivoteh1 = hardwareMap.get(DcMotor.class, "pivoteh1");
        LeftFrontDistanceSensor = hardwareMap.get(DistanceSensor.class, "LeftFrontDistanceCH12Cbus3-3");
        RightFrontDistanceSensor = hardwareMap.get(DistanceSensor.class, "RightFrontDistanceEH12Cbus1-1");
        LeftRearDistanceSensor = hardwareMap.get(DistanceSensor.class, "LeftRearDistanceEH12Cbus2-2");
        RightRearDistanceSensor = hardwareMap.get(DistanceSensor.class, "RightRearDistanceCH12Cbus2-2");

        runtime = new ElapsedTime();

        /* Set the Drive train Motor directions */
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
        lrrech1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lftch0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rftch2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrrch3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivoteh1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        runtime.reset();
        if (opModeIsActive()) {

            clawleh1.setPosition(1);
            clawreh0.setPosition(1);

            /*To raise the Pivot to upper chamber  */
            IntialPivotPos = pivoteh1.getCurrentPosition();
            pivoteh1.setTargetPosition((IntialPivotPos) + (2900));
            pivoteh1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivoteh1.setPower(0.9);
            DriveForward(0.3, 0.3, 0.3, 0.3, 23);
            sleep(2000);

            /*Moving the Wrist down to hang */
            pivotreh2.setPosition(1);
            pivotleh3.setPosition(1);

            sleep(1000);

            /* Operating the Pivot to score the Specimen to the high chamber */
            pivoteh1.setTargetPosition((IntialPivotPos) + (2500));
            pivoteh1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivoteh1.setPower(0.9);

            sleep(300);

            clawleh1.setPosition(0);
            clawreh0.setPosition(0);

            sleep(1000);

            DriveRearward(0.3, 0.3, 0.3, 0.3, 7);
            sleep(500);

            clawleh1.setPosition(0.5);
            clawreh0.setPosition(0.5);

            sleep(1000);

            /*Moving the Wrist up to set preset position */
            pivotreh2.setPosition(0.26);
            pivotleh3.setPosition(0.26);

            sleep(1000);

            /*Navigating to plow the 1st sample to the observation zone */
            StrafeRight(0.4, 0.4, 0.4, 0.4, 24);
            sleep(500);
            TurnLeft(0.1, 0.1, 0.1, 0.1, 5);
            sleep(500);
            telemetry.addData("Pivotposition", pivoteh1.getCurrentPosition());
            telemetry.update();

            /*****This block is used for testing *****/
          /*  pivoteh1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pivoteh1.setTargetPosition((IntialPivotPos)+(2500));
            pivoteh1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivoteh1.setPower(0.9);*/

            sleep(500);
            DriveForward(0.3, 0.3, 0.3, 0.3, 25);
            sleep(500);
            StrafeRight(0.4, 0.4, 0.4, 0.4, 10);
            sleep(400);
            TurnLeft(0.1, 0.1, 0.1, 0.1, 2);
            sleep(500);
            DriveRearward(0.3, 0.3, 0.3, 0.3, 27);
            sleep(2000);
            if ((LeftRearDistanceSensor.getDistance(DistanceUnit.INCH) >= 12)) {
                LeftRearDistance = LeftRearDistanceSensor.getDistance(DistanceUnit.INCH) - 12;

                DriveRearward(0.3, 0.3, 0.3, 0.3, (int) LeftRearDistance);
                telemetry.addData("LeftDistance:", LeftRearDistanceSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
            } else if (LeftRearDistanceSensor.getDistance(DistanceUnit.INCH) <= 11) {

                LeftRearDistance = 11 - LeftRearDistanceSensor.getDistance(DistanceUnit.INCH);
                DriveForward(0.3, 0.3, 0.3, 0.3, (int) LeftRearDistance);
                telemetry.addData("LeftDistance:", LeftRearDistanceSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
            if ((RightRearDistanceSensor.getDistance(DistanceUnit.INCH) >= 12)) {
                RightRearDistance = RightRearDistanceSensor.getDistance(DistanceUnit.INCH) - 12;

                DriveRearward(0.3, 0.3, 0.3, 0.3, (int) RightRearDistance);
                telemetry.addData("RightDistance:", RightRearDistanceSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
            } else if (RightRearDistanceSensor.getDistance(DistanceUnit.INCH) <= 11) {

                RightRearDistance = 11 - RightRearDistanceSensor.getDistance(DistanceUnit.INCH);
                DriveForward(0.3, 0.3, 0.3, 0.3, (int) RightRearDistance);
                telemetry.addData("RightDistance:", RightRearDistanceSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }

            /*Plow the  2nd sample to the observation zone */

            StrafeLeft(0.4, 0.4, 0.4, 0.4, 2);
            sleep(500);
            DriveForward(0.3, 0.3, 0.3, 0.3, 30);
            sleep(500);

            StrafeRight(0.4, 0.4, 0.4, 0.4, 8);
            sleep(500);
            //        TurnRight(0.1,0.1,0.1,0.1,2);
            sleep(500);
            // DriveRearward(0.3, 0.3, 0.3, 0.3, 30);

            DriveRearward(0.3, 0.3, 0.3, 0.3, 29);


            //*** Steps to take the specimen form the wall and hang it on the chamber ***/
            TurnLeft(0.1, 0.1, 0.1, 0.1, 90);
            sleep(2000);
            if ((LeftFrontDistanceSensor.getDistance(DistanceUnit.INCH) >= 12)) {
                LeftFrontDistance = LeftFrontDistanceSensor.getDistance(DistanceUnit.INCH) - 12;

                DriveForward(0.3, 0.3, 0.3, 0.3, (int) LeftFrontDistance);
                telemetry.addData("LeftDistance:", LeftFrontDistanceSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
            } else if (LeftFrontDistanceSensor.getDistance(DistanceUnit.INCH) <= 11) {

                LeftFrontDistance = 11 - LeftFrontDistanceSensor.getDistance(DistanceUnit.INCH);
                DriveRearward(0.3, 0.3, 0.3, 0.3, (int) LeftFrontDistance);
                telemetry.addData("LeftDistance:", LeftFrontDistanceSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
            if ((RightFrontDistanceSensor.getDistance(DistanceUnit.INCH) >= 12)) {
                RightFrontDistance = RightFrontDistanceSensor.getDistance(DistanceUnit.INCH) - 12;

                DriveForward(0.3, 0.3, 0.3, 0.3, (int) RightFrontDistance);
                telemetry.addData("RightDistance:", RightFrontDistanceSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
            } else if (RightFrontDistanceSensor.getDistance(DistanceUnit.INCH) <= 10) {

                RightFrontDistance = 10 - RightFrontDistanceSensor.getDistance(DistanceUnit.INCH);
                DriveRearward(0.3, 0.3, 0.3, 0.3, (int) RightFrontDistance);
                telemetry.addData("RightDistance:", RightFrontDistanceSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }

            /*TO pick up the specimen form the wall */
            clawleh1.setPosition(0);
            clawreh0.setPosition(0);
//          IntialPivotPos=pivoteh1.getCurrentPosition();
            sleep(500);
            pivoteh1.setTargetPosition(1002);
            //     pivoteh1.setTargetPosition((IntialPivotPos)+576);
            pivoteh1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivoteh1.setPower(0.8);
            sleep(500);
            clawleh1.setPosition(1);
            clawreh0.setPosition(1);
            sleep(500);
            pivoteh1.setTargetPosition(1452);
            pivoteh1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivoteh1.setPower(0.8);
        }
    }

    /**
     * Describe this function...
     */
    private void DriveForward(double RightFrontPWR, double RightRearPWR, double LeftFrontPWR, double LeftRearPWR, int ForwardDistance) {
        //   Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));
        lrrech1.setDirection(DcMotor.Direction.REVERSE);
        lftch0.setDirection(DcMotor.Direction.REVERSE);
        rrrch3.setDirection(DcMotor.Direction.FORWARD);
        rftch2.setDirection(DcMotor.Direction.FORWARD);
        lftch0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lrrech1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rftch2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrrch3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // 38.4615385 Ticks Per Inch
        lftch0.setTargetPosition((int) (38.4615385 * ForwardDistance));
        lrrech1.setTargetPosition((int) (38.4615385 * ForwardDistance));
        rftch2.setTargetPosition((int) (38.4615385 * ForwardDistance));
        rrrch3.setTargetPosition((int) (38.4615385 * ForwardDistance));
        // lftch0.setTargetPosition((int) (TICKS_PER_INCH * ForwardDistance));
        // lrrech1.setTargetPosition((int) (TICKS_PER_INCH * ForwardDistance));
        // rftch2.setTargetPosition((int) (TICKS_PER_INCH * ForwardDistance));
        // rrrch3.setTargetPosition((int) (TICKS_PER_INCH * ForwardDistance));


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
    private void DriveRearward(double RightFrontPWR, double RightRearPWR, double LeftFrontPWR, double LeftRearPWR, int RearwardDistance) {
        // Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1805));
        lftch0.setDirection(DcMotor.Direction.FORWARD);
        lrrech1.setDirection(DcMotor.Direction.FORWARD);
        rftch2.setDirection(DcMotor.Direction.REVERSE);
        rrrch3.setDirection(DcMotor.Direction.REVERSE);
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
    private void TurnRight(double RightFrontPWR, double RightRearPWR, double LeftFrontPWR, double LeftRearPWR, double RightTurnDegrees) {
        //    Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1825));
        // ToPivotRightSideMotorValuesReversed
        lftch0.setDirection(DcMotor.Direction.REVERSE);
        lrrech1.setDirection(DcMotor.Direction.REVERSE);
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
    private void StrafeRight(double RightFrontPWR, double RightRearPWR, double LeftFrontPWR, double LeftRearPWR, int StrafeDistance) {

        //  Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));
        lftch0.setDirection(DcMotor.Direction.REVERSE);
        lrrech1.setDirection(DcMotor.Direction.FORWARD);
        rftch2.setDirection(DcMotor.Direction.REVERSE);
        rrrch3.setDirection(DcMotor.Direction.FORWARD);
        lftch0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lrrech1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rftch2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrrch3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // 38.4615385 Ticks Per Inch
        lftch0.setTargetPosition((int) (38.4615385 * StrafeDistance));
        lrrech1.setTargetPosition((int) (38.4615385 * StrafeDistance));
        rftch2.setTargetPosition((int) (38.4615385 * StrafeDistance));
        rrrch3.setTargetPosition((int) (38.4615385 * StrafeDistance));
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
    private void StrafeLeft(double RightFrontPWR, double RightRearPWR, double LeftFrontPWR, double LeftRearPWR, int StrafeDistance) {

        //  Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));
        lftch0.setDirection(DcMotor.Direction.FORWARD);
        lrrech1.setDirection(DcMotor.Direction.REVERSE);
        rftch2.setDirection(DcMotor.Direction.FORWARD);
        rrrch3.setDirection(DcMotor.Direction.REVERSE);
        lftch0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lrrech1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rftch2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrrch3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // 38.4615385 Ticks Per Inch
        lftch0.setTargetPosition((int) (38.4615385 * StrafeDistance));
        lrrech1.setTargetPosition((int) (38.4615385 * StrafeDistance));
        rftch2.setTargetPosition((int) (38.4615385 * StrafeDistance));
        rrrch3.setTargetPosition((int) (38.4615385 * StrafeDistance));
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
    private void TurnLeft(double RightFrontPWR, double RightRearPWR, double LeftFrontPWR, double LeftRearPWR, double LeftTurnDegrees) {
        //  Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1845));
        // ToPivotRightSideMotorValuesReversed
        lftch0.setDirection(DcMotor.Direction.FORWARD);
        lrrech1.setDirection(DcMotor.Direction.FORWARD);
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
}