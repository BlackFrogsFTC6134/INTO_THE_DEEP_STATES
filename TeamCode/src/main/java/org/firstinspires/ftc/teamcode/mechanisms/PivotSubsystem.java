package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.constants.Constants.GLOBAL_DEBUG_MODE;
import static org.firstinspires.ftc.teamcode.constants.Constants.GLOBAL_SLEEP_TIME;
import static org.firstinspires.ftc.teamcode.constants.PivotConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Subsystem;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;
import org.firstinspires.ftc.teamcode.utils.GamepadStatic;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.utils.PIDFController.FeedForward;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class PivotSubsystem implements Subsystem {
    private DcMotorEx pivotMotor;
    private Servo pivotLeftServo;
    private Servo pivotRightServo;

    private final ElapsedTime runtime                     = new ElapsedTime();

    Telemetry telemetry;
    Telemetry.Item pivotPosition;

    public static double target = 0;
    public static double actualTarget = 0;
    public static double PIVOT_MOTOR_POWER = 0;

    private PIDFController controller;
    private VoltageSensor voltage;

    private final FtcDashboard dashboard;
    LinearOpMode opMode;

    public PivotSubsystem(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;
        this.opMode.telemetry.setAutoClear(false);
        this.dashboard = FtcDashboard.getInstance();
    }

    /**
     * Initializes hardware on the robot. Gets and stores references to the robot configuration and
     * sets motors and servos to their starting positions.
     *
     * @param hardwareMap robot's hardware map
     */
    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            pivotMotor = hardwareMap.get(DcMotorEx.class, pivotMotorName);
            voltage = hardwareMap.voltageSensor.iterator().next();
        }
        catch (IllegalArgumentException e) {
            throw new IllegalArgumentException(getClass().getSimpleName() + " check pivot motor name in Driver Station configuration.");
        }

        try {
            pivotLeftServo = hardwareMap.get(Servo.class, pivotServoLeftName);
            pivotRightServo = hardwareMap.get(Servo.class, pivotServoRightName);
        }
        catch (IllegalArgumentException e) {
            throw new IllegalArgumentException(getClass().getSimpleName() + " check pivot servo names in Driver Station configuration.");
        }

        if(PIVOT_MOTOR_USE_PID){
            controller = new PIDFController(PIVOT_KP, PIVOT_KI, PIVOT_KD, PIVOT_KF);
            controller.setFeedForward(FeedForward.ROTATIONAL);
            controller.setRotationConstants(PIVOT_HIGH_CHAMBER_POS, PIVOT_MOTOR_TICKS_PER_REV);

            //pivotMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0, 0, 0, getMotorVelocityF(PIVOT_MOTOR_RPM / 60 * PIVOT_MOTOR_TICKS_PER_REV)));
            //pivotMotor.setVelocity(0); //TODO: Check if this is correct
        }
        else {
            pivotMotor.setPower(0); //TODO: Check if this is correct
            pivotMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            pivotMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        pivotMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        pivotLeftServo.setDirection(Servo.Direction.FORWARD);
        pivotRightServo.setDirection(Servo.Direction.FORWARD);

        pivotLeftServo.scaleRange(MIN_LEFT_PIVOT_POS, MAX_LEFT_PIVOT_POS); //TODO: Check if this is correct
        pivotRightServo.scaleRange(MIN_RIGHT_PIVOT_POS, MAX_RIGHT_PIVOT_POS); //TODO: Check if this is correct

        //pivotDown(); //Uncomment if needed and comment pivotBack() line
        pivotServosBack();
        runtime.startTimeNanoseconds();

        if (GLOBAL_DEBUG_MODE || PIVOT_DEBUG_MODE) {
            opMode.telemetry.addLine("PivotSubsystem initialized");
            pivotPosition = telemetry.addData("PivotSubsystem position: ", "left (%5.2f), right (%5.2f)", pivotLeftServo.getPosition(), pivotRightServo.getPosition());
            opMode.sleep(GLOBAL_SLEEP_TIME);
        }
        runtime.startTimeNanoseconds();

        //pivotDown(); //Uncomment if needed and comment pivotBack() line
        pivotServosBack();
    }

    /**
     * Manages gamepad inputs and their corresponding mechanism response
     * The PivotDev or TeleOp code calling this Class has to override this method by providing the gamepad number
     * User need to map the gamepad buttons using Controls class
     * @param gamepad2 = gamepad2
     */
    @Override
    public void update(Gamepad gamepad2, Telemetry telemetry) {
        if (GamepadStatic.isButtonPressed(gamepad2, Controls.PIVOT_DOWN)) {
            pivotServosDown();
        } else if (GamepadStatic.isButtonPressed(gamepad2, Controls.PIVOT_CENTER)) {
            pivotServosBack();
        }
    }

    /**
     * setPivotMotorPosition() method is used to set the pivot motor to a specific targetMotorPosition_Inch.
     * This method first resets the pivot motor encoder and sets the targetMotorPosition_Inch.
     * It then waits until the pivot motor reaches the targetPivotMotorPosition or until the timeout is reached.
     * In ClawConstants class, you can change the PIVOT_MOTOR_TIMEOUT_MS.
     * CLAW_TIMEOUT_MS is the time in milli seconds that the method wait to reach to the target targetMotorPosition_Inch.
     * It is important to set motor power to zero in the end of this method to prevent motor from getting hot.
     * @param targetMotorPosition_Inch = -1.0 to 1.0
     */
    public void setPivotMotorPosition(int targetMotorPosition_Inch) {
        pivotMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        int currentMotorPosition = pivotMotor.getCurrentPosition();

        pivotMotor.setTargetPosition(targetMotorPosition_Inch);
        runtime.reset();
        pivotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        pivotMotor.setPower(PivotConstants.PIVOT_MOTOR_POWER);

        while (pivotMotor.isBusy() && runtime.seconds() <= PIVOT_TIMEOUT_MS) {
            if (GLOBAL_DEBUG_MODE || PIVOT_DEBUG_MODE) {
                opMode.telemetry.addData("PivotSubsystem motor position: ", pivotMotor.getCurrentPosition());
                opMode.telemetry.update();
                opMode.sleep(GLOBAL_SLEEP_TIME);
            }
            opMode.idle();
        }

        pivotMotor.setPower(0); //Stop the motor. Important step to prevent motor from getting hot.
    }

    /**
     * pivotDown() method is used to set the pivot motor to the down position.
     */
    public void pivotServosDown() {
        runtime.reset();
        double ExecutionTime;

        double pivotLeftServoStartPosition = pivotLeftServo.getPosition();
        double pivotRightServoStartPosition = pivotRightServo.getPosition();

        while(isNotAtTargetPosition(PIVOT_DOWN_POS) && runtime.seconds() <= PIVOT_TIMEOUT_MS) {
            pivotLeftServo.setPosition(PIVOT_DOWN_POS);
            pivotRightServo.setPosition(PIVOT_DOWN_POS);
            pivotPosition.setValue(String.valueOf(pivotLeftServo.getPosition()), pivotRightServo.getPosition());
            opMode.idle();
        }
        ExecutionTime = runtime.seconds();

        if (GLOBAL_DEBUG_MODE || PIVOT_DEBUG_MODE) {
            opMode.telemetry.addLine("Pivot servos in down position");
            pivotPosition.setValue(String.valueOf(pivotLeftServo.getPosition()), pivotRightServo.getPosition());
            opMode.telemetry.update();
            opMode.sleep(GLOBAL_SLEEP_TIME);

            // Send telemetry to dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Left Servo Start Position: ", pivotLeftServoStartPosition);
            packet.put("Right Servo Start Position: ", pivotRightServoStartPosition);

            packet.put("PIVOT_DOWN_POS: ", PIVOT_DOWN_POS);

            packet.put("Left Servo Down Position: ", pivotLeftServo.getPosition());
            packet.put("Right Servo Down Position: ", pivotRightServo.getPosition());
            packet.put("Pivot down in [secs]: ", ExecutionTime);

            dashboard.sendTelemetryPacket(packet);
        }
    }

    /**
     * pivotBack() method is used to set the pivot motor to the back position.
     */
    public void pivotServosBack() {
        runtime.reset();

        double pivotLeftServoStartPosition = pivotLeftServo.getPosition();
        double pivotRightServoStartPosition = pivotRightServo.getPosition();
        double ExecutionTime = 0;

        while(isNotAtTargetPosition(PIVOT_CENTER_POS) && runtime.seconds() <= PIVOT_TIMEOUT_MS) {
            pivotLeftServo.setPosition(PIVOT_CENTER_POS);
            pivotRightServo.setPosition(PIVOT_CENTER_POS);
            pivotPosition.setValue(String.valueOf(pivotLeftServo.getPosition()), pivotRightServo.getPosition());
            opMode.idle();
        }
        ExecutionTime = runtime.seconds();

        if (GLOBAL_DEBUG_MODE || PIVOT_DEBUG_MODE) {
            opMode.telemetry.addLine("Pivot servos in center position");
            pivotPosition.setValue(String.valueOf(pivotLeftServo.getPosition()), pivotRightServo.getPosition());
            opMode.telemetry.update();
            opMode.sleep(GLOBAL_SLEEP_TIME);

            // Send telemetry to dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Left Servo Start Position: ", pivotLeftServoStartPosition);
            packet.put("Right Servo Start Position: ", pivotRightServoStartPosition);

            packet.put("PIVOT_CENTER_POS: ", PIVOT_CENTER_POS);

            packet.put("Left Servo Center Position: ", pivotLeftServo.getPosition());
            packet.put("Right Servo Center Position: ", pivotRightServo.getPosition());
            packet.put("Pivot Back in [secs]: ", ExecutionTime);

            dashboard.sendTelemetryPacket(packet);
        }
    }

    /**
     * setPosition() method is used to set the pivot motor to a specific targetPivotServoPosition_Inch.
     * @param targetPivotServoPosition_Inch = -1.0 to 1.0
     */
    public void setPosition(double targetPivotServoPosition_Inch) {
        runtime.reset();

        double pivotLeftServoStartPosition = pivotLeftServo.getPosition();
        double pivotRightServoStartPosition = pivotRightServo.getPosition();
        double ExecutionTime;

        while(isNotAtTargetPosition(targetPivotServoPosition_Inch) && runtime.seconds() <= PIVOT_TIMEOUT_MS){
            pivotLeftServo.setPosition(targetPivotServoPosition_Inch);
            pivotRightServo.setPosition(targetPivotServoPosition_Inch);
            pivotPosition.setValue(String.valueOf(pivotLeftServo.getPosition()), pivotRightServo.getPosition());
            opMode.telemetry.update();
            opMode.idle();
        }

        ExecutionTime = runtime.seconds();

        if(GLOBAL_DEBUG_MODE || PIVOT_DEBUG_MODE){
            opMode.telemetry.addLine("Pivot set targetPivotServoPosition_Inch");
            pivotPosition.setValue(String.valueOf(pivotLeftServo.getPosition()), pivotRightServo.getPosition());
            opMode.telemetry.update();
            opMode.sleep(GLOBAL_SLEEP_TIME);

            // Send telemetry to dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Left Servo Start Position: ", pivotLeftServoStartPosition);
            packet.put("Right Servo Start Position: ", pivotRightServoStartPosition);

            packet.put("PIVOT_CENTER_POS: ", PIVOT_CENTER_POS);

            packet.put("Left Servo Back Position: ", pivotLeftServo.getPosition());
            packet.put("Right Servo Back Position: ", pivotRightServo.getPosition());
            packet.put("Pivot Back in [secs]: ", ExecutionTime);

            dashboard.sendTelemetryPacket(packet);
        }
    }

    /**
     * isNotAtTargetPosition() method is used to check if the pivot motor is not at the targetPivotServoPosition_Inch.
     * @param targetPivotServoPosition_Inch = -1.0 to 1.0
     * @return true if the pivot motor is not at the targetPivotServoPosition_Inch, false otherwise
     */
    public boolean isNotAtTargetPosition(double targetPivotServoPosition_Inch) {
        return !(Math.abs(pivotLeftServo.getPosition() - targetPivotServoPosition_Inch) <= PIVOT_POSITION_TOLERANCE) &&
                !(Math.abs(pivotRightServo.getPosition() - targetPivotServoPosition_Inch) <= PIVOT_POSITION_TOLERANCE);
    }

    /**
     * telemetry() method is used to print the pivot motor position to the telemetry.
     * @param telemetry = telemetry object from the opmode
     */
    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Pivot position");
        pivotPosition.setValue(String.valueOf(pivotLeftServo.getPosition()), pivotRightServo.getPosition());
        opMode.telemetry.update();
        opMode.sleep(GLOBAL_SLEEP_TIME);
    }

    /**
     * stopSubsystem()
     *
     * @param telemetry = null
     */
    @Override
    public void stopSubsystem(Telemetry telemetry) throws InterruptedException {

    }
}