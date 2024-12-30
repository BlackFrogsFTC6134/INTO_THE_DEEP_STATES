package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.constants.ClawConstants.*;
import static org.firstinspires.ftc.teamcode.constants.Constants.GLOBAL_DEBUG_MODE;
import static org.firstinspires.ftc.teamcode.constants.Constants.GLOBAL_SLEEP_TIME;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.SubSystems.Subsystem;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.utils.GamepadStatic;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Mechanism;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.Locale;

/**
 * Author: Kavish Karia
 * Date create: 12/13/2024
 * FTC Team: 6134 - Black Frog
 * Comments: This class is used to control the claw mechanism. This class extends the Mechanism class.
 * For ClawSubsystem mechanism ensure that the clawServoLeft & clawServoRight are defined as Servo objects and not as CRServo objects.
 * Also, ensure that in constructor, the HardwareMap used Servo class and not CRServo class
 * To change the constants, go to the ClawConstants class.
 * It contains methods to grab and release the claw.
 * Pre-requisites: Learn how to program Servo position - <a href="https://www.youtube.com/watch?v=lQDTbY3c374">...</a>
 * Next steps: Set the MIN_CLAW_POS & MAX_CLAW_POS in ClawConstants class.
 */

public class ClawSubsystem implements Subsystem {
    HardwareMap hardwareMap;
    private Servo clawServoLeft = null;
    private Servo clawServoRight = null;

    private final ElapsedTime runtime = new ElapsedTime();

    Telemetry telemetry;
    Telemetry.Item clawPosition;

    public FtcDashboard dashboard;
    public LinearOpMode opMode;

    public ClawSubsystem(LinearOpMode opMode) {
        this.opMode = opMode;
        this.opMode.telemetry.setAutoClear(false);
        this.dashboard = FtcDashboard.getInstance();
    }

    /**
     * grabClaw() method is used to grab the claw.
     * This method checks if the claw is at the target position. If not, it moves the claw to the target position.
     * In ClawConstants class, you can change the target position, CLAW_CLOSE_POSITION & CLAW_TIMEOUT_MS.
     * CLAW_TIMEOUT_MS is the time in milli seconds that the method wait to reach to the target position.
     */
    public void grabClaw() {
        runtime.reset();
        while(isNotAtTargetPosition(CLAW_CLOSE_POSITION) && runtime.seconds() < CLAW_TIMEOUT_MS){
            clawServoLeft.setPosition(CLAW_CLOSE_POSITION);
            clawServoRight.setPosition(CLAW_CLOSE_POSITION);
            clawPosition.setValue(String.valueOf(clawServoLeft.getPosition()), clawServoRight.getPosition());
            opMode.idle();
        }

        if(GLOBAL_DEBUG_MODE || CLAW_DEBUG_MODE){
            opMode.telemetry.addLine("ClawSubsystem grabbed");
            clawPosition.setValue(String.valueOf(clawServoLeft.getPosition()), clawServoRight.getPosition());
            opMode.telemetry.update();
            opMode.sleep(GLOBAL_SLEEP_TIME);
        }
    }

    /**
     * releaseClaw() method is used to release the claw.
     * This method checks if the claw is at the target position. If not, it moves the claw to the target position.
     * In ClawConstants class, you can change the target position, CLAW_OPEN_POSITION & CLAW_TIMEOUT_MS.
     * CLAW_TIMEOUT_MS is the time in milli seconds that the method wait to reach to the target position.
     */
    public void releaseClaw() {
        runtime.reset();
        while(isNotAtTargetPosition(CLAW_OPEN_POSITION) && runtime.seconds() < CLAW_TIMEOUT_MS){
            clawServoLeft.setPosition(CLAW_OPEN_POSITION);
            clawServoRight.setPosition(CLAW_OPEN_POSITION);
            clawPosition.setValue(String.valueOf(clawServoLeft.getPosition()), clawServoRight.getPosition());
            opMode.idle();
        }
        if(GLOBAL_DEBUG_MODE || CLAW_DEBUG_MODE){
            opMode.telemetry.addLine("ClawSubsystem released");
            clawPosition.setValue(String.valueOf(clawServoLeft.getPosition()), clawServoRight.getPosition());
            opMode.telemetry.update();
            opMode.sleep(GLOBAL_SLEEP_TIME);
        }
    }

    /**
     * setPosition() method is used to set the claw to a specific targetClawPosition.
     * This method checks if the claw is at the target targetClawPosition. If not, it moves the claw to the target targetClawPosition.
     * In ClawConstants class, you can change the target targetClawPosition, & CLAW_TIMEOUT_MS.
     * CLAW_TIMEOUT_MS is the time in milli seconds that the method wait to reach to the target targetClawPosition.
     * @param targetClawPosition = -1.0 to 1.0
     */
    public void setPosition(double targetClawPosition) {
        runtime.reset();
        while(isNotAtTargetPosition(targetClawPosition) && runtime.seconds() < CLAW_TIMEOUT_MS){
            clawServoLeft.setPosition(targetClawPosition);
            clawServoRight.setPosition(targetClawPosition);
            clawPosition.setValue(String.valueOf(clawServoLeft.getPosition()), clawServoRight.getPosition());
            opMode.idle();
        }

        if(GLOBAL_DEBUG_MODE || CLAW_DEBUG_MODE){
            opMode.telemetry.addLine("ClawSubsystem set targetClawPosition");
            clawPosition.setValue(String.valueOf(clawServoLeft.getPosition()), clawServoRight.getPosition());
            opMode.telemetry.update();
            opMode.sleep(GLOBAL_SLEEP_TIME);
        }
    }

    /**
     * isNotAtTargetPosition() method is used to check if the claw is at the target position.
     * @param targetPosition = -1.0 to 1.0
     * @return = true if the claw is not at the target position, false otherwise
     */
    public boolean isNotAtTargetPosition(double targetPosition) {
        return !(Math.abs(clawServoLeft.getPosition() - targetPosition) <= CLAW_POSITION_TOLERANCE) &&
                !(Math.abs(clawServoRight.getPosition() - targetPosition) <= CLAW_POSITION_TOLERANCE);
    }

    /**
     * update() method is used to control the claw mechanism using ClawDev or TeleOp code..
     * Map the controller button for grabClaw & releaseClaw using Controls class.
     * @param gamepad2 = gamepad2
     */
    @Override
    public void update(Gamepad gamepad2, Telemetry telemetry) {
        if (GamepadStatic.isButtonPressed(gamepad2, Controls.GRAB)) {
            grabClaw();
        } else if (GamepadStatic.isButtonPressed(gamepad2, Controls.RELEASE)) {
            releaseClaw();
        }
    }

    /**
     * Manages all telemetry data to driver phone or FTC Dashboard
     * Where all telemetry.addData() calls should go
     *
     * @param telemetry = null
     */
    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Left claw position: ", clawServoLeft.getPosition());
        telemetry.addData("Right claw position: ", clawServoRight.getPosition());
    }

    /**
     * toString() method is used to display the claw position on the Driver Station.
     * @return = clawServoLeft.getPosition() and clawServoRight.getPosition()
     */
    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US, "left (%.2f), right (%.2f)", clawServoLeft.getPosition(), clawServoRight.getPosition());
    }

    /**
     * @param hardwareMap = null
     * @param telemetry = null
     */
    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException {
        try {
            clawServoLeft = hardwareMap.get(Servo.class, clawServoLeftName);
            clawServoRight = hardwareMap.get(Servo.class, clawServoRightName);
        }
        catch (IllegalArgumentException e) {
            throw new IllegalArgumentException(getClass().getSimpleName() + " check ClawSubsystem servo names in Driver Station configuration.");
        }

        clawServoLeft.setDirection(Servo.Direction.REVERSE);
        telemetry.addLine("this is the motor direction");
        clawServoRight.setDirection(Servo.Direction.FORWARD);

        if (MIN_CLAW_POS >= MAX_CLAW_POS)
        {
            throw new IllegalArgumentException("MAX_CLAW_POS must be greater than MIN_CLAW_POS.");
        } else {
            clawServoLeft.scaleRange(MIN_CLAW_POS, MAX_CLAW_POS); //TODO: Check if this is correct
        }

        if(GLOBAL_DEBUG_MODE || CLAW_DEBUG_MODE){
            opMode.telemetry.addLine("ClawSubsystem initialized");
            clawPosition = telemetry.addData("ClawSubsystem position: ", "left (%5.2f), right (%5.2f)", clawServoLeft.getPosition(), clawServoRight.getPosition());
            opMode.sleep(GLOBAL_SLEEP_TIME);
        }

        runtime.startTimeNanoseconds();

        //grabClaw();         // Uncomment if needed and comment releaseClaw() line
        releaseClaw();
    }

    /**
     * @param telemetry = null
     */
    @Override
    public void stopSubsystem(Telemetry telemetry) throws InterruptedException{
        telemetry.addLine("ClawSubsystem position");
        clawPosition.setValue(String.valueOf(clawServoLeft.getPosition()), clawServoRight.getPosition());
        opMode.telemetry.update();
        opMode.sleep(GLOBAL_SLEEP_TIME);
    }
}