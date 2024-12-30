package org.firstinspires.ftc.teamcode.RobotHardware.Subsystems;

import static org.firstinspires.ftc.teamcode.constants.Constants.GLOBAL_DEBUG_MODE;
import static org.firstinspires.ftc.teamcode.constants.Constants.GLOBAL_SLEEP_TIME;
import static org.firstinspires.ftc.teamcode.constants.PivotConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.utils.PIDFController;

/**
 * Author: Kavish Karia
 * Date create: 12/28/2024
 * FTC Team: 6134 - Black Frog
 * Comments:
 */
public class Pivot implements Subsystem{
    DcMotorEx pivotMotor;
    private final ElapsedTime runtime = new ElapsedTime();

    Telemetry telemetry;
    Telemetry.Item pivotPosition;

    public FtcDashboard dashboard;
    //public LinearOpMode subsystemOpMode;

    int pivotMotorCurrentPosition = 0;
    int pivotMotorInitialPosition = 0;
    double ExecutionTime = 0;

    public static double target = 0;
    public static double actualTarget = 0;

    private static PIDFController controller;
    private VoltageSensor voltage;

    public Pivot() {
        //subsystemOpMode = opMode;
        //subsystemOpMode.telemetry.setAutoClear(false);
        dashboard = FtcDashboard.getInstance();
    }

    /**
     * initialize(HardwareMap hardwareMap,Telemetry telemetry) function is used to initialize the subsystem
     * @param hardwareMap = hardwareMap
     * @param  telemetry = telemetry
     */
    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            pivotMotor = hardwareMap.get(DcMotorEx.class, pivotMotorName);
        } catch (IllegalArgumentException e) {
            throw new IllegalArgumentException(getClass().getSimpleName() + " check Pivot motor name in Driver Station configuration.");
        }

       pivotMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // motor brakes on zero power
       pivotMotor.setDirection(DcMotorEx.Direction.FORWARD); // sets direction

      if (GLOBAL_DEBUG_MODE || PIVOT_DEBUG_MODE) {
          //telemetry.addLine("PivotSubsystem initialized");
          pivotPosition = telemetry.addData("Pivot motor position: ", "(%5.2f)", pivotMotor.getCurrentPosition());
          //subsystemOpMode.sleep(GLOBAL_SLEEP_TIME);
      }

        if(PIVOT_MOTOR_USE_PID){
            controller = new PIDFController(PIVOT_KP, PIVOT_KI, PIVOT_KD, PIVOT_KF);
            controller.setFeedForward(PIDFController.FeedForward.ROTATIONAL);
            controller.setRotationConstants(PIVOT_HIGH_CHAMBER_POS, PIVOT_MOTOR_TICKS_PER_REV);
        }
        else {
            //pivotMotor.setPower(0); //TODO: Check if this is correct
            pivotMotor.setVelocity(PIVOT_MOTOR_VELOCITY);
            pivotMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            pivotMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        pivotMotorInitialPosition = pivotMotor.getCurrentPosition();
        INIT_POS = pivotMotorInitialPosition;
        runtime.startTimeNanoseconds();
        setPivotMotorPosition(PIVOT_CENTER_POS);
    }

    /**
     * @param telemetry = telemetry
     */
    @Override
    public void sendTelemetry(Telemetry telemetry) {
        telemetry.addData("Pivot Position", pivotMotor.getCurrentPosition());
    }

    /**
     * periodic(Gamepad gamepad) function is used to map gamepad inputs to certain actions
     *
     * @param gamepad1  = gamepad1
     * @param gamepad2  = gamepad2
     * @param telemetry = telemetry
     */
    @Override
    public void periodic(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        if (gamepad2.a) {
            //setPivotMotorPosition(60);
        }
    }

    /**
     * stopSubsystem(Telemetry telemetry) applies zero power to the subsystem
     * @param telemetry = telemetry
     */
    @Override
    public void stopSubsystem(Telemetry telemetry) {
        pivotMotor.setPower(0);
    }


    /**
     * setPivotMotorPosition() method is used to set the pivot motor to a specific target_Motor_Position_Ticks.
     * This method first resets the pivot motor encoder and sets the target_Motor_Position_Ticks.
     * It then waits until the pivot motor reaches the targetPivotMotorPosition or until the timeout is reached.
     * In ClawConstants class, you can change the PIVOT_MOTOR_TIMEOUT_MS.
     * CLAW_TIMEOUT_MS is the time in milli seconds that the method wait to reach to the target target_Motor_Position_Ticks.
     * It is important to set motor power to zero in the end of this method to prevent motor from getting hot.
     * @param target_Motor_Position_Ticks = 0 to 2200
     */
    public void setPivotMotorPosition(int target_Motor_Position_Ticks) {
        pivotMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotorCurrentPosition = pivotMotor.getCurrentPosition();

        pivotMotor.setTargetPosition((target_Motor_Position_Ticks));
        runtime.reset();
        pivotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        pivotMotor.setVelocity(PIVOT_MOTOR_VELOCITY);
        //pivotMotor.setPower(PIVOT_MOTOR_POWER);

        while (pivotMotor.isBusy() && runtime.seconds() <= PIVOT_TIMEOUT_MS) {
            if (GLOBAL_DEBUG_MODE || PIVOT_DEBUG_MODE) {
                telemetry.addData("PivotSubsystem motor position: ", pivotMotor.getCurrentPosition());
                telemetry.update();
                //subsystemOpMode.sleep(GLOBAL_SLEEP_TIME);
            }
            //subsystemOpMode.idle();
        }

        pivotMotor.setPower(0); //Stop the motor. Important step to prevent motor from getting hot.
    }
}
