package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.ViperSlideSubsystem;

@Disabled
@TeleOp (name = "Viper_Subsystem")

public class ViperSlide extends LinearOpMode implements ViperSlideSubsystem {
    @Override
    public void runOpMode() throws InterruptedException {
        class ViperSlideSubsystem {
            private DcMotorEx slideMotor;
            private PIDController pidController;
            private ElapsedTime timer;
            double slideMotor_Target = 0;

            // PID coefficients
            private static final double kP = 0.1;
            private static final double kI = 0.0;
            private static final double kD = 0.05;


/*
            // Constructor
            public ViperSlideSubsystem(HardwareMap hardwareMap) {
                slideMotor = hardwareMap.get(DcMotorEx.class, "viperSlide");
                slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



                pidController = new PIDController_v2(kP, kI, kD);
                timer = new ElapsedTime();
            }
*/
            // Method to set target position
            public void setTargetPosition(int targetPosition) {
                pidController.setSetpoint(targetPosition);
                setTargetPosition(100);

            }

            // Method to update motor power based on PID calculation
            public void update() {
                double currentPosition = slideMotor.getCurrentPosition();
                double pidOutput = pidController.calculate(currentPosition, timer.seconds());
                slideMotor.setPower(pidOutput);

            }

            // Inner PID Controller class
             class PIDController {
                private double kP, kI, kD;
                private double setpoint;
                private double errorSum;
                private double lastError;
                private double lastTime;

                public PIDController(double kP, double kI, double kD) {
                    this.kP = kP;
                    this.kI = kI;
                    this.kD = kD;
                }

                public void setSetpoint(double setpoint) {
                    this.setpoint = setpoint;
                    errorSum = 0;
                    lastError = 0;
                }

                public double calculate(double currentValue, double currentTime) {
                    double error = setpoint - currentValue;
                    double deltaTime = currentTime - lastTime;

                    errorSum += error * deltaTime;
                    double deltaError = (error - lastError) / deltaTime;

                    double output = kP * error + kI * errorSum + kD * deltaError;

                    lastError = error;
                    lastTime = currentTime;
                    if (gamepad1.a){

                    }


                    return output;

                }
            }
        }


    }


    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {

    }

    @Override
    public void setSlidePosition(int targetPosition, Telemetry telemetry) {

    }

    @Override
    public int getSlidePosition(Telemetry telemetry) {
        return 0;
    }

    @Override
    public void setSlideSpeed(double slideSpeed, Telemetry telemetry) {

    }
}
