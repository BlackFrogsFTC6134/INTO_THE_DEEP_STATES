package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.HangingSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Subsystem;


public class Hanging implements Subsystem {
        private DcMotor hangingMotor;
        private boolean isExtended = false;

        @Override
        public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
            hangingMotor = hardwareMap.get(DcMotor.class, "Linear_Actuator");
            hangingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hangingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void extend(Telemetry telemetry, double targetPosition) {
            hangingMotor.setTargetPosition(1000); // Adjust this value
            hangingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hangingMotor.setPower(1.0);
            isExtended = true;
        }

        public void retract(Telemetry telemetry, double targetPosition) {
            hangingMotor.setTargetPosition(0);
            hangingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hangingMotor.setPower(1.0);
            isExtended = false;
        }

        public boolean isExtended(Telemetry telemetry) {
            return isExtended;
        }

    @Override
        public void update(Gamepad gamepad2, Telemetry telemetry) {
            // Implement any periodic checks or updates
        }

    /**
     * Manages all telemetry data to driver phone or FTC Dashboard
     * Where all telemetry.addData() calls should go
     *
     * @param telemetry = null
     */
    @Override
    public void telemetry(Telemetry telemetry) {

    }

    @Override
    public void stopSubsystem(Telemetry telemetry) throws InterruptedException {
            hangingMotor.setPower(0);
        }
    }