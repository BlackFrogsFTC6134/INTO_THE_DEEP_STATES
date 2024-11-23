package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class HangingSubsystemImpl implements HangingSubsystem {
        private DcMotor hangingMotor;
        private boolean isExtended = false;

        @Override
        public void initialize(HardwareMap hardwareMap) {
            hangingMotor = hardwareMap.get(DcMotor.class, "Linear_Actuator");
            hangingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hangingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        @Override
        public void extend() {
            hangingMotor.setTargetPosition(1000); // Adjust this value
            hangingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hangingMotor.setPower(1.0);
            isExtended = true;
        }

        @Override
        public void retract() {
            hangingMotor.setTargetPosition(0);
            hangingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hangingMotor.setPower(1.0);
            isExtended = false;
        }

        @Override
        public boolean isExtended() {
            return isExtended;
        }

    @Override
        public void update(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
            // Implement any periodic checks or updates
        }

        @Override
        public void stopSubsystem(HardwareMap hardwareMap) {
            hangingMotor.setPower(0);
        }
    }

