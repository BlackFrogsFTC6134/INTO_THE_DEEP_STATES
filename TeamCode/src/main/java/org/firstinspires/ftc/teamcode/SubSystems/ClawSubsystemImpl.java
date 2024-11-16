package org.firstinspires.ftc.teamcode.states.SubSystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystemImpl implements ClawSubsystem {
        private Servo clawServo;
        private static final double OPEN_POSITION = 1.0;
        private static final double CLOSED_POSITION = 0.0;
        private boolean isOpen = false;

        @Override
        public void initialize(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
        }

        @Override
        public void openClaw() {
            clawServo.setPosition(OPEN_POSITION);
            isOpen = true;
        }

        @Override
        public void closeClaw() {
            clawServo.setPosition(CLOSED_POSITION);
            isOpen = false;
        }

        @Override
        public boolean isClawOpen() {
            return isOpen;
        }

        @Override
        public void periodic() {
            // Implement any periodic checks or updates
        }

        @Override
        public void stop() {
            // No action needed for servo
        }
    }

