package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class ViperSlideSubsystemImpl implements ViperSlideSubsystem {
        private DcMotorEx linearViper;
    private int LINEAR_VIPER_TARGET_POSITION_UP = 2000; // Target position for moving up (in encoder ticks)
    private int LINEAR_VIPER_TARGET_POSITION_UP_LIMITED = 800; // Target position for moving up (in encoder ticks)
    private int LINEAR_VIPER_TARGET_POSITION_DOWN = 0;// Target position for moving down (in encoder ticks)
    double viperSlideTicksPerRev = 384.5;
    public double LINEAR_VIPER_STARTING_POSTITION;


    @Override
        public void setSlidePosition(int targetPosition, Telemetry telemetry) {
            linearViper.setTargetPosition(targetPosition);
            linearViper.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            linearViper.setPower(1.0);
        }

        @Override
        public int getSlidePosition(Telemetry telemetry) {
            return linearViper.getCurrentPosition();
        }

        @Override
        public void setSlideSpeed(double slideSpeed, Telemetry telemetry) {
            linearViper.setPower(slideSpeed);
        }

        @Override

            public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
                linearViper = hardwareMap.get(DcMotorEx.class, "linear_Viper");
                linearViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                linearViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }
/*
    @Override
        public void update(Telemetry telemetry) {
            // Implement any periodic checks or updates
        }

        @Override
        public void stopSubsystem(Telemetry telemetry) {
            linearViper.setPower(0);
        } */
    }
