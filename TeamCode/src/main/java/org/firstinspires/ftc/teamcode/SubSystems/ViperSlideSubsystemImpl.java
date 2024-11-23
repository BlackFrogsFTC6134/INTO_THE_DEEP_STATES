package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ViperSlideSubsystemImpl implements ViperSlideSubsystem {
        private DcMotorEx linearViper;
    private int LINEAR_VIPER_TARGET_POSITION_UP = 2000; // Target position for moving up (in encoder ticks)
    private int LINEAR_VIPER_TARGET_POSITION_UP_LIMITED = 800; // Target position for moving up (in encoder ticks)
    private int LINEAR_VIPER_TARGET_POSITION_DOWN = 0;// Target position for moving down (in encoder ticks)
    double viperSlideTicksPerRev = 384.5;
    public double LINEAR_VIPER_STARTING_POSTITION;


    @Override
        public void setSlidePosition(int position) {
            linearViper.setTargetPosition(position);
            linearViper.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            linearViper.setPower(1.0);
        }

        @Override
        public int getSlidePosition() {
            return linearViper.getCurrentPosition();
        }

        @Override
        public void setSlideSpeed(double speed) {
            linearViper.setPower(speed);
        }

        @Override

            public void initialize(HardwareMap hardwareMap) {
                linearViper = hardwareMap.get(DcMotorEx.class, "linear_Viper");
                linearViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                linearViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }

    @Override
        public void update(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
            // Implement any periodic checks or updates
        }

        @Override
        public void stopSubsystem(HardwareMap hardwareMap) {
            linearViper.setPower(0);
        }
    }
