package org.firstinspires.ftc.teamcode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.HangingSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.ViperSlideSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.ViperSlideSubsystemImpl;
import org.firstinspires.ftc.teamcode.SubSystems.ClawSubsystemImpl;
import org.firstinspires.ftc.teamcode.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.HangingSubsystemImpl;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveSubsystem;

@Config
public class BFR_TeleOp_2_0 {

    public final DriveSubsystem drive;
    public final ViperSlideSubsystem viperSlide;
    public final ClawSubsystem claw;
    public final HangingSubsystem hanging;

    public BFR_TeleOp_2_0(HardwareMap hardwareMap) {
            drive = new MecanumDriveSubsystem();
            viperSlide = new ViperSlideSubsystemImpl();
            claw = new ClawSubsystemImpl();
            hanging = new HangingSubsystemImpl();

            drive.initialize(hardwareMap);
            viperSlide.initialize(hardwareMap);
            claw.initialize(hardwareMap);
            hanging.initialize(hardwareMap);
        }

        public void periodic() {
            drive.periodic();
            viperSlide.periodic();
            claw.periodic();
            hanging.periodic();
        }

        public void stop() {
            drive.stop();
            viperSlide.stop();
            claw.stop();
            hanging.stop();
        }
    }

