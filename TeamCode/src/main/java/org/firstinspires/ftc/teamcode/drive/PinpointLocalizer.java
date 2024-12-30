package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
//import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.GoBildaPinpointDriver;

public class PinpointLocalizer implements Localizer {
    private GoBildaPinpointDriver odo;

    public PinpointLocalizer(GoBildaPinpointDriver odo) {
        this.odo = odo;
    }

    @NonNull

    public Pose2d getPoseEstimate() {
        Pose2D pose = odo.getPosition();
        return new Pose2d(-pose.getY(DistanceUnit.INCH), pose.getX(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.RADIANS));
    }

    public void setPoseEstimate(@NonNull Pose2d pose2d) {

    }

    @Nullable
    public Pose2d getPoseVelocity() {
        return new Pose2d(odo.getVelX(), odo.getVelY(), odo.getHeadingVelocity());
    }

    public void update1() {
        odo.update();
        Telemetry t = FtcDashboard.getInstance().getTelemetry();
        t.addData("status", odo.getDeviceStatus());
        t.update();
    }

    @Override
    public Twist2dDual<Time> update() {
        return null;
    }
}
