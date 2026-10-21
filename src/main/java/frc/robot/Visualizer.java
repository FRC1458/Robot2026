// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.shooter.ShooterConstants;

import java.util.function.Supplier;

import dev.doglog.DogLog;

/** Add your docs here. */
public class Visualizer {
    private Translation3d[] trajectory = new Translation3d[50];
    private Supplier<Pose3d> poseSupplier;
    private Supplier<ChassisSpeeds> fieldSpeedsSupplier;

    public Visualizer(Supplier<Pose3d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;
    }

    private Translation3d launchVel(LinearVelocity vel, Angle angle) {
        Pose3d robot = poseSupplier.get();
        ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

        double horizontalVel = Math.cos(angle.in(Radians)) * vel.in(MetersPerSecond);
        double verticalVel = Math.sin(angle.in(Radians)) * vel.in(MetersPerSecond);
        double xVel =
                horizontalVel * Math.cos(robot.getRotation().toRotation2d().getRadians());
        double yVel =
                horizontalVel * Math.sin(robot.getRotation().toRotation2d().getRadians());

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        return new Translation3d(xVel, yVel, verticalVel);
    }

    private Translation3d launchVel(LinearVelocity vel, Angle angle, Rotation2d rotation) {
        Pose3d robot = poseSupplier.get();
        ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

        double horizontalVel = Math.cos(angle.in(Radians)) * vel.in(MetersPerSecond);
        double verticalVel = Math.sin(angle.in(Radians)) * vel.in(MetersPerSecond);
        double xVel =
                horizontalVel * Math.cos(rotation.getRadians());
        double yVel =
                horizontalVel * Math.sin(rotation.getRadians());

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        return new Translation3d(xVel, yVel, verticalVel);
    }


    public void updateFuel(LinearVelocity vel, Angle angle, Rotation2d rotation) {
        Translation3d trajVel = launchVel(vel, Degrees.of(90).minus(angle), rotation);
        for (int i = 0; i < trajectory.length; i++) {
            double t = i * 0.04;
            double x = trajVel.getX() * t + poseSupplier.get().getTranslation().getX();
            double y = trajVel.getY() * t + poseSupplier.get().getTranslation().getY();
            double z = trajVel.getZ() * t
                    - 0.5 * 9.81 * t * t
                    + poseSupplier.get().getTranslation().getZ();

            trajectory[i] = new Translation3d(x, y, z);
        }

        DogLog.log("/Trajectory", trajectory);
    }

    public void updateFuel(LinearVelocity vel, Angle angle) {
        Translation3d trajVel = launchVel(vel, Degrees.of(90).minus(angle));
        for (int i = 0; i < trajectory.length; i++) {
            double t = i * 0.04;
            double x = trajVel.getX() * t + poseSupplier.get().getTranslation().getX();
            double y = trajVel.getY() * t + poseSupplier.get().getTranslation().getY();
            double z = trajVel.getZ() * t
                    - 0.5 * 9.81 * t * t
                    + poseSupplier.get().getTranslation().getZ();

            trajectory[i] = new Translation3d(x, y, z);
        }

        DogLog.log("/Trajectory", trajectory);
    }

    public void update3dPose(Angle azimuthAngle, Angle hoodAngle) {
        Pose3d Pose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, azimuthAngle.in(Radians)));
        DogLog.log("/Pose", Pose);
        Pose3d hoodPose = new Pose3d(0.1, 0, 0, new Rotation3d(0, hoodAngle.in(Radians), 0));
        hoodPose = hoodPose.rotateAround(new Translation3d(), new Rotation3d(0, 0, azimuthAngle.in(Radians)));
        hoodPose = new Pose3d(
                hoodPose.getTranslation().plus(ShooterConstants.OFFSET), hoodPose.getRotation());
        DogLog.log("/HoodPose", hoodPose);
    }
}