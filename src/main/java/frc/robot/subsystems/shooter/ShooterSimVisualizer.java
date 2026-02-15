// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ShooterSimVisualizer {
    private Translation3d[] trajectory = new Translation3d[50];
    private Supplier<Pose3d> poseSupplier;
    private Supplier<ChassisSpeeds> fieldSpeedsSupplier;

    public ShooterSimVisualizer(Supplier<Pose3d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;
    }

    private Translation3d launchVel(LinearVelocity vel, Angle elevationAngle, Angle shotYawFieldFrame) {
        ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

        double horizontalVel = Math.cos(elevationAngle.in(Radians)) * vel.in(MetersPerSecond);
        double verticalVel = Math.sin(elevationAngle.in(Radians)) * vel.in(MetersPerSecond);
        double yawRad = shotYawFieldFrame.in(Radians);
        double xVel = horizontalVel * Math.cos(yawRad);
        double yVel = horizontalVel * Math.sin(yawRad);

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        return new Translation3d(xVel, yVel, verticalVel);
    }

    /**
     * @param vel Launch speed
     * @param hoodAngleRad Hood angle from vertical (radians)
     * @param turretYawRobotFrame Turret yaw in robot frame (radians); trajectory direction = robot heading + this
     */
    public void updateFuel(LinearVelocity vel, Angle hoodAngleRad, Angle turretYawRobotFrame) {
        Angle elevationAngle = Degrees.of(90).minus(hoodAngleRad);
        Angle shotYawFieldFrame =
                Radians.of(poseSupplier.get().getRotation().toRotation2d().getRadians() + turretYawRobotFrame.in(Radians));
        Translation3d trajVel = launchVel(vel, elevationAngle, shotYawFieldFrame);
        Translation3d launchPos = poseSupplier.get().plus(ShooterConstants.robotToTurret).getTranslation();
        for (int i = 0; i < trajectory.length; i++) {
            double t = i * 0.04;
            double x = trajVel.getX() * t + launchPos.getX();
            double y = trajVel.getY() * t + launchPos.getY();
            double z = trajVel.getZ() * t - 0.5 * 9.81 * t * t + launchPos.getZ();

            trajectory[i] = new Translation3d(x, y, z);
        }

        Logger.recordOutput("ShooterVisualizer/Trajectory", trajectory);
    }

    public void update3dPose(Angle azimuthAngle, Angle hoodAngle) {
        Pose3d turretPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, azimuthAngle.in(Radians)));
        Logger.recordOutput("ShooterVisualizer/TurretPose", turretPose);
        Pose3d hoodPose = new Pose3d(0.1, 0, 0, new Rotation3d(0, hoodAngle.in(Radians), 0));
        hoodPose = hoodPose.rotateAround(new Translation3d(), new Rotation3d(0, 0, azimuthAngle.in(Radians)));
        hoodPose = new Pose3d(
                hoodPose.getTranslation().plus(ShooterConstants.robotToTurret.getTranslation()),
                hoodPose.getRotation());
        Logger.recordOutput("ShooterVisualizer/HoodPose", hoodPose);
    }
}