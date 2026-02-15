package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;

/**
 * Physics-based shooter calculator: funnel clearance parabola and iterative moving shot.
 * All units: meters, radians, m/s. Hood angle convention: from vertical (π/2 − elevation from
 * horizontal).
 */
public final class ShooterCalculator {

  private static final double G_MPS2 = 9.81;

  private ShooterCalculator() {}

  /**
   * Horizontal distance from robot center to target (x, y), meters.
   */
  public static double getDistanceToTarget(Pose2d robot, Translation3d target) {
    return robot.getTranslation().getDistance(target.toTranslation2d());
  }

  /**
   * Time of flight (s) for a projectile to travel horizontal distance. Hood angle in rad (from
   * vertical).
   */
  public static double calculateTimeOfFlight(
      double exitVelocityMps, double hoodAngleFromVerticalRad, double distanceM) {
    double elevationRad = Math.PI / 2 - hoodAngleFromVerticalRad;
    return distanceM / (exitVelocityMps * Math.cos(elevationRad));
  } // End calculateTimeOfFlight

  /**
   * Predict target position after moving backward by fieldSpeeds * timeOfFlight (robot moving,
   * target fixed in field).
   */
  public static Translation3d predictTargetPos(
      Translation3d target, ChassisSpeeds fieldSpeeds, double timeOfFlightSec) {
    double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlightSec;
    double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlightSec;
    return new Translation3d(predictedX, predictedY, target.getZ());
  } // End predictTargetPos

  /**
   * Exit linear velocity (m/s) to flywheel rad/s.
   */
  public static double linearToAngularVelocity(double exitVelocityMps, double radiusM) {
    return exitVelocityMps / radiusM;
  }

  /**
   * Turret angle in robot frame (rad) to aim at target from turret pivot. Uses pivot position
   * (robot + robotToTurret).
   */
  public static Rotation2d calculateAzimuthAngleRobotFrame(Pose2d robot, Translation3d target) {
    Translation2d pivotTranslation =
        new Pose3d(robot).transformBy(robotToTurret).toPose2d().getTranslation();
    Translation2d direction = target.toTranslation2d().minus(pivotTranslation);
    double robotFrameRad =
        MathUtil.inputModulus(
            direction.getAngle().minus(robot.getRotation()).getRadians(), -Math.PI, Math.PI);
    return Rotation2d.fromRadians(robotFrameRad);
  } // End calculateAzimuthAngleRobotFrame

  /**
   * Field-frame angle from turret pivot to target (direction to aim in field coordinates).
   */
  public static Rotation2d calculateAzimuthAngleFieldFrame(Pose2d robot, Translation3d target) {
    Translation2d pivotTranslation =
        new Pose3d(robot).transformBy(robotToTurret).toPose2d().getTranslation();
    Translation2d direction = target.toTranslation2d().minus(pivotTranslation);
    return direction.getAngle();
  } // End calculateAzimuthAngleFieldFrame

  /**
   * Shot from funnel clearance (parabola passing above funnel). Uses robot center for horizontal
   * distance; launch height from robotToTurret Z. Hood angle returned as from vertical (rad).
   *
   * @param robot robot pose
   * @param actualTarget actual hub target (for scaling r)
   * @param predictedTarget predicted target (for parabola geometry)
   */
  public static ShotData calculateShotFromFunnelClearance(
      Pose2d robot, Translation3d actualTarget, Translation3d predictedTarget) {
    double xDist = getDistanceToTarget(robot, predictedTarget);
    double yDist = predictedTarget.getZ() - robotToTurret.getZ();
    double distToActual = getDistanceToTarget(robot, actualTarget);
    double r =
        (distToActual > 1e-6)
            ? FieldConstants.FUNNEL_RADIUS_M * xDist / distToActual
            : FieldConstants.FUNNEL_RADIUS_M;
    double h = FieldConstants.FUNNEL_HEIGHT_M + kDistanceAboveFunnelM;

    double A1 = xDist * xDist;
    double B1 = xDist;
    double D1 = yDist;
    double A2 = -xDist * xDist + (xDist - r) * (xDist - r);
    double B2 = -r;
    double D2 = h;
    double Bm = -B2 / B1;
    double A3 = Bm * A1 + A2;
    double D3 = Bm * D1 + D2;
    double a = D3 / A3;
    double b = (D1 - A1 * a) / B1;
    double theta = Math.atan(b);
    double v0Sq = -G_MPS2 / (2 * a * Math.cos(theta) * Math.cos(theta));
    double v0 = Math.sqrt(Math.max(0, v0Sq));
    if (Double.isNaN(v0) || Double.isNaN(theta)) {
      v0 = 0;
      theta = 0;
    }
    double hoodAngleFromVerticalRad = Math.PI / 2 - theta;
    return new ShotData(v0, hoodAngleFromVerticalRad, predictedTarget);
  } // End calculateShotFromFunnelClearance

  /**
   * Iterative moving shot: predict target at time of flight, recompute shot, repeat. Returns final
   * ShotData.
   */
  public static ShotData iterativeMovingShotFromFunnelClearance(
      Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
    ShotData shot = calculateShotFromFunnelClearance(robot, target, target);
    double distanceM = getDistanceToTarget(robot, target);
    double timeOfFlightSec =
        calculateTimeOfFlight(shot.exitVelocityMps, shot.hoodAngleFromVerticalRad, distanceM);
    Translation3d predictedTarget = target;

    for (int i = 0; i < iterations; i++) {
      predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlightSec);
      shot = calculateShotFromFunnelClearance(robot, target, predictedTarget);
      distanceM = getDistanceToTarget(robot, predictedTarget);
      timeOfFlightSec =
          calculateTimeOfFlight(shot.exitVelocityMps, shot.hoodAngleFromVerticalRad, distanceM);
    }
    return shot;
  } // End iterativeMovingShotFromFunnelClearance

  /** Exit velocity (m/s), hood angle from vertical (rad), and target used. */
  public record ShotData(
      double exitVelocityMps, double hoodAngleFromVerticalRad, Translation3d target) {}
}
