package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Turret subsystem: one motor with onboard position control, aimed at a hub angle. */
public class Turret extends SubsystemBase {

  private final TurretIO turretIO; // Hardware Abstraction IO Interface for the turret motors
  private final TurretIO.TurretIOInputs turretInputs = new TurretIO.TurretIOInputs(); // Data Container for the turret values
  private static final double kAtHubToleranceRad = Units.degreesToRadians(2.0);

  private Rotation2d hubAngleRelativeToRobot = Rotation2d.kZero; // The angle to aim the turret at the hub

  public Turret(TurretIO io) {
    turretIO = io;
  } // End Turret Constructor

  @Override
  public void periodic() {
    // Update the turret inputs and record the values
    turretIO.updateInputs(turretInputs);
    Logger.recordOutput("Turret/Inputs/MotorConnected", turretInputs.motorConnected);
    Logger.recordOutput("Turret/Inputs/PositionRads", turretInputs.positionRads);
    Logger.recordOutput("Turret/Inputs/VelocityRadsPerSec", turretInputs.velocityRadsPerSec);
    Logger.recordOutput("Turret/Inputs/AppliedVolts", turretInputs.appliedVolts);
    Logger.recordOutput("Turret/Inputs/SupplyCurrentAmps", turretInputs.supplyCurrentAmps);
    Logger.recordOutput("Turret/PositionDegrees", getPosition().getDegrees());
    Logger.recordOutput("Turret/HubDegrees", getHubAngleRelativeToRobot().getDegrees());

    // If the robot is disabled, no power to turret
    if (DriverStation.isDisabled()) {
      turretIO.stop();
      return;
    }

    // Calculate and set the target turret angle in radians (relative to robot frame)
    double hubRadRelativeToRobot = MathUtil.clamp(hubAngleRelativeToRobot.getRadians(), kMinAngleRad, kMaxAngleRad);
    double targetAngleRad = hubRadRelativeToRobot + kEncoderZeroOffsetRad;
    turretIO.setTargetPosition(targetAngleRad);
  } // End periodic

  /** Set the hub angle (robot frame: 0 = forward). Clamped to min/max in periodic. */
  public void setHubAngleRelativeToRobot(Rotation2d angle) {
    hubAngleRelativeToRobot = angle;
  } // End setHubAngleRelativeToRobot

  /** Get the current hub angle. */
  public Rotation2d getHubAngleRelativeToRobot() {
    return hubAngleRelativeToRobot;
  } // End getHubAngleRelativeToRobot

  /** Get the current turret position (robot frame: 0 = forward). */
  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(turretInputs.positionRads + kEncoderZeroOffsetRad);
  } // End getPosition

  /** Whether the turret is at the hub within tolerance. */
  public boolean aimedAtHub() {
    double hubRadRelativeToRobot = MathUtil.clamp(hubAngleRelativeToRobot.getRadians(), kMinAngleRad, kMaxAngleRad);
    double currentAngleRad = turretInputs.positionRads + kEncoderZeroOffsetRad;
    return Math.abs(MathUtil.angleModulus(currentAngleRad - hubRadRelativeToRobot)) <= kAtHubToleranceRad;
  } // End aimedAtHub
}
