package frc.robot.subsystems.shooter.turret;

import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kD;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kGearRatio;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kI;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kIdleMode;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMotorId;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kP;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kSmartCurrentLimitAmps;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/** Turret IO using a single SPARK MAX (NEO 550) with onboard position control. */
public class TurretIOSparkMax implements TurretIO {

  private final SparkMax motor;
  private final SparkClosedLoopController closedLoopController;
  private final RelativeEncoder encoder;

  private double lastP = kP;
  private double lastI = kI;
  private double lastD = kD;
  private double targetPosition;

  public TurretIOSparkMax() {
    motor = new SparkMax(kMotorId, SparkLowLevel.MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    var sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.idleMode(kIdleMode);
    sparkMaxConfig.smartCurrentLimit(kSmartCurrentLimitAmps);
    sparkMaxConfig
        .encoder
        .positionConversionFactor(1.0 / kGearRatio)
        .velocityConversionFactor(1.0 / kGearRatio);
    sparkMaxConfig.closedLoop.p(kP).i(kI).d(kD);
    sparkMaxConfig.signals
        .appliedOutputPeriodMs(31)
        .busVoltagePeriodMs(31)
        .outputCurrentPeriodMs(31)
        .primaryEncoderPositionPeriodMs(31)
        .primaryEncoderVelocityPeriodMs(547);
    motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  } // End TurretIOSparkMax Constructor

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    double p = SmartDashboard.getNumber("Turret/kP", kP);
    double i = SmartDashboard.getNumber("Turret/kI", kI);
    double d = SmartDashboard.getNumber("Turret/kD", kD);

    if (p != lastP || i != lastI || d != lastD) {
      lastP = p;
      lastI = i;
      lastD = d;

      var sparkMaxConfig = new SparkMaxConfig();
      sparkMaxConfig.closedLoop.p(p).i(i).d(d);
      sparkMaxConfig.signals
          .appliedOutputPeriodMs(40)
          .busVoltagePeriodMs(40)
          .outputCurrentPeriodMs(40)
          .primaryEncoderPositionPeriodMs(40)
          .primaryEncoderVelocityPeriodMs(547);
      motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    inputs.motorConnected = motor.getLastError() == REVLibError.kOk;
    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition());
    inputs.targetPositionRads = targetPosition;
    inputs.velocityRadsPerSec = Units.rotationsToRadians(encoder.getVelocity() / 60.0);
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
  } // End updateInputs

  @Override
  public void setTargetPosition(double targetRads) {
    setTargetPosition(targetRads, 0.0);
  } // End setTargetPosition

  @Override
  public void setTargetPosition(double targetRads, double velocityFeedforwardRadPerSec) {
    double targetRot = Units.radiansToRotations(targetRads);
    closedLoopController.setSetpoint(targetRot, SparkBase.ControlType.kPosition);
    targetPosition = targetRads;
    // SPARK MAX position control does not expose velocity feedforward; ignore
  } // End setTargetPosition
  
  @Override
  public void resetEncoder() { 
    encoder.setPosition(0.0);
  } // End resetEncoder

  @Override
  public void stop() {
    motor.set(0.0);
  } // End stop
}
