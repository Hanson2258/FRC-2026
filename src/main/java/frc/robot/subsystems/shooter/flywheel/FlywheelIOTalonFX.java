package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Flywheel IO using Talon FX with onboard velocity control. */
public class FlywheelIOTalonFX implements FlywheelIO {

  private static final double kVelocityRampRateRadPerSecSq =
      Units.rotationsPerMinuteToRadiansPerSecond(kVelocityRampRateRpmPerSec);

  private final TalonFX motor;
  private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  private final PIDController velocityPID = new PIDController(kP, kI, kD);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);

  private final SlewRateLimiter velocityRamp = new SlewRateLimiter(kVelocityRampRateRadPerSecSq);

  private double lastP = kP;
  private double lastI = kI;
  private double lastD = kD;
  private double lastV = kV;
  private double lastS = kS;

  public FlywheelIOTalonFX() {
    motor = new TalonFX(kMotorId, TunerConstants.kCANBus);

    var talonFxConfig = new TalonFXConfiguration();
    talonFxConfig.MotorOutput.NeutralMode = kNeutralMode;
    talonFxConfig.MotorOutput.Inverted =
        kMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    talonFxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFxConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimitAmps;

    talonFxConfig.Slot0.kP = kP;
    talonFxConfig.Slot0.kI = kI;
    talonFxConfig.Slot0.kD = kD;
    talonFxConfig.Slot0.kV = kV;
    talonFxConfig.Slot0.kS = kS;

    tryUntilOk(5, () -> motor.getConfigurator().apply(talonFxConfig, 0.25));
  } // End FlywheelIOTalonFX Constructor

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    double p = SmartDashboard.getNumber("Flywheel/kP", kP);
    double i = SmartDashboard.getNumber("Flywheel/kI", kI);
    double d = SmartDashboard.getNumber("Flywheel/kD", kD);
    double v = SmartDashboard.getNumber("Flywheel/kV", kV);
    double s = SmartDashboard.getNumber("Flywheel/kS", kS);

    if (p != lastP || i != lastI || d != lastD || v != lastV || s != lastS) {
      lastP = p;
      lastI = i;
      lastD = d;
      lastV = v;
      lastS = s;

      var slot0 = new Slot0Configs().withKP(p).withKI(i).withKD(d).withKV(v).withKS(s);

      velocityPID.setP(p);
      velocityPID.setI(i);
      velocityPID.setD(d);

      feedforward.setKs(s);
      feedforward.setKv(v);

      tryUntilOk(5, () -> motor.getConfigurator().apply(slot0, 0.25));
    }

    var signalRefreshStatus =
        BaseStatusSignal.refreshAll(
            motor.getVelocity(),
            motor.getMotorVoltage(),
            motor.getSupplyCurrent());
    inputs.motorConnected = signalRefreshStatus.equals(StatusCode.OK);
    double motorShaftRps = motor.getVelocity().getValueAsDouble();
    inputs.velocityRadsPerSec = Units.rotationsToRadians(motorShaftRps) / kGearRatio;
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
  } // End updateInputs

  @Override
  public void setTargetVelocity(double targetVelocityRadPerSec) {
    boolean atSpeed = Math.abs(motor.getVelocity().getValueAsDouble() - targetVelocityRadPerSec) <= kAtTargetVelocityToleranceRadPerSec;
    if (atSpeed) {
      setVelocityControl(targetVelocityRadPerSec);
    } else {
      setTorqueControl(targetVelocityRadPerSec);
    }

  } // End setTargetVelocity

  private void setVelocityControl(double targetVelocityRadPerSec) {
    motor.setControl(velocityVoltageRequest.withVelocity(calculateTargetRps(targetVelocityRadPerSec)));
  }

  private void setTorqueControl(double targetVelocityRadPerSec) {
    double targetRps = calculateTargetRps(targetVelocityRadPerSec);
    double currentRps = motor.getVelocity().getValueAsDouble();

    double torqueCommand = feedforward.calculate(targetRps) + velocityPID.calculate(currentRps, targetRps);
    
    motor.setControl(torqueRequest.withOutput(torqueCommand));
  }

  private double calculateTargetRps(double targetVelocityRadPerSec) {
    double rampedRadPerSec = velocityRamp.calculate(targetVelocityRadPerSec);
    double targetRps = (rampedRadPerSec / (2.0 * Math.PI)) * kGearRatio;
    return targetRps;
  }

  @Override
  public void stop() {
    velocityRamp.reset(0.0);
    motor.setControl(new NeutralOut());
  } // End stop
}
