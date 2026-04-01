package frc.robot.subsystems.shooter.transfer;

import static frc.robot.subsystems.shooter.transfer.TransferConstants.kMaxVoltage;
import static frc.robot.subsystems.shooter.transfer.TransferConstants.kMotorInverted;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Transfer IO for simulation; NEO 550 voltage control. */
public class TransferIOSim implements TransferIO {

  private static final double kLoopPeriodSecs = 0.02;

  private final DCMotor motorModel = DCMotor.getNeo550(1);
  private final DCMotorSim motorSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, 0.001, 1.0), motorModel);

  private double appliedVolts = 0.0;
  private boolean isStopped = true;

  @Override
  public void updateInputs(TransferIOInputs inputs) {
    if (isStopped) {
      appliedVolts = 0.0;
      motorSim.setInputVoltage(0.0);
    } else {
      motorSim.setInputVoltage(appliedVolts);
    }
    motorSim.update(kLoopPeriodSecs);

    inputs.motorConnected = true;
    inputs.appliedVolts = isStopped ? 0.0 : appliedVolts;
    inputs.supplyCurrentAmps = motorSim.getCurrentDrawAmps();
    inputs.colorSensorTripped = false;
  } // End updateInputs

  @Override
  public void setVoltage(double volts, boolean ignoreLimits) {
    double clamped = ignoreLimits
        ? MathUtil.clamp(volts, -Constants.kNominalVoltage, Constants.kNominalVoltage)
        : MathUtil.clamp(volts, -kMaxVoltage, kMaxVoltage);
    appliedVolts = kMotorInverted ? -clamped : clamped;
    isStopped = false;
  } // End setVoltage

  @Override
  public void stop() {
    isStopped = true;
  } // End stop
}
