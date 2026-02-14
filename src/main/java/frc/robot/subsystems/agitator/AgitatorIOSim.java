package frc.robot.subsystems.agitator;

import static frc.robot.subsystems.agitator.AgitatorConstants.kMaxVoltage;
import static frc.robot.subsystems.agitator.AgitatorConstants.kMotorInverted;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Agitator IO for simulation; NEO 550 voltage control. */
public class AgitatorIOSim implements AgitatorIO {

  private static final double kLoopPeriodSecs = 0.02;

  private final DCMotor motorModel = DCMotor.getNeo550(1);
  private final DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(motorModel, 0.001, 1.0),
          motorModel);

  private double appliedVolts = 0.0;
  private boolean isStopped = true;

  @Override
  public void updateInputs(AgitatorIOInputs inputs) {
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
  } // End updateInputs

  @Override
  public void setVoltage(double volts) {
    double clamped = MathUtil.clamp(volts, -kMaxVoltage, kMaxVoltage);
    appliedVolts = kMotorInverted ? -clamped : clamped;
    isStopped = false;
  } // End setVoltage

  @Override
  public void stop() {
    isStopped = true;
  } // End stop
}
