package frc.robot.subsystems.shooter.transfer;

import static frc.robot.subsystems.shooter.transfer.TransferConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
// import com.revrobotics.ColorSensorV3;
import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.I2C;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import frc.robot.generated.TunerConstants;

/** Transfer IO using Talon FX (Kraken) with voltage control and optional colour sensor. */
public class TransferIOTalonFX implements TransferIO {

  private final TalonFX motor;
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  // private final ColorSensorV3 colorSensor;

  public TransferIOTalonFX() {
    motor = new TalonFX(kMotorId, TunerConstants.kCANBus);
    // colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    var talonFxConfig = new TalonFXConfiguration();
    talonFxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    talonFxConfig.MotorOutput.Inverted =
        kMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    talonFxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFxConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimitAmps;

    tryUntilOk(5, () -> motor.getConfigurator().apply(talonFxConfig, 0.25));
  } // End TransferIOTalonFX Constructor

  @Override
  public void updateInputs(TransferIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            motor.getMotorVoltage(),
            motor.getSupplyCurrent());
    inputs.motorConnected = status.equals(StatusCode.OK);
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    // inputs.colorSensorTripped = colorSensor.getProximity() >= kColorSensorProximityThreshold;
    inputs.colorSensorTripped = false;
  } // End updateInputs

  @Override
  public void setVoltage(double volts) {
    double clamped = MathUtil.clamp(volts, -kMaxVoltage, kMaxVoltage);
    motor.setControl(voltageRequest.withOutput(clamped));
  } // End setVoltage

  @Override
  public void stop() {
    motor.setControl(voltageRequest.withOutput(0.0));
  } // End stop
}
