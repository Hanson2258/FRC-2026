package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;

/** Hood IO using an Axon Servo and Analog Input. */
public class HoodIOAxon implements HoodIO {
  private Servo axonServo;
  private double servoAngle = kDisabledAngleRad;
  private AnalogInput axonEncoder;


  public HoodIOAxon() {
    axonServo = new Servo(kServoId);
    axonEncoder = new AnalogInput(kEncoderId);
  } // End HoodIOAxon Constructor

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.positionRads = axonEncoder.getAverageVoltage()/3.3 * 180;
  } // End updateInputss

  @Override
  public void setTargetPosition(double targetRads) {
    servoAngle = Units.radiansToDegrees(targetRads);
    axonServo.set(servoAngle);
  } // End setTargetPosition

  @Override
  public void resetEncoder() {
  } // End resetEncoder

  @Override
  public void stop() {
    axonServo.set(kDisabledAngleRad);
  } // End stop
}
