package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Hood IO using a single SPARK MAX (NEO 550) with onboard position control. */
public class HoodIOAxon implements HoodIO {
  private Servo axonServo;
  private double servoAngle = kDisabledAngleRad;
  private AnalogInput axonEncoder;


  public HoodIOAxon() {
    axonServo = new Servo(0);
    axonEncoder = new AnalogInput(1);
  } // End HoodIOSparkMax Constructor

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
