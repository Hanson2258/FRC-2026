package frc.robot.subsystems.extender;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase {

  private final ExtenderIO extenderIO;
  private final ExtenderIO.ExtenderIOInputs extenderInputs = new ExtenderIO.ExtenderIOInputs();
  //private final ExtenderIO.ExtenderIOInputs inputs;

  public enum ExtenderStates {
    UP,
    DOWN,
    IDLE
  }
  public Extender(ExtenderIO io) {
    extenderIO = io; 
  }

  @Override
  public void periodic() {

  }

  public void resetEncoders() {

  }

  public void setTargetPosition(double position) {
    extenderInputs.targetPositionRads = position;
  }

  public double getTargetPosition() {
    return extenderInputs.positionRads;
  }

  public void stepPosition(double steps) {

  }


}
