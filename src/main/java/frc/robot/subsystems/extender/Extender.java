package frc.robot.subsystems.extender;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase {

  private final ExtenderIO extenderIO;
  private final ExtenderIO.ExtenderIOInputs extenderInputs = new ExtenderIO.ExtenderIOInputs();
  //private final ExtenderIO.ExtenderIOInputs inputs;

  public Extender(ExtenderIO io) {
    extenderIO = io; 
  }
}
