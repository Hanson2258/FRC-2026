package frc.robot.simulation;

/**
 * AdvantageKit log prefix for the optional second sim robot. Keeps all second-robot fields under one
 * tree (see Constants.SimulationDrive toggles). Duplicate subsystems must log under here so they do not
 * overwrite the primary robot’s {@code Subsystems/...} NT/log keys each scheduler tick.
 */
public final class SecondSimRobotOutputs {

  /** Root path for second sim robot telemetry. */
  public static final String ROOT = "/SecondSimRobotOutputs";

  private SecondSimRobotOutputs() {}

  /** {@code ROOT + "/" + relative} — {@code relative} must not start with "/". */
  public static String path(String relative) {
    return ROOT + "/" + relative;
  } // End path

  /**
   * Prefix for SmartDashboard keys (no leading slash). Second sim mechanisms must not share the primary
   * robot’s {@code Flywheel/...} table with another instance.
   */
  public static String smartDashboardPrefix(String relative) {
    return "SecondSimRobotOutputs/" + relative;
  } // End smartDashboardPrefix
}
