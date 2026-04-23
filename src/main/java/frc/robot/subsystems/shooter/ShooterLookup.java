package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.hood.HoodConstants;

/**
 * Distance-based shooter table: hand-edited rows, linearly interpolated. Hub vs passing use
 * different tables (target height). Row distance is the lookup key (m); hub uses 1.0–7.5 m every
 * 0.5 m, passing 3.0–12.0 m every 0.5 m. {@code hoodAngleDeg} is the desired ball exit elevation
 * (deg); {@code commandedHoodAngleDeg} is sent to the hood so realized exit matches {@code
 * hoodAngleDeg} (often equal at seed time). Both must stay within {@link HoodConstants#kMinAngleRad}
 * and {@link HoodConstants#kMaxAngleRad} (50°–80°).
 *
 * <p>Regenerate: {@code .\gradlew.bat printShooterLookupSeed}.
 */
public final class ShooterLookup {

  private ShooterLookup() {}

  /**
   * One row: range key (m), exit hood (deg), commanded hood (deg), flywheel RPM at additive=0, TOF
   * (s). RPM matches {@code Shooter/CalculatorVelocityRpm} when {@code
   * Shooter/ExitVelocityCompensationMultiplierAdditive} is 0; runtime scales by {@code (kExit +
   * additive) / kExit}.
   */
  public record TableEntry(
      double distanceMeters,
      double hoodAngleDeg,
      double commandedHoodAngleDeg,
      double flywheelRpm,
      double timeOfFlightSec) {}

  /**
   * Interpolated exit hood (rad, for logging), commanded hood (rad, for {@link
   * frc.robot.subsystems.shooter.hood.Hood} setpoint), table RPM, TOF (s).
   */
  public record Result(
      double hoodExitAngleRad,
      double commandedHoodAngleRad,
      double flywheelRpmTable,
      double timeOfFlightSec) {}

  private static final TableEntry[] HUB_ENTRIES =
      new TableEntry[] {
        // .85m constant - Add distance from Hub Wall to this value for distance value.
        // Need to tune TOF

        // distance_m, hood_exit_deg, hood_commanded_deg, flywheel_rpm @ additive=0, time_of_flight_s
        new TableEntry(0.90, 80.0, 80.0, 3300, 0.935480),
        new TableEntry(1.15, 80.0, 80.0, 3000, 0.935480), // Soft balls launch at steeper angle (works from 2700 to 3000 rpm), hard balls at shallower angle (misses-hits hub).
        new TableEntry(1.47, 80.0, 80.0, 3000, 0.899378),
        new TableEntry(1.85, 80.0, 68.0, 3000, 0.899378),
        new TableEntry(2.25, 80.0, 65.0, 3000, 0.899378),
        new TableEntry(2.65, 80.0, 63.0, 3200, 0.899378),

      };

  private static final TableEntry[] PASSING_ENTRIES =
      new TableEntry[] {
        // distance_m, hood_exit_deg, hood_commanded_deg, flywheel_rpm @ additive=0, time_of_flight_s
        new TableEntry(3.000000, 50.0000, 50.0000, 2304.9467, 0.905894),
        new TableEntry(3.500000, 50.0000, 50.0000, 2509.7487, 0.970633),
        new TableEntry(4.000000, 50.0000, 50.0000, 2699.5142, 1.031315),
        new TableEntry(4.500000, 50.0000, 50.0000, 2877.0874, 1.088621),
        new TableEntry(5.000000, 50.0000, 50.0000, 3044.5238, 1.143057),
        new TableEntry(5.500000, 50.0000, 50.0000, 3203.3638, 1.195016),
        new TableEntry(6.000000, 50.0000, 50.0000, 3354.7966, 1.244807),
        new TableEntry(6.500000, 50.0000, 50.0000, 3499.7616, 1.292683),
        new TableEntry(7.000000, 50.0000, 50.0000, 3639.0166, 1.338848),
        new TableEntry(7.500000, 50.0000, 50.0000, 3773.1828, 1.383473),
        new TableEntry(8.000000, 50.0000, 50.0000, 3902.7768, 1.426702),
        new TableEntry(8.500000, 50.0000, 50.0000, 4028.2338, 1.468660),
        new TableEntry(9.000000, 50.0000, 50.0000, 4149.9243, 1.509453),
        new TableEntry(9.500000, 50.0000, 50.0000, 4268.1670, 1.549171),
        new TableEntry(10.000000, 50.0000, 50.0000, 4383.2379, 1.587896),
        new TableEntry(10.500000, 50.0000, 50.0000, 4495.3785, 1.625699),
        new TableEntry(11.000000, 50.0000, 50.0000, 4604.8009, 1.662643),
        new TableEntry(11.500000, 50.0000, 50.0000, 4711.6930, 1.698783),
        new TableEntry(12.000000, 50.0000, 50.0000, 4816.2222, 1.734171),
      };

  /**
   * Interpolates exit vs commanded hood (rad), flywheel RPM (table; scale exit-velocity additive in
   * caller), and time of flight (s) for the given horizontal range key.
   */
  public static Result lookup(boolean hubShot, double distanceMeters) {
    TableEntry[] table = hubShot ? HUB_ENTRIES : PASSING_ENTRIES;
    return interpolate(table, distanceMeters);
  }

  private static Result interpolate(TableEntry[] table, double distanceMeters) {
    double minDeg = Math.toDegrees(HoodConstants.kMinAngleRad);
    double maxDeg = Math.toDegrees(HoodConstants.kMaxAngleRad);
    if (table.length == 0) {
      return new Result(0.0, 0.0, 0.0, 0.0);
    }
    if (distanceMeters <= table[0].distanceMeters()) {
      TableEntry e = table[0];
      double exitDeg = MathUtil.clamp(e.hoodAngleDeg(), minDeg, maxDeg);
      double cmdDeg = MathUtil.clamp(e.commandedHoodAngleDeg(), minDeg, maxDeg);
      return new Result(
          Units.degreesToRadians(exitDeg),
          Units.degreesToRadians(cmdDeg),
          e.flywheelRpm(),
          e.timeOfFlightSec());
    }
    TableEntry last = table[table.length - 1];
    if (distanceMeters >= last.distanceMeters()) {
      double exitDeg = MathUtil.clamp(last.hoodAngleDeg(), minDeg, maxDeg);
      double cmdDeg = MathUtil.clamp(last.commandedHoodAngleDeg(), minDeg, maxDeg);
      return new Result(
          Units.degreesToRadians(exitDeg),
          Units.degreesToRadians(cmdDeg),
          last.flywheelRpm(),
          last.timeOfFlightSec());
    }
    int lower = 0;
    while (lower + 1 < table.length && table[lower + 1].distanceMeters() < distanceMeters) {
      lower++;
    }
    TableEntry lo = table[lower];
    TableEntry hi = table[lower + 1];
    double t =
        MathUtil.inverseInterpolate(lo.distanceMeters(), hi.distanceMeters(), distanceMeters);
    double exitDeg = MathUtil.interpolate(lo.hoodAngleDeg(), hi.hoodAngleDeg(), t);
    exitDeg = MathUtil.clamp(exitDeg, minDeg, maxDeg);
    double cmdDeg = MathUtil.interpolate(lo.commandedHoodAngleDeg(), hi.commandedHoodAngleDeg(), t);
    cmdDeg = MathUtil.clamp(cmdDeg, minDeg, maxDeg);
    double rpm = MathUtil.interpolate(lo.flywheelRpm(), hi.flywheelRpm(), t);
    double tof = MathUtil.interpolate(lo.timeOfFlightSec(), hi.timeOfFlightSec(), t);
    return new Result(Units.degreesToRadians(exitDeg), Units.degreesToRadians(cmdDeg), rpm, tof);
  }
}
