// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PhoenixUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    // Negate to match NavX convention used elsewhere
    Rotation2d simYaw = gyroSimulation.getGyroReading();
    inputs.yawPosition = new Rotation2d(-simYaw.getRadians());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(
        -gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

    inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryYawPositions = Arrays.stream(gyroSimulation.getCachedGyroReadings())
        .map(r -> new Rotation2d(-r.getRadians()))
        .toArray(Rotation2d[]::new);
  }
}