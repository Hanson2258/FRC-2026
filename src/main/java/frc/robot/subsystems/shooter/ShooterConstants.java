// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/** Constants for the shooter assembly (turret position on robot, camera on turret, etc.). */
public final class ShooterConstants {

  private ShooterConstants() {}

  /** Transform from robot center to turret pivot. +X = forward, +Y = left, +Z = up (meters). */
  public static final Transform3d robotToTurret =
      new Transform3d(0.27686, -0.0508, 0.6, Rotation3d.kZero);

  /** Transform from turret pivot to camera (when camera is on turret). */
  public static final Transform3d turretToCamera =
      new Transform3d(
          0.0, 0.0, 0.0, new Rotation3d(0.0, Math.toRadians(-22.2), Math.PI));
}
