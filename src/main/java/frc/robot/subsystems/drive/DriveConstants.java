// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.8;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(20.75);
  public static final double wheelBase = Units.inchesToMeters(20.75);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Device CAN IDs
  public static final int pigeonCanId = 24;

  public static final int frontLeftDriveCanId = 19;
  public static final int backLeftDriveCanId = 22;
  public static final int frontRightDriveCanId = 29;
  public static final int backRightDriveCanId = 26;

  public static final int frontLeftTurnCanId = 18;
  public static final int backLeftTurnCanId = 21;
  public static final int frontRightTurnCanId = 28;
  public static final int backRightTurnCanId = 25;

  public static final int frontLeftEncoderCanId = 20;
  public static final int backLeftEncoderCanId = 23;
  public static final int frontRightEncoderCanId = 30;
  public static final int backRightEncoderCanId = 27;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2.193);
  public static final double driveMotorReduction = 6.12;
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 150 / 7;
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // relative turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  public static final boolean frontLeftTurnEncoderInverted = false;
  public static final boolean frontRightTurnEncoderInverted = false;
  public static final boolean backLeftTurnEncoderInverted = false;
  public static final boolean backRightTurnEncoderInverted = false;

  // can coder configuration
  public static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

  public static final double frontLeftEncoderOffset = (0.15234375);
  public static final double frontRightEncoderOffset = (-0.4873046875);
  public static final double backLeftEncoderOffset = (-0.219482421875);
  public static final double backRightEncoderOffset = (0.17236328125);

  public static final boolean frontLeftEncoderInverted = false;
  public static final boolean frontRightEncoderInverted = false;
  public static final boolean backLeftEncoderInverted = false;
  public static final boolean backRightEncoderInverted = false;

  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
