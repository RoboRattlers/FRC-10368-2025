// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import swervelib.math.Matter;

public final class Constants {

	public static final double TIMESTEP = 0.05;
	public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
	public static final Matter CHASSIS =
			new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
	public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
	public static final double MAX_SPEED = Units.feetToMeters(14.5);
	// Maximum speed of the robot in meters per second, used to limit acceleration.

	public static final class DrivebaseConstants {

		// Hold time on motor brakes when disabled
		public static final double WHEEL_LOCK_TIME = 10; // seconds
	}

	public static class OperatorConstants {

		// Joystick Deadband
		public static final double DEADBAND = 0.035;
		public static final double LEFT_Y_DEADBAND = 0.1;
		public static final double RIGHT_X_DEADBAND = 0.1;
		public static final double TURN_CONSTANT = 6;
	}

	public static class CanId {
		public static final int ELEVATOR_TOP = 10;

		public static final int ELEVATOR_BOTTOM = 11;

		public static final int PIVOT = 12;

		public static final int CORALGAE_INNER = 13;
		public static final int CORALGAE_OUTER_TOP = 14;

		public static final int CORALGAE_OUTER_BOTTOM = 15;

		public static final int CORINTAKE_PIVOT = 30;
		public static final int CORINTAKE_INTAKE = 31;
	}
}
