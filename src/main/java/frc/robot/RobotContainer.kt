// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.Constants.OperatorConstants
import frc.robot.subsystems.swervedrive.SwerveSubsystem
import swervelib.SwerveInputStream
import java.io.File
import kotlin.math.cos
import kotlin.math.sin

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the [Robot] periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {

    val driverXbox: CommandXboxController = CommandXboxController(0)
    private val swerve = SwerveSubsystem(File(Filesystem.getDeployDirectory(), "swerve"))

    private val DriveInputStreams = object {
        val angularVelocity = SwerveInputStream.of(
            swerve.swerveDrive,
            { -driverXbox.leftY },
            { -driverXbox.leftX })
            .withControllerRotationAxis { driverXbox.rightX }
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true)

        val directAngle = angularVelocity
            .copy()
            .withControllerHeadingAxis({ driverXbox.rightX }, { driverXbox.rightY })
            .headingWhile(true)

        // Derive the heading axis with math!
        val directAngleKeyboard = angularVelocity
            .copy()
            .withControllerHeadingAxis(
                { sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2) },
                { cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2) })
            .headingWhile(true)
            .translationHeadingOffset(true)
            .translationHeadingOffset(Rotation2d.fromDegrees(0.0))
    }

    init {
        configureBindings()
        DriverStation.silenceJoystickConnectionWarning(true)
    }

    private fun configureBindings() {
        swerve.defaultCommand = if (RobotBase.isSimulation())
            swerve.driveFieldOriented(DriveInputStreams.angularVelocity)
                else swerve.driveFieldOriented(DriveInputStreams.directAngleKeyboard)

        driverXbox.a().onTrue(Commands.runOnce({ swerve.zeroGyro() }))
        driverXbox.leftBumper()
            .whileTrue(Commands.run({ swerve.lock() }, swerve))
    }

    val autonomousCommand: Command
        get() = swerve.getAutonomousCommand("New Auto")

    fun setMotorBrake(brake: Boolean) {
        swerve.setMotorBrake(brake)
    }
}
