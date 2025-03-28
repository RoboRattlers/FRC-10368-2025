// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.Constants.OperatorConstants
import frc.robot.subsystems.swervedrive.ElevatorSubsystem
import frc.robot.subsystems.swervedrive.PivotSubsystem
import frc.robot.subsystems.swervedrive.SwerveSubsystem
import swervelib.SwerveInputStream
import java.io.File
import kotlin.math.cos
import kotlin.math.sin

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
class Robot : TimedRobot() {

    companion object {
        lateinit var instance: Robot
            private set
    }

    private val driverXbox: CommandXboxController = CommandXboxController(0)
    private var disabledTimer = Timer()
    private val swerve = SwerveSubsystem(File(Filesystem.getDeployDirectory(), "swerve"))
    private val elevator = ElevatorSubsystem()
    private val pivot = PivotSubsystem()

    init {
        instance = this
        DriverStation.silenceJoystickConnectionWarning(true)

        // set up swerve controls
        run {
            val driveInputStreams = object {
                val angularVelocity = SwerveInputStream.of(
                    swerve.swerveDrive,
                    { -driverXbox.leftY },
                    { -driverXbox.leftX })
                    .withControllerRotationAxis { -driverXbox.rightX }
                    .deadband(OperatorConstants.DEADBAND)
                    .scaleTranslation(0.8)
                    .allianceRelativeControl(true)

                val directAngle = angularVelocity
                    .copy()
                    .withControllerHeadingAxis({ driverXbox.rightX }, { driverXbox.rightY })
                    .headingWhile(true)
            }

            swerve.defaultCommand = swerve.driveFieldOriented(driveInputStreams.angularVelocity)

            driverXbox.a().onTrue(runOnce(swerve::zeroGyro))
            driverXbox.leftBumper().whileTrue(Commands.run(swerve::lock, swerve))
        }

        // set up elevator controls
        run {
            driverXbox.povUp().onTrue(elevator.toPosCommand(0.9))
            driverXbox.povLeft().onTrue(elevator.toPosCommand(0.6))
            driverXbox.povRight().onTrue(elevator.toPosCommand(0.3))
            driverXbox.povDown().onTrue(elevator.toPosCommand(0.0))
        }


        driverXbox.x().onTrue(pivot.toPosCommand(0.0))
        driverXbox.y().onTrue(pivot.toPosCommand(400.0))
        driverXbox.b().onTrue(pivot.toPosCommand(800.0))

    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }
    override fun disabledInit() {
        swerve.setMotorBrake(true)
        disabledTimer.reset()
        disabledTimer.start()
    }
    override fun disabledPeriodic() {
        if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
            swerve.setMotorBrake(false)
            disabledTimer.stop()
            disabledTimer.reset()
        }
        CommandScheduler.getInstance().cancelAll() // no reason any commands should be scheduled while disabled, but for safety
    }
    override fun testInit() {
        swerve.setMotorBrake(true)
    }
    override fun teleopInit() {
        swerve.setMotorBrake(true)
    }
    override fun autonomousInit() {
        swerve.setMotorBrake(true)
        swerve.getAutonomousCommand("Leave").schedule()
    }

}
