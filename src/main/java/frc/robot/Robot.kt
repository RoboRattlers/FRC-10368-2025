// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.Constants.OperatorConstants
import frc.robot.subsystems.swervedrive.CoralgaeSubsystem
import frc.robot.subsystems.swervedrive.ElevatorSubsystem
import frc.robot.subsystems.swervedrive.PivotSubsystem
import frc.robot.subsystems.swervedrive.SwerveSubsystem
import swervelib.SwerveInputStream
import java.io.File

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

    private val driver1: CommandXboxController = CommandXboxController(0)
    private val driver2: CommandXboxController = CommandXboxController(1)

    private var disabledTimer = Timer()

    private val swerve = SwerveSubsystem(File(Filesystem.getDeployDirectory(), "swerve"))
    private val elevator = ElevatorSubsystem()
    private val pivot = PivotSubsystem()
    private val coralgae = CoralgaeSubsystem()

    init {
        instance = this
        DriverStation.silenceJoystickConnectionWarning(true)

        // set up swerve controls
        run {
            val driveInputStreams = object {
                val angularVelocity = SwerveInputStream.of(
                    swerve.swerveDrive,
                    { driver1.leftY },
                    { driver1.leftX })
                    .withControllerRotationAxis { driver1.rightX }
                    .deadband(OperatorConstants.DEADBAND)
                    .scaleTranslation(0.8)
                    .allianceRelativeControl(true)

                val directAngle = angularVelocity
                    .copy()
                    .withControllerHeadingAxis({ driver1.rightX }, { driver1.rightY })
                    .headingWhile(true)
            }

            swerve.defaultCommand = swerve.driveFieldOriented(driveInputStreams.angularVelocity)

            driver1.a().onTrue(runOnce(swerve::zeroGyroWithAlliance))
        }

        // set up elevator controls
        run {
            driver1.povUp().onTrue(elevator.toPosCommand(1.0))
            driver1.start().onTrue(elevator.toPosCommand(0.73))
            driver1.povLeft().onTrue(elevator.toPosCommand(0.61))
            driver1.povRight().onTrue(elevator.toPosCommand(0.26))
            driver1.povDown().onTrue(elevator.toPosCommand(0.0))
        }

        // set up pivot controls
        run {
            driver1.x().onTrue(pivot.toPosCommand(340.0))
            driver1.y().onTrue(pivot.toPosCommand(700.0))
            driver1.b().onTrue(pivot.toPosCommand(1580.0))
            driver1.back().onTrue(pivot.toPosCommand(1570.0))
        }

        // set up end effector controls
        run {

            driver1.leftTrigger().whileTrue(coralgae.algaeIntakeCommand(1.0))
            driver1.rightTrigger().whileTrue(coralgae.algaeShootCommand(1.0))

            driver1.leftStick().whileTrue(coralgae.coralIntakeCommand(0.5))
            driver1.leftBumper().whileTrue(coralgae.coralIntakeCommand(0.1))
            driver1.rightBumper().whileTrue(coralgae.coralIntakeCommand(-0.1))

        }

    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        pivot.lowerBound = if (elevator.currentHeight < 0.05) 0.0 else 300.0
        pivot.upperBound = if (elevator.currentHeight >= 0.25) 1680.0 else 900.0
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
        pivot.toPosCommand(400.0).schedule()
    }
    override fun autonomousInit() {
        swerve.setMotorBrake(true)
        swerve.getAutonomousCommand("Leave").schedule()
    }

}
