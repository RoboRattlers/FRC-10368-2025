// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.run
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.Constants.OperatorConstants
import frc.robot.subsystems.swervedrive.CoralgaeSubsystem
import frc.robot.subsystems.swervedrive.ElevatorSubsystem
import frc.robot.subsystems.swervedrive.PivotSubsystem
import frc.robot.subsystems.swervedrive.SwerveSubsystem
import swervelib.SwerveInputStream
import java.io.File
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

enum class DriveMode {
    DRIVING,
    CORAL_GROUND_INTAKE,
    ALGAE_SCORING_INTAKE,
    CORAL_SCORING_HP_INTAKE,
}

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

    private var disabledTimer = Timer()

    val swerve = SwerveSubsystem(File(Filesystem.getDeployDirectory(), "swerve"))
    val elevator = ElevatorSubsystem()
    val pivot = PivotSubsystem()
    val coralgae = CoralgaeSubsystem()
    val driver1: CommandXboxController = CommandXboxController(0)
    val driver2: CommandXboxController = CommandXboxController(1)

    val dpadMagnitude = 0.2
    val dpadLeft = { driver1.povUpLeft().asBoolean || driver1.povLeft().asBoolean || driver1.povDownLeft().asBoolean }
    val dpadRight = { driver1.povUpRight().asBoolean || driver1.povRight().asBoolean || driver1.povDownRight().asBoolean }
    val dpadUp = { driver1.povUpLeft().asBoolean || driver1.povUp().asBoolean || driver1.povUpRight().asBoolean }
    val dpadDown = { driver1.povDownLeft().asBoolean || driver1.povDown().asBoolean || driver1.povDownRight().asBoolean }

    val dpadVertical = { if (dpadUp()) dpadMagnitude else 0.0 - if (dpadDown()) dpadMagnitude else 0.0 }
    val dpadHorizontal = { ( if (dpadRight()) dpadMagnitude else 0.0 - if (dpadLeft()) dpadMagnitude else 0.0 ) }

    private val driveInputStreams = object {
        val angularVelocity = SwerveInputStream.of(
            swerve.swerveDrive,
            { driver1.leftY + cos(swerve.heading.radians) * dpadVertical.invoke() + sin(swerve.heading.radians) * dpadHorizontal.invoke()
                     },
            { driver1.leftX + sin(swerve.heading.radians) * dpadVertical.invoke() - cos(swerve.heading.radians) * dpadHorizontal.invoke() })
            .withControllerRotationAxis { driver1.rightX }
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true)
    }

    var currentDriveMode = DriveMode.DRIVING
    var modeSelectionIndex = 0 // used e.g. to select heights in coral scoring mode, or to select intake style in algae intake mode
    var modeSelectionLowerBound = 0
    var modeSelectionUpperBound = 0
    lateinit var driveModeSetup: () -> Unit

    var elevatorPeriodic: () -> Unit = {}
    var pivotPeriodic: () -> Unit = {}

    init {
        instance = this
        DriverStation.silenceJoystickConnectionWarning(true)

        // set up swerve controls
        run {

            swerve.defaultCommand = swerve.driveFieldOriented(driveInputStreams.angularVelocity)

            driver1.back().onTrue(runOnce(swerve::zeroGyroWithAlliance))
            driver1.a().toggleOnTrue(
                swerve.driveFieldOriented(
                    driveInputStreams.angularVelocity
                        .copy()
                        .scaleTranslation(0.25)
                        .scaleRotation(0.4)
                )
            )
        }

        elevator.defaultCommand = run({ elevatorPeriodic() }, elevator)
        pivot.defaultCommand = run({pivotPeriodic()}, pivot)

        driver1.leftBumper().onTrue(runOnce({ modeSelectionIndex = clamp(modeSelectionIndex - 1, modeSelectionLowerBound, modeSelectionUpperBound) }))
        driver1.rightBumper().onTrue(runOnce({ modeSelectionIndex = clamp(modeSelectionIndex + 1, modeSelectionLowerBound, modeSelectionUpperBound) }))

        // driving mode controls
        run {
            val inMode = { currentDriveMode == DriveMode.DRIVING }
            driveModeSetup = setup@{
                currentDriveMode = DriveMode.DRIVING
                elevatorPeriodic = {
                  elevator.setpoint = if (pivot.currentPos > 700.0) 0.5 else 0.0
                }
                pivotPeriodic = {
                    pivot.setpoint = if (elevator.currentPos > 0.1) 500.0 else 200.0
                }
                elevator.toPosCommand(0.0).schedule()
                pivot.toPosCommand(200.0).schedule()
            }
            driver1.start().onTrue(runOnce(driveModeSetup))
            driver1.leftTrigger()
                .and(driver1.rightTrigger())
                .and(inMode)
                .and { elevator.currentPos < 0.1 }
                .onTrue(pivot.autoHomeCommand())
        }

        // algae intake mode controls
        run {
            val inMode = { currentDriveMode == DriveMode.ALGAE_SCORING_INTAKE }
            val setup = setup@{

                if (inMode()) {
                    driveModeSetup()
                    return@setup
                }

                currentDriveMode = DriveMode.ALGAE_SCORING_INTAKE
                modeSelectionIndex = 1
                modeSelectionLowerBound = 0
                modeSelectionUpperBound = 2

                elevatorPeriodic =
                    { when (modeSelectionIndex) {
                        0 -> elevator.setpoint = if (driver1.leftTrigger().asBoolean) 0.3 else 0.4
                        1 -> elevator.setpoint = 0.3
                        2 -> elevator.setpoint = 0.65
                    } }
                    elevator

                pivotPeriodic = { when (modeSelectionIndex) {
                        0 -> pivot.setpoint = 1610.0
                        else -> pivot.setpoint = 700.0
                    } }

            }
            driver1.b().onTrue(runOnce(setup))
            driver1.leftTrigger().and(driver1.rightTrigger().negate()).and(inMode)
                .whileTrue(coralgae.algaeIntakeCommand(1.0)) // intake
            driver1.rightTrigger().and(driver1.leftTrigger().negate()).and(inMode)
                .whileTrue(coralgae.algaeShootCommand(1.0)) // hard outtake
            driver1.leftTrigger().and(driver1.rightTrigger()).and(inMode)
                .whileTrue(coralgae.algaeIntakeCommand(-0.5)) // soft outtake
        }

        // coral scoring mode controls
        run {
            val inMode = { currentDriveMode == DriveMode.CORAL_SCORING_HP_INTAKE }
            val setup = setup@{

                if (inMode()) {
                    driveModeSetup()
                    return@setup
                }

                modeSelectionLowerBound = 0
                modeSelectionUpperBound = 3

                if (currentDriveMode == DriveMode.ALGAE_SCORING_INTAKE) {
                    modeSelectionIndex = max(modeSelectionIndex, 1)
                } else {
                    modeSelectionIndex = 1
                }
                currentDriveMode = DriveMode.CORAL_SCORING_HP_INTAKE

                elevatorPeriodic = { when (modeSelectionIndex) {
                        0 -> elevator.setpoint = 0.63
                        1 -> elevator.setpoint = 0.25
                        2 -> elevator.setpoint = 0.65
                        3 -> elevator.setpoint = 1.0
                    } }

                pivotPeriodic = { when (modeSelectionIndex) {
                        0 -> pivot.setpoint = 1600.0
                        1 -> pivot.setpoint = 700.0
                        2 -> pivot.setpoint = 700.0
                        3 -> pivot.setpoint = 340.0
                    } }

            }
            driver1.x().onTrue(runOnce(setup))
            driver1.leftTrigger().and(driver1.rightTrigger().negate()).and(inMode)
                .whileTrue(coralgae.coralIntakeCommand(0.1)) // intake
            driver1.rightTrigger().and(driver1.leftTrigger().negate()).and(inMode)
                .whileTrue(coralgae.coralIntakeCommand(-0.1)) // outtake
            driver1.leftTrigger().and(driver1.rightTrigger()).and(inMode)
                .whileTrue(coralgae.coralIntakeCommand(0.5)) // hard intake
        }


        // set up elevator controls
        /*run {
            driver1.povUp().onTrue(elevator.toPosCommand(1.0))
            driver1.start().onTrue(elevator.toPosCommand(0.73))
            driver1.povLeft().onTrue(elevator.toPosCommand(0.61))
            driver1.rightStick().onTrue(elevator.toPosCommand(0.33))
            driver1.povRight().onTrue(elevator.toPosCommand(0.26))
            driver1.povDown().onTrue(elevator.toPosCommand(0.0))
        }

        // set up pivot controls
        run {
            driver1.x().onTrue(pivot.toPosCommand(340.0))
            driver1.y().onTrue(pivot.toPosCommand(700.0))
            driver1.b().onTrue(pivot.toPosCommand(1680.0))
            driver1.back().onTrue(pivot.toPosCommand(1570.0))
            driver1.leftTrigger().and(driver1.rightTrigger()).onTrue(pivot.autoHomeCommand())
        }

        // set up end effector controls
        run {

            driver1.leftTrigger().whileTrue(coralgae.algaeIntakeCommand(1.0))
            driver1.rightTrigger().whileTrue(coralgae.algaeShootCommand(1.0))

            driver1.leftStick().whileTrue(coralgae.coralIntakeCommand(0.5))
            driver1.leftBumper().and(driver1.rightBumper().negate()).whileTrue(coralgae.coralIntakeCommand(0.1))
            driver1.rightBumper().and(driver1.leftBumper().negate()).whileTrue(coralgae.coralIntakeCommand(-0.1))

        }*/

    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        pivot.lowerBound = if (elevator.currentPos < 0.05) 0.0 else 300.0
        pivot.upperBound = if (elevator.currentPos >= 0.25) 1680.0 else 900.0
        elevator.lowerBound = if (pivot.currentPos > 900.0) 0.3 else 0.0
        elevator.upperBound = if (pivot.currentPos > 300.0) 1.0 else 0.05
        modeSelectionIndex = clamp(modeSelectionIndex, modeSelectionLowerBound, modeSelectionUpperBound)
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
        driveModeSetup()
    }
    override fun autonomousInit() {
        swerve.setMotorBrake(true)
        swerve.getAutonomousCommand("Leave").schedule()
    }

}
