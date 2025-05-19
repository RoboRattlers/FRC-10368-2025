// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.hal.AllianceStationID
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.run
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.Constants.OperatorConstants
import frc.robot.subsystems.swervedrive.*
import frc.robot.util.MathUtils.wrapAngle
import swervelib.SwerveInputStream
import java.io.File
import kotlin.math.*

enum class DriveMode {
    DRIVING,
    CORAL_GROUND_INTAKE,
    ALGAE_SCORING_INTAKE,
    CORAL_SCORING_HP_INTAKE,
}

data class ElepivotSetpoint(val elevatorPos: Double, val pivotPos: Double)

data object Setpoints {
    val Stowed = ElepivotSetpoint(0.0, 0.0)
    val AlgaePreGroundIntake = ElepivotSetpoint(0.4, 700.0)
    val AlgaeGroundIntake = ElepivotSetpoint(0.32, 1610.0)
    val AlgaeProcessor = ElepivotSetpoint(0.0, 700.0)
    val AlgaeL2 = ElepivotSetpoint(0.3, 700.0)
    val AlgaeL3 = ElepivotSetpoint(0.65, 700.0)
    val AlgaeShot = ElepivotSetpoint(1.0, 300.0)
    val CoralHPIntake = ElepivotSetpoint(0.63, 1600.0)
    val CoralL2 = ElepivotSetpoint(0.25, 700.0)
    val CoralL3 = ElepivotSetpoint(0.65, 700.0)
    val CoralL4 = ElepivotSetpoint(1.0, 700.0)
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
    val corintakePivot = CorintakePivotSubsystem()
    val corintake = CorintakeSubsystem()

    lateinit var autoChooser: SendableChooser<Command>

    val driver1: CommandXboxController = CommandXboxController(0)
    val driver2: CommandXboxController = CommandXboxController(1)

    val dpadMagnitude = 0.2
    val dpadLeft = { driver1.povUpLeft().asBoolean || driver1.povLeft().asBoolean || driver1.povDownLeft().asBoolean }
    val dpadRight = { driver1.povUpRight().asBoolean || driver1.povRight().asBoolean || driver1.povDownRight().asBoolean }
    val dpadUp = { driver1.povUpLeft().asBoolean || driver1.povUp().asBoolean || driver1.povUpRight().asBoolean }
    val dpadDown = { driver1.povDownLeft().asBoolean || driver1.povDown().asBoolean || driver1.povDownRight().asBoolean }

    val dpadVertical = { if (dpadUp()) dpadMagnitude else 0.0 - if (dpadDown()) dpadMagnitude else 0.0 }
    val dpadHorizontal = { ( if (dpadRight()) dpadMagnitude else 0.0 - if (dpadLeft()) dpadMagnitude else 0.0 ) }

    val forwardSupplier = { -driver1.leftY + cos(swerve.heading.radians) * dpadVertical.invoke() + sin(swerve.heading.radians) * dpadHorizontal.invoke() }
    val leftSupplier = { -driver1.leftX + sin(swerve.heading.radians) * dpadVertical.invoke() - cos(swerve.heading.radians) * dpadHorizontal.invoke() }

    private val driveInputStreams = object {
        val angularVelocity = SwerveInputStream.of(
            swerve.swerveDrive,
            forwardSupplier,
            leftSupplier)
            .withControllerRotationAxis { -driver1.rightX }
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true)

        var targetAngle = 0.0
        private val deadzone = 0.4
        val anglePid = angularVelocity.copy()
            .withControllerRotationAxis {
                targetAngle = if (driver1.rightX.pow(2.0) + driver1.rightY.pow(2.0) < deadzone.pow(2.0))
                    targetAngle else atan2(driver1.rightX, driver1.rightY)
                return@withControllerRotationAxis wrapAngle((targetAngle - swerve.heading.radians)) * -0.75
            }
    }

    var currentDriveMode = DriveMode.DRIVING
    var modeSelectionIndex = 0 // used e.g. to select heights in coral scoring mode, or to select intake style in algae intake mode
    var modeSelectionLowerBound = 0
    var modeSelectionUpperBound = 0
    lateinit var driveModeSetup: () -> Unit
    lateinit var algaeIntakeScoreModeSetup: () -> Unit
    lateinit var coralIntakeScoreModeSetup: () -> Unit

    var elevatorPeriodic: () -> Unit = {}
    var pivotPeriodic: () -> Unit = {}
    var corintakePivotPeriodic: () -> Unit = {}

    init {
        instance = this
        DriverStation.silenceJoystickConnectionWarning(true)
        // set up swerve controls
        run {

            swerve.defaultCommand = swerve.driveFieldOriented(driveInputStreams.angularVelocity)

            driver1.back().onTrue(runOnce(swerve::zeroGyroWithAlliance))

        }

        elevator.defaultCommand = run({ elevatorPeriodic() }, elevator)
        pivot.defaultCommand = run({pivotPeriodic()}, pivot)
        corintakePivot.defaultCommand = run({corintakePivotPeriodic()}, corintakePivot)

        driver1.rightStick().onTrue(runOnce({driveInputStreams.targetAngle = swerve.heading.radians}))
        driver1.rightStick().toggleOnTrue(swerve.driveFieldOriented(driveInputStreams.anglePid))
        val minMagnitude = 0.04
        driver1.a().and(driver1.leftBumper())
            .onTrue(
                swerve.reefAutoAlignCommand(AutoAlignSide.LEFT)
                    .until { abs(forwardSupplier()) + abs(leftSupplier()) > minMagnitude }
            )
        driver1.a().and(driver1.rightBumper())
            .onTrue(
                swerve.reefAutoAlignCommand(AutoAlignSide.RIGHT)
                    .until { abs(forwardSupplier()) + abs(leftSupplier()) > minMagnitude }
            )
        driver1.a()
            .and(driver1.leftBumper().negate())
            .and(driver1.rightBumper().negate())
            .onTrue(
                swerve.reefAutoAlignCommand(AutoAlignSide.CENTER)
                    .until { abs(forwardSupplier()) + abs(leftSupplier()) > minMagnitude }
            )
        driver1.leftBumper().and(driver1.a().negate())
            .onTrue(runOnce({ modeSelectionIndex = clamp(modeSelectionIndex - 1, modeSelectionLowerBound, modeSelectionUpperBound) }))
        driver1.rightBumper().and(driver1.a().negate())
            .onTrue(runOnce({ modeSelectionIndex = clamp(modeSelectionIndex + 1, modeSelectionLowerBound, modeSelectionUpperBound) }))

        // driving mode controls
        run {
            val inMode = { currentDriveMode == DriveMode.DRIVING }
            driveModeSetup = setup@{
                currentDriveMode = DriveMode.DRIVING
                elevatorPeriodic = {
                  elevator.setpoint = if (pivot.currentPos > 700.0) 0.5 else 0.0
                }
                pivotPeriodic = {
                    pivot.setpoint = if (elevator.currentPos > 0.1) 500.0 else 0.0
                }
                corintakePivotPeriodic = {
                    corintakePivot.setpoint = when (modeSelectionIndex) {
                        0 -> 0.0
                        1 -> 1.0
                        else -> 1.0
                    }
                }
                elevator.toPosCommand(0.0).schedule()
                pivot.toPosCommand(200.0).schedule()

                modeSelectionIndex = 1
                modeSelectionLowerBound = 0
                modeSelectionUpperBound = 1
            }
            driver1.start().onTrue(runOnce(driveModeSetup))
            driver2.start().onTrue(runOnce(driveModeSetup))
            driver1.leftTrigger()
                .and(driver1.rightTrigger())
                .and(inMode)
                .and { elevator.currentPos < 0.1 }
                .onTrue(pivot.autoHomeCommand())
            driver1.leftTrigger()
                .and(inMode)
                .and { modeSelectionIndex == 0 }
                .whileTrue( corintake.runIntakeCommand(1.0) )
            driver1.rightTrigger()
                .and(inMode)
                .and { modeSelectionIndex == 0 }
                .whileTrue( corintake.runIntakeCommand(-1.0) )
            driver1.leftTrigger()
                .and(inMode)
                .and { modeSelectionIndex == 1 }
                .whileTrue( ParallelCommandGroup(
                    corintake.runIntakeCommand(0.1),
                    coralgae.coralIntakeCommand(0.1)
                ) )
            driver2.leftTrigger()
                .and(inMode)
                .and { modeSelectionIndex == 1 }
                .whileTrue( ParallelCommandGroup(
                    corintake.runIntakeCommand(0.1),
                    coralgae.coralIntakeCommand(0.1)
                ) )
            driver1.rightTrigger()
                .and(inMode)
                .and { modeSelectionIndex == 1 }
                .whileTrue( ParallelCommandGroup(
                    corintake.runIntakeCommand(-1.0),
                    coralgae.coralIntakeCommand(-0.1)
                ) )
        }

        // algae intake mode controls
        run {
            val inMode = { currentDriveMode == DriveMode.ALGAE_SCORING_INTAKE }
            algaeIntakeScoreModeSetup = setup@{

                corintakePivotPeriodic = { corintakePivot.setpoint = 1.0 }


                currentDriveMode = DriveMode.ALGAE_SCORING_INTAKE
                modeSelectionIndex = 1
                modeSelectionLowerBound = -1
                modeSelectionUpperBound = 3

                elevatorPeriodic =
                    { when (modeSelectionIndex) {
                        -1 -> elevator.setpoint = if (driver1.leftTrigger().asBoolean)
                            Setpoints.AlgaeGroundIntake.elevatorPos
                            else Setpoints.AlgaePreGroundIntake.elevatorPos
                        0 -> elevator.setpoint = Setpoints.AlgaeProcessor.elevatorPos
                        1 -> elevator.setpoint = Setpoints.AlgaeL2.elevatorPos
                        2 -> elevator.setpoint = Setpoints.AlgaeL3.elevatorPos
                        3 -> elevator.setpoint = Setpoints.AlgaeShot.elevatorPos
                    } }
                    elevator

                pivotPeriodic = { when (modeSelectionIndex) {
                        -1 -> pivot.setpoint = Setpoints.AlgaeGroundIntake.pivotPos
                        3 -> pivot.setpoint = Setpoints.AlgaeShot.pivotPos
                        else -> pivot.setpoint = Setpoints.AlgaeL2.pivotPos
                    } }

            }
            driver1.b().onTrue(runOnce({
                if (inMode()) {
                    driveModeSetup()
                    return@runOnce
                }
                algaeIntakeScoreModeSetup()
            }))
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
            coralIntakeScoreModeSetup = setup@{

                corintakePivotPeriodic = { corintakePivot.setpoint = 1.0 }

                modeSelectionLowerBound = 0
                modeSelectionUpperBound = 3

                if (currentDriveMode == DriveMode.ALGAE_SCORING_INTAKE) {
                    modeSelectionIndex = max(modeSelectionIndex, 1)
                } else {
                    modeSelectionIndex = 1
                }
                currentDriveMode = DriveMode.CORAL_SCORING_HP_INTAKE

                elevatorPeriodic = { when (modeSelectionIndex) {
                        0 -> elevator.setpoint = Setpoints.CoralHPIntake.elevatorPos
                        1 -> elevator.setpoint = Setpoints.CoralL2.elevatorPos
                        2 -> elevator.setpoint = Setpoints.CoralL3.elevatorPos
                        3 -> elevator.setpoint = Setpoints.CoralL4.elevatorPos
                    } }

                pivotPeriodic = { when (modeSelectionIndex) {
                        0 -> pivot.setpoint = Setpoints.CoralHPIntake.pivotPos
                        1 -> pivot.setpoint = Setpoints.CoralL2.pivotPos
                        2 -> pivot.setpoint = Setpoints.CoralL3.pivotPos
                        3 -> pivot.setpoint = Setpoints.CoralL4.pivotPos
                    } }

            }
            driver1.x().onTrue(runOnce({
                if (inMode()) {
                    driveModeSetup()
                    return@runOnce
                }
                coralIntakeScoreModeSetup()
            }))
            driver1.leftTrigger().and(driver1.rightTrigger().negate()).and(inMode)
                .whileTrue(coralgae.coralIntakeCommand(0.1)) // intake
            driver1.rightTrigger().and(driver1.leftTrigger().negate()).and(inMode)
                .whileTrue(coralgae.coralIntakeCommand(-0.1)) // outtake
            driver1.leftTrigger().and(driver1.rightTrigger()).and(inMode)
                .whileTrue(coralgae.coralIntakeCommand(0.5)) // hard intake
        }

        driver2.rightStick().onTrue(runOnce( {
            algaeIntakeScoreModeSetup()
            modeSelectionIndex = -1
        } ))
        driver2.a().onTrue(runOnce( {
            algaeIntakeScoreModeSetup()
            modeSelectionIndex = 0
        } ))
        driver2.b().onTrue(runOnce( {
            algaeIntakeScoreModeSetup()
            modeSelectionIndex = 1
        } ))
        driver2.x().onTrue(runOnce( {
            algaeIntakeScoreModeSetup()
            modeSelectionIndex = 2
        } ))
        driver2.y().onTrue(runOnce( {
            algaeIntakeScoreModeSetup()
            modeSelectionIndex = 3
        } ))

        driver2.povDown().onTrue(runOnce( {
            coralIntakeScoreModeSetup()
            modeSelectionIndex = 0
        } ))
        driver2.povRight().onTrue(runOnce( {
            coralIntakeScoreModeSetup()
            modeSelectionIndex = 1
        } ))
        driver2.povLeft().onTrue(runOnce( {
            coralIntakeScoreModeSetup()
            modeSelectionIndex = 2
        } ))
        driver2.povUp().onTrue(runOnce( {
            coralIntakeScoreModeSetup()
            modeSelectionIndex = 3
        } ))

        driver2.leftTrigger().whileTrue(coralgae.coralIntakeCommand(0.1))
        driver2.rightTrigger().whileTrue(coralgae.coralIntakeCommand(-0.1))

        NamedCommands.registerCommands(mutableMapOf(
            "Algae Processor Setpoint" to toSetpointCommand(Setpoints.AlgaeProcessor),
            "Algae L2 Setpoint" to toSetpointCommand(Setpoints.AlgaeL2),
            "Algae L3 Setpoint" to toSetpointCommand(Setpoints.AlgaeL3),
            "Algae Shot Setpoint" to toSetpointCommand(Setpoints.AlgaeShot),
            "Coral L4 Setpoint" to toSetpointCommand(Setpoints.CoralL4),
            "Stowed Setpoint" to toSetpointCommand(Setpoints.Stowed),
            "Coral Outtake" to coralgae.coralIntakeCommand(0.5).withTimeout(0.5),
            "Coral Intake" to coralgae.coralIntakeCommand(0.1).withTimeout(0.5),
            "Coral Reversal" to coralgae.coralIntakeCommand(-0.1).withTimeout(0.5),
            "Coral Hard Reversal" to coralgae.coralIntakeCommand(-0.5).withTimeout(0.5),
            "Algae Intake" to coralgae.algaeIntakeCommand(1.0).withTimeout(3.0),
            "Algae Shot" to coralgae.algaeShootCommand(1.0),
            "Pivot Home" to pivot.autoHomeCommand(0.5)
        ))

        swerve.setupPathPlanner()
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        pivot.lowerBound = if (elevator.currentPos < 0.05) 0.0 else 250.0
        pivot.upperBound = if (elevator.currentPos >= 0.25) 1680.0 else 900.0
        elevator.lowerBound = if (pivot.currentPos > 900.0) 0.3 else 0.0
        elevator.upperBound = if (pivot.currentPos > 230.0) 1.0 else 0.05
        modeSelectionIndex = clamp(modeSelectionIndex, modeSelectionLowerBound, modeSelectionUpperBound)
    }
    override fun disabledInit() {
        swerve.setMotorBrake(true)
        disabledTimer.reset()
        disabledTimer.start()
        elevatorPeriodic = {}
        pivotPeriodic = {}
        corintakePivotPeriodic = {}
        corintakePivot.setpoint = 1.0
    }
    override fun disabledPeriodic() {
        if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME) && elevator.currentPos < 0.1) {
            swerve.setMotorBrake(false)
            elevator.setMotorBrake(false)
            pivot.setMotorBrake(false)
            disabledTimer.reset()
            disabledTimer.stop()
        }

        CommandScheduler.getInstance().cancelAll() // no reason any commands should be scheduled while disabled, but for safety
    }
    override fun testInit() {
        swerve.setMotorBrake(true)
        elevator.setMotorBrake(true)
        pivot.setMotorBrake(true)
        driveModeSetup()
    }
    override fun teleopInit() {
        swerve.setMotorBrake(true)
        elevator.setMotorBrake(true)
        pivot.setMotorBrake(true)
        driveModeSetup()
    }
    override fun autonomousInit() {
        swerve.setMotorBrake(true)
        elevator.setMotorBrake(true)
        pivot.setMotorBrake(true)
        autoChooser.selected.schedule()
    }
    override fun simulationPeriodic() {
    }

    fun toSetpointCommand(setpoint: ElepivotSetpoint): Command {
        return ParallelCommandGroup(
            elevator.toPosCommand(setpoint.elevatorPos),
            pivot.toPosCommand(setpoint.pivotPos)
        )
    }

    fun driveModeCommand(): Command {
        return Commands.none()
    }

    fun algaeIntakeScoreModeCommand(index: Int = 1): Command {
        return runOnce( {
            algaeIntakeScoreModeSetup()
            modeSelectionIndex = index
        }, elevator, pivot )
    }

    fun coralIntakeScoreModeCommand(index: Int = 1): Command {
        return Commands.none()
    }



}
