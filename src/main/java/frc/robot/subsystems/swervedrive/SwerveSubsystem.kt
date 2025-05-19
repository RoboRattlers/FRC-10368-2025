// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.swervedrive

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.commands.PathfindingCommand
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.util.DriveFeedforwards
import com.pathplanner.lib.util.swerve.SwerveSetpoint
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.Constants
import frc.robot.util.LimelightHelpers
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial
import org.json.simple.parser.ParseException
import swervelib.SwerveController
import swervelib.SwerveDrive
import swervelib.SwerveDriveTest
import swervelib.SwerveModule
import swervelib.math.SwerveMath
import swervelib.parser.SwerveControllerConfiguration
import swervelib.parser.SwerveDriveConfiguration
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity
import java.io.File
import java.io.IOException
import java.util.*
import java.util.concurrent.atomic.AtomicReference
import java.util.function.Consumer
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.pow

enum class AutoAlignSide {
    LEFT,
    CENTER,
    RIGHT
}

class SwerveSubsystem : SubsystemBase {

	val swerveDrive: SwerveDrive

    constructor(directory: File?) {
        val blueAlliance = false
        val startingPose = if (blueAlliance
        ) Pose2d(Translation2d(Units.Meter.of(1.0), Units.Meter.of(4.0)), Rotation2d.fromDegrees(0.0))
        else Pose2d(
            Translation2d(Units.Meter.of(16.0), Units.Meter.of(4.0)), Rotation2d.fromDegrees(180.0)
        )
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects
        // being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH
        try {
            swerveDrive = SwerveParser(directory)
                .createSwerveDrive(Constants.MAX_SPEED, startingPose)
            // Alternative method if you don't want to supply the conversion factor via JSON files.
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
            // angleConversionFactor, driveConversionFactor);
        } catch (e: Exception) {
            throw RuntimeException(e)
        }
        swerveDrive.setHeadingCorrection(
            false
        ) // Heading correction should only be used while controlling the robot via
        // angle.
        swerveDrive.setCosineCompensator(
            false
        ) // !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for
        // simulations since it causes discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(
            true, true,
            0.1
        ) // Correct for skew that gets worse as angular velocity increases. Start with
        // a coefficient of 0.1.
        swerveDrive.setModuleEncoderAutoSynchronize(
            false,
            1.0
        ) // Enable if you want to resynchronize your absolute encoders and motor encoders
        // periodically when they are not moving.
        // swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the
        // internal encoder and push the offsets onto it. Throws warning if not possible
        /*if (visionDriveTest)
		{
		setupPhotonVision();
		// Stop the odometry thread if we are using vision that way we can synchronize updates better.
		swerveDrive.stopOdometryThread();
		}*/
        //setupPathPlanner()
    }

    constructor(driveCfg: SwerveDriveConfiguration?, controllerCfg: SwerveControllerConfiguration?) {
        swerveDrive = SwerveDrive(
            driveCfg,
            controllerCfg,
            Constants.MAX_SPEED,
            Pose2d(Translation2d(Units.Meter.of(2.0), Units.Meter.of(0.0)), Rotation2d.fromDegrees(0.0))
        )
    }

    override fun periodic() {

        val robotYaw: Double = swerveDrive.yaw.radians
        LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0)

        SmartDashboard.putNumber(
            "Top right azimuth rotations",
            swerveDrive.modulePositions[1].angle.rotations
        )
        SmartDashboard.putNumber(
            "Back right azimuth rotations",
            swerveDrive.modulePositions[3].angle.rotations
        )
        SmartDashboard.putNumber(
            "Back left azimuth rotations",
            swerveDrive.modulePositions[2].angle.rotations
        )
        SmartDashboard.putNumber(
            "Front left azimuth rotations",
            swerveDrive.modulePositions[0].angle.rotations
        )
        SmartDashboard.putNumber(
            "Top right drive meters", swerveDrive.modulePositions[1].distanceMeters
        )
        SmartDashboard.putNumber(
            "Back right drive meters", swerveDrive.modulePositions[3].distanceMeters
        )
        SmartDashboard.putNumber(
            "Back left drive meters", swerveDrive.modulePositions[2].distanceMeters
        )
        SmartDashboard.putNumber(
            "Front left drive meters", swerveDrive.modulePositions[0].distanceMeters
        )
        SmartDashboard.putNumber(
            "Swerve heading", swerveDrive.pose.rotation.degrees
        )
        SmartDashboard.putNumber(
            "Swerve X", swerveDrive.pose.x
        )
        SmartDashboard.putNumber(
            "Swerve Y", swerveDrive.pose.y
        )
    }

    override fun simulationPeriodic() {}

    val VALID_REEF_TAGS = listOf(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22)

    fun reefAutoAlignCommand(side: AutoAlignSide): Command {
        var tagId = 0
        val translationKp = 0.05
        val headingKp = 0.05
        // facing tag
        // x+ = right
        // y+ = down
        // z+ = toward me
        val centeringSign = if (side == AutoAlignSide.LEFT) 1.0
        else if (side == AutoAlignSide.RIGHT) -1.0
        else 0.0
        val goalOffset = Pose2d(Translation2d(20.0, 7.0 * centeringSign), Rotation2d(0.0))

        return SequentialCommandGroup( runOnce {
            val tagId = LimelightHelpers.getFiducialID("").toInt()
            if (tagId !in VALID_REEF_TAGS) {
                return@runOnce
            }
        }, run {
            //LimelightHelpers.setPriorityTagID("", tagId)
            if (LimelightHelpers.getFiducialID("").toInt() == tagId) {
                val currentOffset = LimelightHelpers.getTargetPose3d_RobotSpace("")
                    .toPose2d()

                
                swerveDrive.drive(
                    Translation2d(
                        currentOffset.x - goalOffset.x,
                        currentOffset.y - goalOffset.y
                    )
                        .times(translationKp),
                    (currentOffset.rotation.radians - goalOffset.rotation.radians) * headingKp,
                    false,
                    true
                )
            }
        } )
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */
    fun setupPathPlanner() {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        val config: RobotConfig
        try {
            config = RobotConfig.fromGUISettings()

            val enableFeedforward = true
            // Configure AutoBuilder last
            AutoBuilder.configure(
                { this.pose },  // Robot pose supplier
                { initialHolonomicPose: Pose2d? -> this.resetOdometry(initialHolonomicPose) },  // Method to reset odometry (will be called if your auto has a starting pose)
                { this.robotVelocity },  // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                { speedsRobotRelative: ChassisSpeeds?, moduleFeedForwards: DriveFeedforwards ->
                    if (enableFeedforward) {
                        swerveDrive.drive(
                            speedsRobotRelative,
                            swerveDrive.kinematics.toSwerveModuleStates(
                                speedsRobotRelative
                            ),
                            moduleFeedForwards.linearForces()
                        )
                    } else {
                        swerveDrive.setChassisSpeeds(speedsRobotRelative)
                    }
                },  // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                // optionally outputs individual module feedforwards
                PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                    // holonomic drive trains
                    PIDConstants(5.0, 0.0, 0.0),  // Translation PID constants
                    PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config,  // The robot configuration
                {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    val alliance = DriverStation.getAlliance()
                    if (alliance.isPresent) {
                        return@configure alliance.get() == Alliance.Red
                    }
                    false
                },
                this // Reference to this subsystem to set requirements
            )
        } catch (e: Exception) {
            // Handle exception as needed
            e.printStackTrace()
        }

        PathfindingCommand.warmupCommand().schedule()
    }

    /**
     * Get the path follower with events.
     *
     * @param pathName PathPlanner path name.
     * @return [AutoBuilder.followPath] path command.
     */
    fun getAutonomousCommand(pathName: String?): Command {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        val auto = PathPlannerAuto(pathName)
        return auto
    }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target [Pose2d] to go to.
     * @return PathFinding command
     */
    fun driveToPose(pose: Pose2d?): Command {
        // Create the constraints to use while pathfinding
        val constraints = PathConstraints(
            swerveDrive.maximumChassisVelocity,
            4.0,
            swerveDrive.maximumChassisAngularVelocity,
            edu.wpi.first.math.util.Units.degreesToRadians(720.0)
        )

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
            pose,
            constraints,
            Units.MetersPerSecond.of(0.0) // Goal end velocity in meters/sec
        )
    }

    /**
     * Drive with [SwerveSetpointGenerator] from 254, implemented by PathPlanner.
     *
     * @param robotRelativeChassisSpeed Robot relative [ChassisSpeeds] to achieve.
     * @return [Command] to run.
     * @throws IOException    If the PathPlanner GUI settings is invalid
     * @throws ParseException If PathPlanner GUI settings is nonexistent.
     */
    @Throws(IOException::class, ParseException::class)
    private fun driveWithSetpointGenerator(robotRelativeChassisSpeed: Supplier<ChassisSpeeds>): Command {
        val setpointGenerator = SwerveSetpointGenerator(
            RobotConfig.fromGUISettings(), swerveDrive.maximumChassisAngularVelocity
        )
        val prevSetpoint = AtomicReference(
            SwerveSetpoint(
                swerveDrive.robotVelocity,
                swerveDrive.states,
                DriveFeedforwards.zeros(swerveDrive.modules.size)
            )
        )
        val previousTime = AtomicReference<Double>()

        return startRun({ previousTime.set(Timer.getFPGATimestamp()) }, {
            val newTime = Timer.getFPGATimestamp()
            val newSetpoint = setpointGenerator.generateSetpoint(
                prevSetpoint.get(),
                robotRelativeChassisSpeed.get(),
                newTime - previousTime.get()
            )
            swerveDrive.drive(
                newSetpoint.robotRelativeSpeeds(),
                newSetpoint.moduleStates(),
                newSetpoint.feedforwards().linearForces()
            )
            prevSetpoint.set(newSetpoint)
            previousTime.set(newTime)
        })
    }

    /**
     * Drive with 254's Setpoint generator; port written by PathPlanner.
     *
     * @param fieldRelativeSpeeds Field-Relative [ChassisSpeeds]
     * @return Command to drive the robot using the setpoint generator.
     */
    fun driveWithSetpointGeneratorFieldRelative(
        fieldRelativeSpeeds: Supplier<ChassisSpeeds?>
    ): Command {
        try {
            return driveWithSetpointGenerator {
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldRelativeSpeeds.get(), heading
                )
            }
        } catch (e: Exception) {
            DriverStation.reportError(e.toString(), true)
        }
        return Commands.none()
    }

    /**
     * Command to characterize the robot drive motors using SysId
     *
     * @return SysId Drive Command
     */
    fun sysIdDriveMotorCommand(): Command {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setDriveSysIdRoutine(SysIdRoutine.Config(), this, swerveDrive, 12.0, true),
            3.0,
            5.0,
            3.0
        )
    }

    /**
     * Command to characterize the robot angle motors using SysId
     *
     * @return SysId Angle Command
     */
    fun sysIdAngleMotorCommand(): Command {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setAngleSysIdRoutine(SysIdRoutine.Config(), this, swerveDrive),
            3.0,
            5.0,
            3.0
        )
    }

    /**
     * Returns a Command that centers the modules of the SwerveDrive subsystem.
     *
     * @return a Command that centers the modules of the SwerveDrive subsystem
     */
    fun centerModulesCommand(): Command {
        return run { Arrays.asList(*swerveDrive.modules).forEach(Consumer { it: SwerveModule -> it.setAngle(0.0) }) }
    }

    /**
     * Returns a Command that drives the swerve drive to a specific distance at a given speed.
     *
     * @param distanceInMeters       the distance to drive in meters
     * @param speedInMetersPerSecond the speed at which to drive in meters per second
     * @return a Command that drives the swerve drive to a specific distance at a given speed
     */
    fun driveToDistanceCommand(distanceInMeters: Double, speedInMetersPerSecond: Double): Command {
        return run { drive(ChassisSpeeds(speedInMetersPerSecond, 0.0, 0.0)) }
            .until {
                (swerveDrive.pose.translation.getDistance(Translation2d(0.0, 0.0))
                        > distanceInMeters)
            }
    }

    /**
     * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
     *
     * @param kS the static gain of the feedforward
     * @param kV the velocity gain of the feedforward
     * @param kA the acceleration gain of the feedforward
     */
    fun replaceSwerveModuleFeedforward(kS: Double, kV: Double, kA: Double) {
        swerveDrive.replaceSwerveModuleFeedforward(SimpleMotorFeedforward(kS, kV, kA))
    }

    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     *
     * @param translationX     Translation in the X direction. Cubed for smoother controls.
     * @param translationY     Translation in the Y direction. Cubed for smoother controls.
     * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
     * @return Drive command.
     */
    fun driveCommand(
        translationX: DoubleSupplier,
        translationY: DoubleSupplier,
        angularRotationX: DoubleSupplier
    ): Command {
        return run {
            // Make the robot move
            swerveDrive.drive(
                SwerveMath.scaleTranslation(
                    Translation2d(
                        translationX.asDouble
                                * swerveDrive.maximumChassisVelocity,
                        translationY.asDouble
                                * swerveDrive.maximumChassisVelocity
                    ),
                    0.8
                ),
                angularRotationX.asDouble.pow(3.0) * swerveDrive.maximumChassisAngularVelocity,
                true,
                false
            )
        }
    }

    /**
     * Command to drive the robot using translative values and heading as a setpoint.
     *
     * @param translationX Translation in the X direction. Cubed for smoother controls.
     * @param translationY Translation in the Y direction. Cubed for smoother controls.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    fun driveCommand(
        translationX: DoubleSupplier,
        translationY: DoubleSupplier,
        headingX: DoubleSupplier,
        headingY: DoubleSupplier
    ): Command {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for
        // this kind of control.
        return run {
            val scaledInputs = SwerveMath.scaleTranslation(
                Translation2d(translationX.asDouble, translationY.asDouble), 0.8
            )
            // Make the robot move
            driveFieldOriented(
                swerveDrive.swerveController.getTargetSpeeds(
                    scaledInputs.x,
                    scaledInputs.y,
                    headingX.asDouble,
                    headingY.asDouble,
                    swerveDrive.odometryHeading.radians,
                    swerveDrive.maximumChassisVelocity
                )
            )
        }
    }

    /**
     * The primary method for controlling the drivebase.  Takes a [Translation2d] and a rotation rate, and
     * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
     * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
     *
     * @param translation   [Translation2d] that is the commanded linear velocity of the robot, in meters per
     * second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
     * torwards port (left).  In field-relative mode, positive x is away from the alliance wall
     * (field North) and positive y is torwards the left wall when looking through the driver station
     * glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
     * relativity.
     * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
     */
    fun drive(translation: Translation2d?, rotation: Double, fieldRelative: Boolean) {
        swerveDrive.drive(
            translation,
            rotation,
            fieldRelative,
            false
        ) // Open loop is disabled since it shouldn't be used most of the time.
    }

    fun driveFieldOriented(velocity: ChassisSpeeds?) {
        swerveDrive.driveFieldOriented(velocity)
    }

    fun driveFieldOriented(velocity: Supplier<ChassisSpeeds?>): Command {
        return run {
            swerveDrive.driveFieldOriented(velocity.get())
        }
    }

    fun drive(velocity: ChassisSpeeds?) {
        swerveDrive.drive(velocity)
    }

    val kinematics: SwerveDriveKinematics
        get() = swerveDrive.kinematics

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    fun resetOdometry(initialHolonomicPose: Pose2d?) {
        println(initialHolonomicPose)
        Thread.dumpStack()
        swerveDrive.resetOdometry(initialHolonomicPose)
    }

    val pose: Pose2d
        get() = swerveDrive.pose

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds?) {
        swerveDrive.setChassisSpeeds(chassisSpeeds)
    }

    fun postTrajectory(trajectory: Trajectory?) {
        swerveDrive.postTrajectory(trajectory)
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    fun zeroGyro() {
        swerveDrive.zeroGyro()
    }

    private val isRedAlliance: Boolean
        /**
         * Checks if the alliance is red, defaults to false if alliance isn't available.
         *
         * @return true if the red alliance, false if blue. Defaults to false if none is available.
         */
        get() {
            val alliance = DriverStation.getAlliance()
            return if (alliance.isPresent) alliance.get() == Alliance.Red else false
        }

    /**
     * This will zero (calibrate) the robot to assume the current position is facing forward
     *
     *
     * If red alliance rotate the robot 180 after the drviebase zero command
     */
    fun zeroGyroWithAlliance() {
        if (isRedAlliance) {
            zeroGyro()
            // Set the pose 180 degrees
            resetOdometry(Pose2d(pose.translation, Rotation2d.fromDegrees(180.0)))
        } else {
            zeroGyro()
        }
    }

    fun setMotorBrake(brake: Boolean) {
        swerveDrive.setMotorIdleMode(brake)
    }

    val heading: Rotation2d
        /**
         * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
         * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
         *
         * @return The yaw angle
         */
        get() = pose.rotation

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return [ChassisSpeeds] which can be sent to the Swerve Drive.
     */
    fun getTargetSpeeds(
        xInput: Double, yInput: Double, headingX: Double, headingY: Double
    ): ChassisSpeeds {
        val scaledInputs = SwerveMath.cubeTranslation(Translation2d(xInput, yInput))
        return swerveDrive.swerveController.getTargetSpeeds(
            scaledInputs.x,
            scaledInputs.y,
            headingX,
            headingY,
            heading.radians,
            Constants.MAX_SPEED
        )
    }

    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
     * 90deg.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a [Rotation2d].
     * @return [ChassisSpeeds] which can be sent to the Swerve Drive.
     */
    fun getTargetSpeeds(xInput: Double, yInput: Double, angle: Rotation2d): ChassisSpeeds {
        val scaledInputs = SwerveMath.cubeTranslation(Translation2d(xInput, yInput))

        return swerveDrive.swerveController.getTargetSpeeds(
            scaledInputs.x,
            scaledInputs.y,
            angle.radians,
            heading.radians,
            Constants.MAX_SPEED
        )
    }

    val fieldVelocity: ChassisSpeeds
        get() = swerveDrive.fieldVelocity

    val robotVelocity: ChassisSpeeds
        get() = swerveDrive.robotVelocity

    val swerveController: SwerveController
        get() = swerveDrive.swerveController

    val swerveDriveConfiguration: SwerveDriveConfiguration
        get() = swerveDrive.swerveDriveConfiguration

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    fun lock() {
        swerveDrive.lockPose()
    }

    val pitch: Rotation2d
        get() = swerveDrive.pitch
}
