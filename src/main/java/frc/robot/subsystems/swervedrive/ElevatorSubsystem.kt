package frc.robot.subsystems.swervedrive

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.EncoderConfig
import com.revrobotics.spark.config.SoftLimitConfig
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.CanId
import kotlin.math.sign

class ElevatorSubsystem : SubsystemBase() {

    val ELEVATOR_MAX_ENCODER_VALUE = 1750.0
    val kP = 20.0
    val kI = 0.0
    val kD = 0.0
    val kFF = 0.2//0.8
    val kS = (0.8 - 0.4)/2.0

    var currentPos = 0.0
        private set

    val pidController = PIDController(kP, kI, kD)
    var setpoint = 0.0
    val slewRateLimiter = SlewRateLimiter(1.3)
    var lowerBound = 0.0
    var upperBound = 1.0

    val motorTop = SparkMax(CanId.ELEVATOR_TOP, SparkLowLevel.MotorType.kBrushless)
    val motorConfig = SparkMaxConfig()
        .smartCurrentLimit(40)
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .apply(
            SoftLimitConfig()
                .forwardSoftLimit(ELEVATOR_MAX_ENCODER_VALUE)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(0.0)
                .reverseSoftLimitEnabled(true)
        )
        .apply(
            EncoderConfig()
                .positionConversionFactor(42.0)
        )
    val motorBottom = SparkMax(CanId.ELEVATOR_BOTTOM, SparkLowLevel.MotorType.kBrushless)

    init {
        motorTop.configure(
            motorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        motorBottom.configure(
            SparkMaxConfig()
                .apply(motorConfig)
                .follow(motorTop),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        SmartDashboard.putNumber("Elevator FF", 0.0)
    }

    override fun periodic() {
        val encoderPos = motorTop.encoder.position
        val heightFraction = encoderPos/ELEVATOR_MAX_ENCODER_VALUE
        currentPos = heightFraction
        SmartDashboard.putNumber("Elevator Encoder", encoderPos)
        SmartDashboard.putNumber("Elevator Fraction", heightFraction)
        SmartDashboard.putNumber("Elevator Setpoint", pidController.setpoint)
        SmartDashboard.putNumber("Elevator Current", motorTop.outputCurrent)

        val setpointAfterClamp = clamp(setpoint, lowerBound, upperBound)
        /*if (setpoint != setpointAfterClamp) {
            slewRateLimiter.reset(setpointAfterClamp)
        }*/
        pidController.setpoint = slewRateLimiter.calculate(setpointAfterClamp)
        val voltage = pidController.calculate(heightFraction) + kFF + kS * sign(pidController.setpoint - encoderPos)
        motorTop.setVoltage(clamp(voltage, -12.0, 12.0) )
        //motorTop.setVoltage(0.0)
    }

    fun toPosCommand(pos: Double): Command {
        return runOnce {
            setpoint = pos
        }
    }

}