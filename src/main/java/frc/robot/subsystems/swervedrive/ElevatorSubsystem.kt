package frc.robot.subsystems.swervedrive

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.EncoderConfig
import com.revrobotics.spark.config.SoftLimitConfig
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import com.thethriftybot.ThriftyNova
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.CanId
import kotlin.math.sign

private const val MAX_OUTPUT = 0.8
private const val STATOR_CURRENT_LIMIT = 40.0

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

    var motorTop = ThriftyNova(CanId.ELEVATOR_TOP)
        .setInversion(false)
        .setBrakeMode(true)
        .setMaxCurrent(ThriftyNova.CurrentType.STATOR, STATOR_CURRENT_LIMIT)
        .setMaxOutput(MAX_OUTPUT)
        .setSoftLimits(0.0, ELEVATOR_MAX_ENCODER_VALUE)
        .useEncoderType(ThriftyNova.EncoderType.INTERNAL)
    var motorBottom = ThriftyNova(CanId.ELEVATOR_BOTTOM)
        .setInversion(false)
        .setBrakeMode(true)
        .setMaxCurrent(ThriftyNova.CurrentType.STATOR, STATOR_CURRENT_LIMIT)
        .setMaxOutput(MAX_OUTPUT)
        .setSoftLimits(0.0, ELEVATOR_MAX_ENCODER_VALUE)
        .useEncoderType(ThriftyNova.EncoderType.INTERNAL)

    init {
        motorTop.canFreq.setSensor(0.01)
        motorBottom.canFreq.setSensor(0.01)
    }

    fun setMotorBrake(brake: Boolean) {
        motorTop.setBrakeMode(brake)
        motorBottom.setBrakeMode(brake)
    }

    override fun periodic() {
        val encoderPos = motorBottom.positionInternal
        val heightFraction = encoderPos/ELEVATOR_MAX_ENCODER_VALUE
        currentPos = heightFraction
        SmartDashboard.putNumber("Elevator Encoder", encoderPos)
        SmartDashboard.putNumber("Elevator Fraction", heightFraction)
        SmartDashboard.putNumber("Elevator Setpoint", pidController.setpoint)
        SmartDashboard.putNumber("Elevator Current", motorBottom.statorCurrent)

        val setpointAfterClamp = clamp(setpoint, lowerBound, upperBound)
        pidController.setpoint = slewRateLimiter.calculate(setpointAfterClamp)
        val voltage = pidController.calculate(heightFraction) + kFF + kS * sign(pidController.setpoint - encoderPos)
        motorBottom.setVoltage(clamp(voltage, -12.0, 12.0) )
        motorTop.setVoltage(clamp(voltage, -12.0, 12.0) )


    }

    fun toPosCommand(pos: Double): Command {
        return run {
            setpoint = pos
        } .until( { currentPos - setpoint < 0.03 } )
    }

}