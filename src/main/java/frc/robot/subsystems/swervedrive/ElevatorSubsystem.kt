package frc.robot.subsystems.swervedrive

import com.thethriftybot.ThriftyNova
import com.thethriftybot.ThriftyNova.PIDSlot
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.CanId
import kotlin.math.sign

class ElevatorSubsystem : SubsystemBase() {

    val ELEVATOR_TOP_ENCODER_VALUE = 1562.0
    val kP = 0.02
    val kI = 0.0
    val kD = 0.0
    val kFF = 1.35/2.0
    val kS = 0.125
    val pidController = PIDController(kP, kI, kD)

    val motorTop = ThriftyNova(CanId.ELEVATOR_TOP)
        .setInversion(false)
        .setBrakeMode(true)
        .setMaxCurrent(ThriftyNova.CurrentType.STATOR, 40.0)
        .setMaxOutput(0.8)
        .setSoftLimits(0.0, ELEVATOR_TOP_ENCODER_VALUE)
        .useEncoderType(ThriftyNova.EncoderType.INTERNAL)
    val motorBottom = ThriftyNova(CanId.ELEVATOR_BOTTOM)
        .setInversion(false)
        .setBrakeMode(true)
        .setMaxCurrent(ThriftyNova.CurrentType.STATOR, 40.0)
        .setMaxOutput(0.8)
        .setSoftLimits(0.0, ELEVATOR_TOP_ENCODER_VALUE)

    init {
        //motorBottom.follow(CanId.ELEVATOR_TOP)
    }

    override fun periodic() {
        val encoderPos = motorBottom.positionInternal
        SmartDashboard.putNumber("Elevator Encoder", encoderPos)
        SmartDashboard.putNumber("Elevator Setpoint", pidController.setpoint)
        SmartDashboard.putNumber("Elevator Current", motorBottom.statorCurrent)
        val voltage = pidController.calculate(encoderPos) + kFF + kS * sign(pidController.setpoint - encoderPos)
        motorTop.setVoltage(voltage)
        motorBottom.setVoltage(voltage)
    }

    fun toPosCommand(pos: Double): Command {
        return runOnce {
            pidController.setpoint = pos * ELEVATOR_TOP_ENCODER_VALUE
        }
    }

}