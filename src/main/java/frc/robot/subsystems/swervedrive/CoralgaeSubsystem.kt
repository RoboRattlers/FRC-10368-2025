package frc.robot.subsystems.swervedrive

import com.thethriftybot.ThriftyNova
import edu.wpi.first.wpilibj2.command.*
import frc.robot.Constants

class CoralgaeSubsystem : SubsystemBase() {

    val motorInner = ThriftyNova(Constants.CanId.CORALGAE_INNER)
        .setInversion(true)
        .setBrakeMode(false)
        .setMaxCurrent(ThriftyNova.CurrentType.STATOR, 10.0)
        .useEncoderType(ThriftyNova.EncoderType.INTERNAL)

    val motorOuterTop = ThriftyNova(Constants.CanId.CORALGAE_OUTER_TOP)
        .setInversion(false)
        .setBrakeMode(false)
        .setMaxCurrent(ThriftyNova.CurrentType.STATOR, 40.0)
        .useEncoderType(ThriftyNova.EncoderType.INTERNAL)

    val motorOuterBottom = ThriftyNova(Constants.CanId.CORALGAE_OUTER_BOTTOM)
        .setInversion(true)
        .setBrakeMode(false)
        .setMaxCurrent(ThriftyNova.CurrentType.STATOR, 40.0)
        .useEncoderType(ThriftyNova.EncoderType.INTERNAL)

    var algaeStall = false

    init {
        this.defaultCommand = runOnce {
            motorInner.setVoltage(if (algaeStall) 0.5 else 0.0)
            motorOuterTop.setVoltage(0.0)
            motorOuterBottom.setVoltage(0.0)
        }
    }

    fun algaeIntakeCommand(fraction: Double): Command {
        return run {
            motorInner.setVoltage(6.0 * fraction)
            motorOuterTop.setVoltage(12.0 * fraction)
            motorOuterBottom.setVoltage(12.0 * fraction)
            algaeStall = fraction > 0.0
        }
    }

    fun algaeShootCommand(fraction: Double): Command {
        return SequentialCommandGroup(
            run {
                motorOuterTop.setVoltage(-12.0 * fraction)
                motorOuterBottom.setVoltage(-12.0 * fraction)
                motorInner.setVoltage(0.5)
            } .withTimeout(1.0),
            run {
                motorInner.setVoltage(-6.0)
                motorOuterTop.setVoltage(-12.0 * fraction)
                motorOuterBottom.setVoltage(-12.0 * fraction)
                algaeStall = false
            } .withTimeout(1.0) )
    }

    fun coralIntakeCommand(fraction: Double): Command {
        return run {
            motorOuterTop.setVoltage(-12.0 * fraction)
            motorInner.setVoltage(12.0 * fraction)
            algaeStall = false
        }
    }

}