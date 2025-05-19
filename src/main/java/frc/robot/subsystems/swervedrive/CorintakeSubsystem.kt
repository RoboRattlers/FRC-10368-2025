package frc.robot.subsystems.swervedrive

import com.thethriftybot.ThriftyNova
import edu.wpi.first.wpilibj2.command.*
import frc.robot.Constants

class CorintakeSubsystem : SubsystemBase() {

    val motor = ThriftyNova(Constants.CanId.CORINTAKE_INTAKE)
        .setInversion(false)
        .setBrakeMode(true)
        .setMaxCurrent(ThriftyNova.CurrentType.STATOR, 40.0)
        .useEncoderType(ThriftyNova.EncoderType.INTERNAL)

    init {
        this.defaultCommand = runOnce {
            motor.setVoltage(0.0)
        }
    }

    fun runIntakeCommand(fraction: Double): Command {
        return run {
            motor.set(fraction)
        }
    }

}