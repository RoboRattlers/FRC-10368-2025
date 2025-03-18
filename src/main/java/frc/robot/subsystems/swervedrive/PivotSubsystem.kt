package frc.robot.subsystems.swervedrive

import com.thethriftybot.ThriftyNova
import com.thethriftybot.ThriftyNova.PIDSlot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants

class PivotSubsystem : SubsystemBase() {

    val motor = ThriftyNova(Constants.CanId.PIVOT)
        .setInversion(false)
        .setBrakeMode(true)
        .setMaxCurrent(ThriftyNova.CurrentType.STATOR, 10.0)
        .setMaxOutput(1.0)
        .setSoftLimits(0.0, 900.0)
        .useEncoderType(ThriftyNova.EncoderType.INTERNAL)

    init {
        motor.pid0.setP(0.001)
            .setI(0.0)
            .setD(0.0)
        motor.usePIDSlot(PIDSlot.SLOT0)
    }

    override fun periodic() {
        SmartDashboard.putNumber("Pivot Encoder", motor.positionInternal)
        SmartDashboard.putNumber("Pivot Current", motor.statorCurrent)
    }

    fun toPosCommand(pos: Double): Command {
        return runOnce {
            motor.position = pos
        }
    }

}