package frc.robot.subsystems.swervedrive

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode
import com.ctre.phoenix.motorcontrol.can.VictorSPX
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.Constants
import frc.robot.util.MathUtils.mapRange

class CorintakePivotSubsystem : SubsystemBase() {

    private val motor = VictorSPX(Constants.CanId.CORINTAKE_PIVOT)

    val maxOutput = 0.4
    val encoder = DutyCycleEncoder(DigitalInput(9))
    var encoderLowerBound = 0.1
    var encoderUpperBound = 0.45

    val kP = 1.0
    val kI = 0.0
    val kD = 0.0
    val kFF = 0.1

    var currentPos = 0.0
        private set

    val pidController = PIDController(kP, kI, kD)
    var setpoint = 1.0
    val slewRateLimiter = SlewRateLimiter(2.0)
    var usePid = true

    init {
        /*SequentialCommandGroup(
            WaitCommand(5.0),
            runOnce { home() }
        ).schedule()*/
    }

    override fun periodic() {

        val setpointAfterClamp = clamp(setpoint, 0.0, 1.0)
        pidController.setpoint = slewRateLimiter.calculate(setpointAfterClamp)

        val percent = pidController.calculate(mapRange(encoder.get(), encoderLowerBound, encoderUpperBound, 0.0 ,1.0)) + kFF
        if (usePid) {
            motor.set(VictorSPXControlMode.PercentOutput, clamp(percent, -maxOutput, maxOutput))
        }
        SmartDashboard.putNumber("Ground Intake Setpoint", setpoint)
        SmartDashboard.putNumber("Ground Intake Percent Output", percent)
        SmartDashboard.putNumber("Actual Encoder Value", encoder.get())
        SmartDashboard.putNumber("Remapped Encoder Value", mapRange(encoder.get(), encoderLowerBound, encoderUpperBound, 0.0 ,1.0))
    }

    fun home() {
        val encoderRange = encoderUpperBound - encoderLowerBound
        encoderUpperBound = encoder.get()
        encoderLowerBound = encoderUpperBound - encoderRange
    }

    fun toPosCommand(pos: Double): Command {
        return runOnce {
            setpoint = pos
        }
    }

}