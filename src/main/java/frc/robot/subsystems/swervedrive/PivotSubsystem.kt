package frc.robot.subsystems.swervedrive

import com.thethriftybot.ThriftyNova
import com.thethriftybot.ThriftyNova.PIDSlot
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants

class PivotSubsystem : SubsystemBase() {

    private val motor = ThriftyNova(Constants.CanId.PIVOT)
        .setInversion(false)
        .setBrakeMode(true)
        .setMaxCurrent(ThriftyNova.CurrentType.STATOR, 35.0)
        .setMaxOutput(1.0)
        .setSoftLimits(0.0, 1400.0)
        .useEncoderType(ThriftyNova.EncoderType.INTERNAL)

    val encoderMutiplier = (56.0/14.0) / (26.0/10.0)

    val kP = 0.02
    val kI = 0.0
    val kD = 0.0
    val kFF = 0.0
    val kS = (0.8 - 0.4)/2.0
    val pidController = PIDController(kP, kI, kD)
    var setpoint = 0.0
        private set
    val slewRateLimiter = SlewRateLimiter(2000.0)

    var usePid = true
    var lowerBound = 400.0
    var upperBound = 900.0

    init {
        motor.pid0.setP(0.001)
            .setI(0.0)
            .setD(0.0)
        //motor.usePIDSlot(PIDSlot.SLOT0)
    }

    override fun periodic() {
        SmartDashboard.putNumber("Pivot Encoder", motor.positionInternal)
        SmartDashboard.putNumber("Pivot Current", motor.statorCurrent)

        val setpointBeforeClamp = setpoint
        setpoint = clamp(setpoint, lowerBound, upperBound)
        if (setpointBeforeClamp != setpoint) {
            slewRateLimiter.reset(setpoint)
        }
        pidController.setpoint = slewRateLimiter.calculate(setpoint)
        val voltage = pidController.calculate(motor.positionInternal * encoderMutiplier) +
                kFF
        //+ kS * sign(pidController.setpoint - encoderPos)
        motor.setVoltage(clamp(voltage, -12.0, 12.0) )
        for (error in motor.errors) {
            println(error)
        }
        motor.clearErrors()

    }

    fun toPosCommand(pos: Double): Command {
        return runOnce {
            setpoint = pos
        }
    }

    fun autoHomeCommand(): Command {
        val timer = Timer()
        return SequentialCommandGroup(
            runOnce {
                timer.reset()
                timer.start()
                usePid = false
                    },
            run {
                motor.setVoltage(-3.0)
            }.until { timer.hasElapsed(1.5) },
            runOnce {
                motor.setVoltage(0.0)
                motor.setEncoderPosition(-100.0)
                usePid = true
                toPosCommand(380.0).schedule()
            }
        )
    }

}