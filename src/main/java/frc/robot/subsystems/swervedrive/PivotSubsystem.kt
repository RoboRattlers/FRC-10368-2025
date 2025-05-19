package frc.robot.subsystems.swervedrive

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.EncoderConfig
import com.revrobotics.spark.config.SoftLimitConfig
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import com.thethriftybot.ThriftyNova
import com.thethriftybot.ThriftyNova.CANFreqConfig
import com.thethriftybot.ThriftyNova.PIDSlot
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.CanId

class PivotSubsystem : SubsystemBase() {

    private var motor = ThriftyNova(CanId.PIVOT)
        .setInversion(false)
        .setBrakeMode(true)
        .setMaxCurrent(ThriftyNova.CurrentType.STATOR, 35.0)
        .setMaxOutput(1.0)
        .useEncoderType(ThriftyNova.EncoderType.INTERNAL)
    val bandaidFixDebounceTimer = Timer()
    val encoderMutiplier = (56.0/14.0) / (26.0/10.0)

    val kP = 0.02
    val kI = 0.0
    val kD = 0.0
    val kFF = 0.0
    val balancedEncoderOffset = 0.0

    var currentPos = 0.0
        private set

    val pidController = PIDController(kP, kI, kD)
    var setpoint = 0.0
    val slewRateLimiter = SlewRateLimiter(2000.0)
    var usePid = true
    var lowerBound = 400.0
    var upperBound = 900.0

    init {
        motor.canFreq.setSensor(0.01)
    }

    fun setMotorBrake(brake: Boolean) {
        motor.setBrakeMode(brake)
    }

    override fun periodic() {
        SmartDashboard.putNumber("Pivot Encoder", motor.positionInternal)

        val setpointAfterClamp = clamp(setpoint, lowerBound, upperBound)
        if (setpoint != setpointAfterClamp) {
            //slewRateLimiter.reset(setpointAfterClamp)
        }
        pidController.setpoint = slewRateLimiter.calculate(setpointAfterClamp)
        currentPos = motor.positionInternal * encoderMutiplier
        val voltage = pidController.calculate(currentPos) + kFF
        if (usePid) {
            motor.setVoltage(clamp(voltage, -12.0, 12.0))
        }


    }

    fun toPosCommand(pos: Double): Command {
        return run {
            setpoint = pos
        } .until { currentPos - setpoint < 150.0 }
    }

    fun autoHomeCommand(timeout: Double = 1.5): Command {
        val timer = Timer()
        return SequentialCommandGroup(
            runOnce {
                timer.reset()
                timer.start()
                usePid = false
                    },
            run {
                motor.setVoltage(-3.0)
            }.until { timer.hasElapsed(timeout) },
            runOnce {
                motor.setVoltage(0.0)
                motor.setEncoderPosition(-100.0)
                usePid = true
            }
        )
    }

}