// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
class Robot : TimedRobot() {

    private var m_autonomousCommand: Command? = null
    private var m_robotContainer: RobotContainer? = null
    private var disabledTimer: Timer? = null

    init {
        instance = this
    }
    companion object {
        lateinit var instance: Robot
            private set
    }

    override fun robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = RobotContainer()

        // Create a timer to disable motor brake a few seconds after disable.  This will let the
        // robot stop
        // immediately when disabled, but then also let it be pushed more
        disabledTimer = Timer()

        if (isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true)
        }
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
     * during disabled, autonomous, teleoperated and test.
     *
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit() {
        m_robotContainer!!.setMotorBrake(true)
        disabledTimer!!.reset()
        disabledTimer!!.start()
    }

    override fun disabledPeriodic() {
        if (disabledTimer!!.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
            m_robotContainer!!.setMotorBrake(false)
            disabledTimer!!.stop()
            disabledTimer!!.reset()
        }
    }

    /**
     * This autonomous runs the autonomous command selected by your [RobotContainer] class.
     */
    override fun autonomousInit() {
        m_robotContainer!!.setMotorBrake(true)
        m_autonomousCommand = m_robotContainer!!.autonomousCommand
        m_autonomousCommand!!.schedule()
    }

    override fun autonomousPeriodic() {}

    override fun teleopInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun teleopPeriodic() {}

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }
    override fun testPeriodic() {}

    override fun simulationInit() {}
    override fun simulationPeriodic() {}

}
