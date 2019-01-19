package org.ghrobotics.frc2019.subsystems.led

import kotlinx.coroutines.GlobalScope
import org.ghrobotics.frc2019.Controls
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.frc2019.subsystems.drive.VisionDriveCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.utils.launchFrequency
import org.ghrobotics.lib.utils.monitor
import java.awt.Color

object LEDSubsystem : FalconSubsystem(), EmergencyHandleable {

    private val fault = BlinkingLEDCommand(Color.RED, 500.millisecond)
    private val climb = BlinkingLEDCommand(Color.ORANGE, 500.millisecond)
    private val vision = SolidLEDCommand(Color.GREEN)

    private val climbingMonitor = { Controls.isClimbing && !Robot.emergencyActive }.monitor
    private val visionMonitor = { VisionDriveCommand.isActive && !Robot.emergencyActive }.monitor

    init {
        defaultCommand = SolidLEDCommand(Color.BLACK)
    }

    override fun lateInit() {
        GlobalScope.launchFrequency {
            climbingMonitor.onChange {
                if (it) climb.start()
                else climb.stop()
            }
            visionMonitor.onChange {
                if (it) vision.start()
                else vision.stop()
            }
        }
    }

    fun set(color: Color) {
        // TODO
    }

    override fun activateEmergency() = fault.start()
    override fun recoverFromEmergency() = fault.stop()
}