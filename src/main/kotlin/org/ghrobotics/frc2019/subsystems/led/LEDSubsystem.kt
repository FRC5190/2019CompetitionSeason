package org.ghrobotics.frc2019.subsystems.led

import kotlinx.coroutines.CoroutineStart
import kotlinx.coroutines.GlobalScope
import org.ghrobotics.frc2019.Controls
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.utils.launchFrequency
import org.ghrobotics.lib.utils.monitor
import java.awt.Color

object LEDSubsystem : FalconSubsystem(), EmergencyHandleable {

    private val fault = BlinkingLEDCommand(Color.RED, 500.millisecond)
    private val climb = BlinkingLEDCommand(Color.ORANGE, 500.millisecond)

    private val climbingMonitor = { Controls.isClimbing && !Robot.emergencyActive }.monitor

    init {
        defaultCommand = SolidLEDCommand(Color.BLACK)
    }

    override fun lateInit() {
        GlobalScope.launchFrequency(start = CoroutineStart.LAZY) {
            climbingMonitor.onChange {
                if (it) climb.start()
                else climb.stop()
            }
        }
    }

    fun set(color: Color) {
        // TODO Canifier or Arduino
    }

    override fun activateEmergency() = fault.start()
    override fun recoverFromEmergency() = fault.stop()
}