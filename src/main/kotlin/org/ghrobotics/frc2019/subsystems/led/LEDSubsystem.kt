package org.ghrobotics.frc2019.subsystems.led

import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.millisecond
import java.awt.Color

object LEDSubsystem : FalconSubsystem(), EmergencyHandleable {

    init {
        defaultCommand = SolidLEDCommand(Color.BLACK)
    }

    fun set(color: Color) {
        // TODO Canifier or Arduino
    }

    override fun activateEmergency() {
        BlinkingLEDCommand(Color.RED, 500.millisecond).start()
    }

    override fun recoverFromEmergency() {
        defaultCommand.start()
    }
}