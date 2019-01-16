/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import org.ghrobotics.frc2019.subsystems.drive.VisionDriveCommand
import org.ghrobotics.lib.wrappers.hid.*

object Controls {

    val mainXbox = xboxController(0) {
        button(kA).change(VisionDriveCommand())

        button(kBack).changeOn {
            Robot.emergencyReadySystems.forEach { system -> system.activateEmergency() }
        }
        button(kStart).changeOn {
            Robot.emergencyReadySystems.forEach { system -> system.recoverFromEmergency() }
        }
    }

    fun update() {
        mainXbox.update()
    }
}