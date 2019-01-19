/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import org.ghrobotics.frc2019.subsystems.drive.VisionDriveCommand
import org.ghrobotics.lib.wrappers.hid.*

object Controls {

    var isClimbing = false
        private set

    val mainXbox = xboxController(0) {
        state({ !isClimbing }) {
            // Align with Vision Target
            button(kB).change(VisionDriveCommand())

            // Emergency Mode
            button(kBack).changeOn {
                Robot.emergencyReadySystems.forEach { system -> system.activateEmergency() }
                Robot.emergencyActive = true
            }
            button(kStart).changeOn {
                Robot.emergencyReadySystems.forEach { system -> system.recoverFromEmergency() }
                Robot.emergencyActive = false
            }
            button(kBumperRight).change(VisionDriveCommand())
        }
    }

    fun update() {
        mainXbox.update()
    }
}