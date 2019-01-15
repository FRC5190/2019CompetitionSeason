/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import org.ghrobotics.frc2019.subsystems.drive.VisionDriveCommand
import org.ghrobotics.lib.wrappers.hid.button
import org.ghrobotics.lib.wrappers.hid.kA
import org.ghrobotics.lib.wrappers.hid.xboxController

object Controls {
    val mainXbox = xboxController(0) {
        button(kA).change(VisionDriveCommand())
    }

    fun update() {
        mainXbox.update()
    }
}