/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.robot

import org.ghrobotics.frc2019.robot.subsytems.drive.DriveSubsystem
import org.ghrobotics.frc2019.robot.vision.VisionProcessing
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.wrappers.hid.button
import org.ghrobotics.lib.wrappers.hid.kA
import org.ghrobotics.lib.wrappers.hid.xboxController

object Controls {
    val mainXbox = xboxController(0) {
        button(kA).change(
            DriveSubsystem.driveToLocation {
                val pose = VisionProcessing.currentlyTrackedObject.objectLocationOnField + Constants.kForwardIntakeToCenter
                val snappedAngle = (Math.round(DriveSubsystem.localization().rotation.degree / 90.0) * 90.0).degree
                Pose2d(pose.translation, snappedAngle)
            }
        )
    }
}