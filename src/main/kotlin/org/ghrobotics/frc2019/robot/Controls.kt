/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.robot

import org.ghrobotics.lib.wrappers.hid.xboxController

object Controls {
    val mainXbox = xboxController(0) {
        //        button(kA).change(
//            DriveSubsystem.driveToLocation {
//                val pose =
//                    VisionProcessing.currentlyTrackedObject.objectLocationOnField + Constants.kForwardIntakeToCenter
//                val snappedAngle = (Math.round(DriveSubsystem.localization().rotation.degree / 90.0) * 90.0).degree
//                Pose2d(pose.translation, snappedAngle)
//            }
//        )
    }

    fun update() {
        mainXbox.update()
    }
}