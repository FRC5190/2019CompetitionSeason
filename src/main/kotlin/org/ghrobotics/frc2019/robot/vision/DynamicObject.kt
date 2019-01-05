package org.ghrobotics.frc2019.robot.vision

import org.ghrobotics.lib.localization.Localization
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.Time

/**
 * Localization helper for objects on the field
 *
 * @param localization the localization tracker that this object will be based on
 * @param defaultLocation the default location of the object on the field (This is not relative to robot)
 */
class DynamicObject(
    private val localization: Localization,
    defaultLocation: Translation2d
) {

    var objectLocationOnField = defaultLocation
        private set

    val objectLocationRelativeToRobot get() = Pose2d(objectLocationOnField) inFrameOfReferenceOf localization()

    fun setSample(sampleTime: Time, robotToObjectPose: Pose2d) {
        val fieldToRobotPose = localization[sampleTime]
        objectLocationOnField = fieldToRobotPose.transformBy(robotToObjectPose).translation
    }
}