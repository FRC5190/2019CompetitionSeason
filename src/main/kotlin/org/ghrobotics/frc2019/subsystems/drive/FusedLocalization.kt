package org.ghrobotics.frc2019.subsystems.drive

import org.ghrobotics.frc2019.vision.TrackedTarget
import org.ghrobotics.lib.localization.Localization
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Twist2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.utils.Source

class FusedLocalization(
    headingSource: Source<Rotation2d>,
    private val leftEncoder: Source<Length>,
    private val rightEncoder: Source<Length>
) : Localization(headingSource) {

    private var prevLeftEncoder = 0.0
    private var prevRightEncoder = 0.0

    override fun resetInternal(newPosition: Pose2d) {
        super.resetInternal(newPosition)
        prevLeftEncoder = leftEncoder().value
        prevRightEncoder = rightEncoder().value
    }

    var fusedDelta = Pose2d()

    fun addVisionSample(target: TrackedTarget, staticLocation: Pose2d): Boolean {
        val timestamp = target.lastUpdated
        val historicalPose = this[timestamp]

        val toTarget = target.averagePose inFrameOfReferenceOf historicalPose
        val visionHistoricalPose = staticLocation - toTarget

        if (historicalPose.translation.distance(visionHistoricalPose.translation) < kMinDeadReckoningVsVisionNorm.value) {
            fusedDelta = Pose2d(
                visionHistoricalPose.translation.x - historicalPose.translation.x,
                visionHistoricalPose.translation.y - historicalPose.translation.y,
                0.degree
            )
            System.out.printf(
                "[FUSED LOCALIZER] Added Delta: X: %3.3f inches, Y: %3.3f inches",
                fusedDelta.translation.x.inch, fusedDelta.translation.y.inch
            )
            return true
        }
        return false
    }

    override fun update(deltaHeading: Rotation2d): Pose2d {
        val newLeftEncoder = leftEncoder().value
        val newRightEncoder = rightEncoder().value

        val deltaLeft = newLeftEncoder - prevLeftEncoder
        val deltaRight = newRightEncoder - prevRightEncoder

        this.prevLeftEncoder = newLeftEncoder
        this.prevRightEncoder = newRightEncoder

        val odometricDelta = forwardKinematics(deltaLeft, deltaRight, deltaHeading).asPose

        val delta = Pose2d(
            odometricDelta.translation.x + fusedDelta.translation.x,
            odometricDelta.translation.y + fusedDelta.translation.y,
            odometricDelta.rotation + fusedDelta.rotation
        )

        fusedDelta = Pose2d()
        return delta
    }

    private fun forwardKinematics(leftDelta: Double, rightDelta: Double, rotationDelta: Rotation2d): Twist2d {
        val dx = (leftDelta + rightDelta) / 2.0
        return Twist2d(dx.meter, 0.0.meter, rotationDelta)
    }

    companion object {
        val kMinDeadReckoningVsVisionNorm = 6.inch
    }
}