package org.ghrobotics.frc2019.vision

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.millisecond
import kotlin.math.absoluteValue

object TargetTracker {

    private val _trackedTargets = mutableListOf<TrackedTarget>()
    val trackedTargets: List<TrackedTarget> = _trackedTargets

    val bestTarget: TrackedTarget?
        get() {
            val robotPose = DriveSubsystem.localization()

            return _trackedTargets.minBy {
                val translation = (it.averagePose inFrameOfReferenceOf robotPose).translation
                Math.atan2(
                    translation.y.value,
                    translation.x.value
                ).absoluteValue
            }
        }

    fun addSamples(time: Time, targets: List<Pose2d>) {
        for (targetPose in targets) {
            val closestTarget = _trackedTargets.minBy {
                it.averagePose.distance(targetPose)
            }
            if (closestTarget == null
                || closestTarget.averagePose.distance(targetPose) > Constants.kMaxTargetTrackingDistance.value
            ) {
                // Create a new target if none is close enough
                _trackedTargets += TrackedTarget(time, targetPose)
            } else {
                // Update target location
                closestTarget.update(time, targetPose)
            }
        }
       // _trackedTargets.removeIf { !it.isAlive }
    }

}

class TrackedTarget(creationTime: Time, initialPose: Pose2d) {

    private val targetSamples = mutableListOf(creationTime to initialPose)

    var averagePose = initialPose
        private set

    var lastUpdated = creationTime
        private set

    val isAlive get() = (System.currentTimeMillis().millisecond - lastUpdated) < Constants.kMaxTargetTrackingLifetime

    fun update(time: Time, newPose: Pose2d) {
        targetSamples += time to newPose
        lastUpdated = time

        val currentTime = System.currentTimeMillis().millisecond

        targetSamples.removeIf {
            (currentTime - it.first) > Constants.kMaxTargetTrackingLifetime
        }
    }

    private fun updateAverage() {
        var accumX = 0.0
        var accumY = 0.0
        var accumAngle = 0.0
        targetSamples.forEach { sample ->
            accumX += sample.second.translation.x.meter
            accumY += sample.second.translation.y.meter
            accumAngle += sample.second.rotation.degree
        }
        accumX /= targetSamples.size
        accumY /= targetSamples.size
        accumAngle /= targetSamples.size
        averagePose = Pose2d(
            accumX.meter,
            accumY.meter,
            accumAngle.degree
        )
    }

}