package org.ghrobotics.frc2019.vision

import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.second

object TargetTracker {

    private val _trackedTargets = mutableListOf<TrackedTarget>()
    val trackedTargets: List<TrackedTarget> = _trackedTargets

    var bestTargetFront: TrackedTarget? = null
        private set

    var bestTargetBack: TrackedTarget? = null
        private set

    @Suppress("ComplexMethod")
    fun addSamples(time: Time, targets: List<Pose2d>) {
        val current = (Timer.getFPGATimestamp() + 1.0).second
        if (time >= current) return

        for (targetPose in targets) {
            val closestTarget = _trackedTargets.minBy {
                it.averagePose.translation.distance(targetPose.translation)
            }
            if (closestTarget == null
                || closestTarget.averagePose.translation.distance(targetPose.translation) > Constants.kMaxTargetTrackingDistance.value
            ) {
                // Create a new target if none is close enough
                _trackedTargets += TrackedTarget(time, targetPose)
            } else {
                // Update target location
                closestTarget.addSample(time, targetPose)
            }
        }
        _trackedTargets.forEach { it.update() }
        _trackedTargets.removeIf { !it.isAlive; }

        val robotPose = DriveSubsystem.localization()

        var newFrontTarget: TrackedTarget? = null
        var tempFrontDistance = 0.0
        var newBackTarget: TrackedTarget? = null
        var tempBackDistance = 0.0

        for (target in _trackedTargets) {
            val targetRelative = target.averagePose inFrameOfReferenceOf robotPose
            val newDistance = targetRelative.translation.norm.value
            if (targetRelative.translation.x.value > 0.0) {
                // Front
                if (newFrontTarget == null || newDistance < tempFrontDistance) {
                    newFrontTarget = target
                    tempFrontDistance = newDistance
                }
            } else {
                // Back
                if (newBackTarget == null || newDistance < tempBackDistance) {
                    newBackTarget = target
                    tempBackDistance = newDistance
                }
            }
        }

        bestTargetFront = newFrontTarget
        bestTargetBack = newBackTarget

        LiveDashboard.visionTargets = _trackedTargets.map {
            it.averagePose
        }

        // println(_trackedTargets.joinToString { it.averagePose.toString() })
    }

}

class TrackedTarget(creationTime: Time, initialPose: Pose2d) {

    private val targetSamples = mutableListOf(creationTime to initialPose)

    var averagePose = initialPose
        private set

    var lastUpdated = creationTime
        private set

    val isAlive get() = targetSamples.isNotEmpty()

    fun update() {
        val currentTime = Timer.getFPGATimestamp().second

        targetSamples.removeIf {
            currentTime - it.first > Constants.kMaxTargetTrackingLifetime
        }

        if (isAlive) {
            updateAverage()
        }
    }

    fun addSample(time: Time, newPose: Pose2d) {
        targetSamples += time to newPose
        lastUpdated = time
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