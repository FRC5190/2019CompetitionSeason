package org.ghrobotics.frc2019.vision

import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.second
import java.util.concurrent.CopyOnWriteArraySet


object TargetTracker {

    private val _targets = CopyOnWriteArraySet<TrackedTarget>()
    val targets: Set<TrackedTarget> = _targets

    var bestTargetFront: TrackedTarget? = null
        private set

    var bestTargetBack: TrackedTarget? = null
        private set

    fun addSamples(creationTime: Time, samples: Iterable<Pose2d>) =
        addSamples(samples.map { TrackedTargetSample(creationTime, it) })

    fun addSamples(samples: Iterable<TrackedTargetSample>) = samples.forEach(::addSample)

    fun addSample(sample: TrackedTargetSample) {
        if (sample.creationTime.second >= Timer.getFPGATimestamp()) return // Cannot predict the future

        val closestTarget = _targets.minBy {
            it.averagedPose2d.translation.distance(sample.targetPose.translation)
        }
        if (closestTarget == null
            || closestTarget.averagedPose2d.translation.distance(sample.targetPose.translation) > Constants.kTargetTrackingDistanceErrorTolerance.value
        ) {
            // Create new target if no targets are within tolerance
            _targets += TrackedTarget(sample)
        } else {
            // Add sample to target within tolerance
            closestTarget.addSample(sample)
        }
    }

    fun update() {
        val currentTime = Timer.getFPGATimestamp().second

        // Update and remove old targets
        _targets.removeIf {
            it.update(currentTime)
            !it.isAlive
        }

        // Update the new best front and back targets
        var newFrontTarget: TrackedTarget? = null
        var tempFrontDistance = 0.0
        var newBackTarget: TrackedTarget? = null
        var tempBackDistance = 0.0

        val currentRobotPose = DriveSubsystem.localization()

        for (target in _targets) {
            if (!target.isReal) continue

            val targetRelativeToRobot = target.averagedPose2d inFrameOfReferenceOf currentRobotPose
            val newDistance = targetRelativeToRobot.translation.norm.value
            if (targetRelativeToRobot.translation.x.value > 0.0) {
                // Front Target
                if (newFrontTarget == null || newDistance < tempFrontDistance) {
                    newFrontTarget = target
                    tempFrontDistance = newDistance
                }
            } else {
                // Back Target
                if (newBackTarget == null || newDistance < tempBackDistance) {
                    newBackTarget = target
                    tempBackDistance = newDistance
                }
            }
        }

        bestTargetFront = newFrontTarget
        bestTargetBack = newBackTarget

        // Publish to dashboard
        LiveDashboard.visionTargets = _targets.asSequence()
            .filter { it.isReal }
            .map { it.averagedPose2d }
            .toList()
    }

    class TrackedTarget(
        initialTargetSample: TrackedTargetSample
    ) {

        private val samples = CopyOnWriteArraySet<TrackedTargetSample>()

        /**
         * The averaged pose2d for x time
         */
        var averagedPose2d = initialTargetSample.targetPose
            private set

        /**
         * When the target was first encountered
         */
        val dateOfBirth = initialTargetSample.creationTime

        /**
         * Targets will be "alive" when it has at least one data point for x time
         */
        var isAlive = true
            private set

        /**
         * Target will become a "real" target once it has received data points for x time
         */
        var isReal = false
            private set

        var stability = 0.0
            private set

        init {
            addSample(initialTargetSample)
        }

        fun addSample(newSamples: TrackedTargetSample) {
            samples.add(newSamples)
        }

        fun update(currentTime: Time) {
            // Remove expired samples
            samples.removeIf { currentTime - it.creationTime >= Constants.kTargetTrackingMaxLifetime }
            // Update State
            isAlive = samples.isNotEmpty()
            isReal = if (isAlive) {
                @Suppress("UnsafeCallOnNullableType")
                val lastSampleTime = samples.maxBy { it.creationTime.value }!!.creationTime
                lastSampleTime - dateOfBirth >= Constants.kTargetTrackingMinLifetime
            } else {
                false
            }
            stability = (samples.size / (Constants.kVisionCameraFPS * Constants.kTargetTrackingMaxLifetime.value))
                .coerceAtMost(1.0)
            // Update Averaged Pose
            var accumulatedX = 0.0
            var accumulatedY = 0.0
            var accumulatedAngle = 0.0
            for (sample in samples) {
                accumulatedX += sample.targetPose.translation.x.value
                accumulatedY += sample.targetPose.translation.y.value
                accumulatedAngle += sample.targetPose.rotation.value
            }
            averagedPose2d = Pose2d(
                Length(accumulatedX / samples.size),
                Length(accumulatedY / samples.size),
                Rotation2d(accumulatedAngle / samples.size)
            )
        }

    }

    data class TrackedTargetSample(
        val creationTime: Time,
        val targetPose: Pose2d
    )

}