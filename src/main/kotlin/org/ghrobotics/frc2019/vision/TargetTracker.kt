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
import kotlin.concurrent.fixedRateTimer


object TargetTracker {

    private val targets = mutableSetOf<TrackedTarget>()

    var bestTargetFront: TrackedTarget? = null
        private set

    var bestTargetFrontLeft: TrackedTarget? = null
        private set

    var bestTargetFrontRight: TrackedTarget? = null
        private set

    var bestTargetBack: TrackedTarget? = null
        private set

    init {
        fixedRateTimer(period = 20L) {
            update()
        }
    }

    fun addSamples(creationTime: Time, samples: Iterable<Pose2d>) =
        addSamples(samples.map { TrackedTargetSample(creationTime, it) })

    fun addSamples(samples: Iterable<TrackedTargetSample>) = samples.forEach {
        addSample(it)
    }

    fun addSample(sample: TrackedTargetSample) = synchronized(targets) {
        if (sample.creationTime.second >= Timer.getFPGATimestamp()) return@synchronized // Cannot predict the future

        val closestTarget = targets.minBy {
            it.averagedPose2d.translation.distance(sample.targetPose.translation)
        }
        if (closestTarget == null
            || closestTarget.averagedPose2d.translation.distance(sample.targetPose.translation) > Constants.kTargetTrackingDistanceErrorTolerance.value
        ) {
            // Create new target if no targets are within tolerance
            targets += TrackedTarget(sample)
        } else {
            // Add sample to target within tolerance
            closestTarget.addSample(sample)
        }
    }

    fun update() = synchronized(targets) {
        val currentTime = Timer.getFPGATimestamp().second

        val currentRobotPose = DriveSubsystem.localization()

        // Update and remove old targets
        targets.removeIf {
            it.update(currentTime, currentRobotPose)
            !it.isAlive
        }

        bestTargetBack = targets.asSequence()
            .filter { it.averagedPose2dRelativeToBot.translation.x.value < 0.0 }
            .minBy { it.averagedPose2dRelativeToBot.translation.norm.value }

        // Sort the targets from best to worst
        val sortedFrontTargets = targets.asSequence()
            .filter { it.averagedPose2dRelativeToBot.translation.x.value < 0.0 }
            .toMutableList()

        sortedFrontTargets.sortBy { it.averagedPose2dRelativeToBot.translation.norm.value }

        // Choose the top two best targets
        val bestFrontTarget1 = sortedFrontTargets.getOrNull(0)
        val bestFrontTarget2 = sortedFrontTargets.getOrNull(1)

        bestTargetFront = bestFrontTarget1

        // Make the left one best left and right one best right
        if (bestFrontTarget1 == null || bestFrontTarget2 == null) {
            this.bestTargetFrontLeft = null
            this.bestTargetFrontRight = null
        } else {
            if (bestFrontTarget1.averagedPose2dRelativeToBot.translation.y <
                bestFrontTarget2.averagedPose2dRelativeToBot.translation.y
            ) {
                bestTargetFrontLeft = bestFrontTarget2
                bestTargetFrontRight = bestFrontTarget1
            } else {
                bestTargetFrontLeft = bestFrontTarget1
                bestTargetFrontRight = bestFrontTarget2
            }
        }

        // Publish to dashboard
        LiveDashboard.visionTargets = targets.asSequence()
            .filter { it.isReal }
            .map { it.averagedPose2d }
            .toList()
    }

    class TrackedTarget(
        initialTargetSample: TrackedTargetSample
    ) {

        private val samples = mutableSetOf<TrackedTargetSample>()

        /**
         * The averaged pose2d for x time
         */
        var averagedPose2d = initialTargetSample.targetPose
            private set

        var averagedPose2dRelativeToBot = Pose2d()
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

        fun addSample(newSamples: TrackedTargetSample) = synchronized(samples) {
            samples.add(newSamples)
        }

        fun update(currentTime: Time, currentRobotPose: Pose2d) = synchronized(samples) {
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
            averagedPose2dRelativeToBot = averagedPose2d inFrameOfReferenceOf currentRobotPose
        }

    }

    data class TrackedTargetSample(
        val creationTime: Time,
        val targetPose: Pose2d
    )

}