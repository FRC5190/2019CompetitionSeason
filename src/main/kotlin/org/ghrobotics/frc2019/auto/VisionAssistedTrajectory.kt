package org.ghrobotics.frc2019.auto

import org.ghrobotics.frc2019.vision.DynamicObject
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryIterator
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TrajectorySamplePoint
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.feet

/**
 *  The goal of [VisionAssistedTrajectory] is to slowly ease into vision targets while the
 *  robot is following a trajectory.
 *
 *  @param originalTrajectory the trajectory you wish to follow
 *  @param dynamicObject the vision target object
 *  @param expectedTargetLocation where the vision target is supposed to be on field
 *  @param maxErrorRadius the allowed error in the vision target location
 */
class VisionAssistedTrajectory(
    private val originalTrajectory: TimedTrajectory<Pose2dWithCurvature>,
    private val dynamicObject: DynamicObject,
    private val expectedTargetLocation: Translation2d,
    private val maxErrorRadius: Length = 5.feet
) : Trajectory<Time, TimedEntry<Pose2dWithCurvature>> {

    private var visionOffset = Translation2d()

    override val firstInterpolant: Time
        get() = originalTrajectory.firstInterpolant
    override val firstState: TimedEntry<Pose2dWithCurvature>
        get() = originalTrajectory.firstState

    override val lastInterpolant: Time
        get() = originalTrajectory.lastInterpolant
    override val lastState: TimedEntry<Pose2dWithCurvature>
        get() = modify(originalTrajectory.lastState)

    override val points: List<TimedEntry<Pose2dWithCurvature>>
        get() = originalTrajectory.points.map { modify(it) }

    override fun iterator(): TrajectoryIterator<Time, TimedEntry<Pose2dWithCurvature>> =
        object : TrajectoryIterator<Time, TimedEntry<Pose2dWithCurvature>>(this) {
            override fun addition(a: Time, b: Time): Time = a + b
        }

    override fun sample(interpolant: Time): TrajectorySamplePoint<TimedEntry<Pose2dWithCurvature>> {
        updateOffset()
        val originalSample = originalTrajectory.sample(interpolant)
        return TrajectorySamplePoint(
            modify(originalSample.state),
            originalSample.indexFloor,
            originalSample.indexCeil
        )
    }

    private fun modify(originalEntry: TimedEntry<Pose2dWithCurvature>): TimedEntry<Pose2dWithCurvature> {
        val percent =
            (originalEntry.t.value - firstInterpolant.value) / (lastInterpolant.value - firstInterpolant.value)
        return TimedEntry(
            Pose2dWithCurvature(
                Pose2d(
                    originalEntry.state.pose.translation + visionOffset * percent,
                    originalEntry.state.pose.rotation
                ),
                originalEntry.state.curvature
            ),
            originalEntry.t,
            originalEntry.velocity,
            originalEntry.acceleration
        )
    }

    private fun updateOffset() {
        val dynamicObjectLocation = dynamicObject.objectLocationOnField.translation
        // Only adjust when the target is within an error range (this is to help prevent tracking wrong targets)
        if (dynamicObjectLocation.distance(expectedTargetLocation) < maxErrorRadius.value) {
            visionOffset = visionOffset.interpolate(
                dynamicObjectLocation - expectedTargetLocation,
                0.1
            )
        }
    }


}