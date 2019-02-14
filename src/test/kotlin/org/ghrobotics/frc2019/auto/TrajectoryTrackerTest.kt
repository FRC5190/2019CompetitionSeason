package org.ghrobotics.frc2019.auto

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.auto.paths.TrajectoryFactory
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Twist2d
import org.ghrobotics.lib.mathematics.units.*
import org.junit.Test

class TrajectoryTrackerTest {
    @Test
    fun testFollowerReachesGoal() {
        val tracker = RamseteTracker(Constants.kDriveBeta, Constants.kDriveZeta)
        val path = TrajectoryFactory.loadingStationToRocketF

        val initialPose = path.firstState.state.pose.transformBy(Pose2d(1.inch, 2.inch, 10.degree))
        tracker.reset(path)

        var pose = initialPose
        var t = 0.second

        val dt = 20.millisecond

        while (!tracker.isFinished) {
            val output = tracker.nextState(pose, t)

            val linear = output.linearVelocity
            val angular = output.angularVelocity

            val twist = Twist2d(linear.value.meter, 0.meter, angular.value.radian * 0.95) * dt.value
            pose += twist.asPose

            t += dt
        }

        val tErrorNorm = (pose.translation - path.lastState.state.pose.translation).norm
        assert(tErrorNorm.value < 0.1)
    }
}