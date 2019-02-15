package org.ghrobotics.lib.mathematics.twodim.control

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.radian
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput
import org.ghrobotics.lib.utils.DeltaTime

/**
 * https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf section 6, polar tracker
 */
class PolarPostureStabilizer(
    val k1: Double,
    val k2: Double,
    val k3: Double
) {

    private var target: Pose2d = Pose2d()

    private var deltaTimeController = DeltaTime()

    var previousLinearVelocity = 0.meter.velocity
    var previousAngularVelocity = 0.radian.velocity

    fun reset(target: Pose2d) {
        this.target = target
        deltaTimeController.reset()
        previousLinearVelocity = 0.meter.velocity
        previousAngularVelocity = 0.radian.velocity
    }

    fun nextState(robotPose: Pose2d, currentTime: Time = System.currentTimeMillis().millisecond): TrajectoryTrackerOutput {
        val deltaTime = deltaTimeController.updateTime(currentTime)

        val robotRelativeToOrigin = robotPose inFrameOfReferenceOf target
        val theta = robotPose.rotation.radian

        val row = robotRelativeToOrigin.translation.norm.meter
        val gamma = (Math.atan2(
            robotRelativeToOrigin.translation.y.value,
            robotRelativeToOrigin.translation.x.value
        ) - theta + Math.PI).radian.value
        val sigma = (gamma + theta).radian.value

        val (linear, angular) =
            (k1 * row * Math.cos(gamma)).meter.velocity to
                (k2 * gamma + k1 * Math.sin(gamma) * Math.cos(gamma) / gamma * (gamma + k3 * sigma)).radian.velocity

        println(linear.value)

        return if (deltaTime.value <= 0) {
            TrajectoryTrackerOutput(
                linearVelocity = linear,
                linearAcceleration = 0.meter.acceleration,
                angularVelocity = angular,
                angularAcceleration = 0.radian.acceleration
            )
        } else {
            TrajectoryTrackerOutput(
                linearVelocity = linear,
                linearAcceleration = (linear - previousLinearVelocity) / deltaTime,
                angularVelocity = angular,
                angularAcceleration = (angular - previousAngularVelocity) / deltaTime
            )
        }
    }
}