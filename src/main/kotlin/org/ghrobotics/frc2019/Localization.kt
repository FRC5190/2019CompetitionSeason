package org.ghrobotics.frc2019

import org.ghrobotics.frc2019.auto.paths.TrajectoryWaypoints
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.feet

object Localization {
    var robotPosition = TrajectoryWaypoints.kSideStart

    var prevL = 0.feet
    var prevR = 0.feet

    fun update() {
        val currentL = DriveSubsystem.leftMotor.sensorPosition
        val currentR = DriveSubsystem.rightMotor.sensorPosition

        val angle = DriveSubsystem.pigeon.fusedHeading

        val dl = currentL - prevL
        val dr = currentR - prevR

        val d = (dl + dr) / 2.0

        val dx = d * Math.cos(Math.toRadians(angle))
        val dy = d * Math.sin(Math.toRadians(angle))

        robotPosition = Pose2d(robotPosition.translation + Translation2d(dx, dy), angle.degree)

        prevL = currentL
        prevR = currentR

        println("x: ${robotPosition.translation.x.feet}, y: ${robotPosition.translation.y.feet}")
    }
}