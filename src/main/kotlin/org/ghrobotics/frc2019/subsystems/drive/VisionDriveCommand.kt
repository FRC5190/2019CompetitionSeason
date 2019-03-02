package org.ghrobotics.frc2019.subsystems.drive

import org.ghrobotics.frc2019.Network
import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.radian
import kotlin.math.absoluteValue

class VisionDriveCommand(private val targetSide: TargetSide) : FalconCommand(DriveSubsystem) {

    private var currentTarget: TargetTracker.TrackedTarget? = null
    private var lastKnownPose: Pose2d? = null

    override suspend fun initialize() {
        isActive = true
    }

    override suspend fun execute() {
        val newTarget = (if (targetSide == TargetSide.FRONT) {
            TargetTracker.bestTargetFront
        } else TargetTracker.bestTargetBack)

        if (newTarget != null) currentTarget = newTarget

        val source = -ManualDriveCommand.speedSource()

        val newPose = currentTarget?.averagedPose2d
        if(currentTarget?.isAlive == true && newPose != null) lastKnownPose = newPose

        val lastKnownPose = this.lastKnownPose

        if (lastKnownPose == null) {
            DriveSubsystem.tankDrive(source, source)
        } else {
            val transform = lastKnownPose inFrameOfReferenceOf DriveSubsystem.localization()
            val angle = Rotation2d(transform.translation.x.value, transform.translation.y.value, true)

            Network.visionDriveAngle.setDouble(angle.degree)
            Network.visionDriveActive.setBoolean(true)

//            val turn =
//                kCorrectionKp * (transform.translation.y.value / transform.translation.x.value.absoluteValue) * (if (targetSide == TargetSide.FRONT) 1.0 else -1.0)
            val turn = kCorrectionKp * (angle + if (targetSide == TargetSide.FRONT) Rotation2d.kZero else Math.PI.radian).radian
            DriveSubsystem.tankDrive(source - turn, source + turn)
        }
    }

    override suspend fun dispose() {
        Network.visionDriveActive.setBoolean(false)
        this.currentTarget = null
        isActive = false
    }

    enum class TargetSide { FRONT, BACK }

    companion object {
        const val kCorrectionKp = 0.35
        var isActive = false
            private set
    }
}