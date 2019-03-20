package org.ghrobotics.frc2019.subsystems.drive

import org.ghrobotics.frc2019.Network
import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.radian

class VisionDriveCommand(private val targetSide: TargetSide) : ManualDriveCommand() {

    private var referencePose = Pose2d()
    private var lastKnownTargetPose: Pose2d? = null

    override suspend fun initialize() {
        isActive = true
        referencePose = DriveSubsystem.robotPosition
    }

    override suspend fun execute() {
        val newTarget = TargetTracker.getBestTargetUsingReference(referencePose, targetSide == TargetSide.FRONT)

        val newPose = newTarget?.averagedPose2d
        if (newTarget?.isAlive == true && newPose != null) lastKnownTargetPose = newPose

        val lastKnownTargetPose = this.lastKnownTargetPose

        val source = -speedSource()

        if (lastKnownTargetPose == null) {
            super.execute()
        } else {
            val transform = lastKnownTargetPose inFrameOfReferenceOf DriveSubsystem.localization()
            val angle = Rotation2d(transform.translation.x, transform.translation.y, true)

            Network.visionDriveAngle.setDouble(angle.degree)
            Network.visionDriveActive.setBoolean(true)

            val turn =
                kCorrectionKp * (angle + if (targetSide == TargetSide.FRONT) Rotation2d.kZero else Math.PI.radian).radian
            tankDrive(source - turn, source + turn)
        }
    }

    override suspend fun dispose() {
        Network.visionDriveActive.setBoolean(false)
        this.lastKnownTargetPose = null
        isActive = false
    }

    enum class TargetSide { FRONT, BACK }

    companion object {
        const val kCorrectionKp = 0.2
        var isActive = false
            private set
    }
}