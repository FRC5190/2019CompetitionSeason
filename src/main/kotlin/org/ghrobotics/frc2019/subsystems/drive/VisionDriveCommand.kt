package org.ghrobotics.frc2019.subsystems.drive

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.Network
import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.frc2019.vision.TrackedTarget
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Rotation2d

class VisionDriveCommand : FalconCommand(DriveSubsystem) {

    private var currentTarget: TrackedTarget? = null

    init {
        finishCondition += { currentTarget == null }
    }

    override suspend fun initialize() {
        findNewTarget()
        isActive = true
    }

    private fun updateTarget() {
        val currentTarget = this.currentTarget ?: return
        if (!currentTarget.isAlive) {
            this.currentTarget = null
            findNewTarget()
        }
    }

    private fun findNewTarget() {
        val currentTarget = this.currentTarget
        // find new target
        if (currentTarget != null) {
            val newTarget = TargetTracker.trackedTargets.minBy {
                it.averagePose.translation.distance(currentTarget.averagePose.translation)
            } ?: return

            // switch over to new target if its close enough to original
            if (newTarget.averagePose.translation.distance(currentTarget.averagePose.translation)
                < Constants.kMaxTargetTrackingDistance.value * 2
            ) {
                this.currentTarget = newTarget
            }
        } else {
            this.currentTarget = TargetTracker.bestTargetFront
        }
    }

    override suspend fun execute() {
        updateTarget()
        val currentTarget = this.currentTarget ?: return

        val transform = currentTarget.averagePose inFrameOfReferenceOf DriveSubsystem.localization()
        val angle = Rotation2d(transform.translation.x.value, transform.translation.y.value, true)

        Network.visionDriveAngle.setDouble(angle.degree)
        Network.visionDriveActive.setBoolean(true)

        val turn = kCorrectionKp * angle.radian
        val source = -ManualDriveCommand.speedSource()
        DriveSubsystem.tankDrive(source - turn, source + turn)
    }

    override suspend fun dispose() {
        Network.visionDriveActive.setBoolean(false)
        this.currentTarget = null
        isActive = false
    }

    companion object {
        const val kCorrectionKp = 0.5
        var isActive = false
            private set
    }
}