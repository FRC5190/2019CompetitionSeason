package org.ghrobotics.frc2019.subsystems.drive

import org.ghrobotics.frc2019.Network
import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.frc2019.vision.TrackedTarget
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Rotation2d

class VisionDriveCommand : FalconCommand(DriveSubsystem) {

    lateinit var target: TrackedTarget
    private var foundTarget = false

    init {
        finishCondition += { !foundTarget }
    }

    override suspend fun initialize() {
        val target = TargetTracker.bestTarget
        if (target == null) {
            foundTarget = false
        } else {
            this.target = target
            foundTarget = true
        }
    }

    override suspend fun execute() {
        if (!foundTarget) return

        val transform = target.averagePose inFrameOfReferenceOf DriveSubsystem.localization()
        val angle = Rotation2d(transform.translation.x.value, transform.translation.y.value, true)

        Network.visionDriveAngle.setDouble(angle.degree)
        Network.visionDriveActive.setBoolean(true)

        val turn = kCorrectionKp * angle.radian
        val source = -ManualDriveCommand.speedSource()
        DriveSubsystem.tankDrive(source - turn, source + turn)
    }

    override suspend fun dispose() {
        Network.visionDriveActive.setBoolean(false)
    }

    companion object {
        const val kCorrectionKp = 1.0
    }
}