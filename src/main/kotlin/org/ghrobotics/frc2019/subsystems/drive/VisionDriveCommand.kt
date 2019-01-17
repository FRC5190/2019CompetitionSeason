package org.ghrobotics.frc2019.subsystems.drive

import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.frc2019.vision.TrackedTarget
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Rotation2d

class VisionDriveCommand : FalconCommand(DriveSubsystem) {

    lateinit var target: TrackedTarget
    private var foundTarget = false

    init {
        finishCondition += ::foundTarget
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

        val turn = kCorrectionKp * angle.radian
        val source = -ManualDriveCommand.speedSource()
        DriveSubsystem.tankDrive(source - turn, source + turn)
    }

    companion object {
        const val kCorrectionKp = 0.3
    }
}