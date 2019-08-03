package org.ghrobotics.frc2019.subsystems.drive

import org.ghrobotics.frc2019.Network
import org.ghrobotics.frc2019.subsystems.elevator.ElevatorSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.frc2019.vision.LimelightManager
import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.radian
import kotlin.math.absoluteValue

class VisionDriveCommand(private val targetSide: TargetSide) : ManualDriveCommand() {

    private var referencePose = Pose2d()
    private var lastKnownTargetPose: Pose2d? = null

    private var prevError = 0.0

    override suspend fun initialize() {
        if (targetSide == TargetSide.FRONT) {
            LimelightManager.turnOnLED()
        }
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
            ElevatorSubsystem.wantedVisionMode = true
            super.execute()
        } else {
            ElevatorSubsystem.wantedVisionMode = false
            val transform = lastKnownTargetPose inFrameOfReferenceOf IntakeSubsystem.robotPositionWithIntakeOffset
            val angle = Rotation2d(transform.translation.x, transform.translation.y, true)

            Network.visionDriveAngle.setDouble(angle.degree)
            Network.visionDriveActive.setBoolean(true)

            val angleError = angle + if (targetSide == TargetSide.FRONT) Rotation2d.kZero else Math.PI.radian

            if (angleError.degree.absoluteValue > 45) {
                // plz no disable us when going to loading station, kthx
                this.lastKnownTargetPose = null
            }

            val error = angleError.radian

            val turn = kCorrectionKp * error + kCorrectionKd * (error - prevError)
            tankDrive(source - turn, source + turn)

            prevError = error
        }
    }

    override suspend fun dispose() {
        if (targetSide == TargetSide.FRONT) {
            LimelightManager.turnOffLED()
        }
        Network.visionDriveActive.setBoolean(false)
        this.lastKnownTargetPose = null
        ElevatorSubsystem.wantedVisionMode = false
        isActive = false
    }

    enum class TargetSide { FRONT, BACK }

    companion object {
        const val kCorrectionKp = 0.8
        const val kCorrectionKd = 8.0
        var isActive = false
            private set
    }
}