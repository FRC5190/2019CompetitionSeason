/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.robot.subsytems.drive

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.robot.Controls
import org.ghrobotics.frc2019.robot.vision.VisionProcessing
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.*
import kotlin.math.absoluteValue

class ManualDriveCommand : FalconCommand(DriveSubsystem) {
    companion object {
        private const val deadband = 0.02
        private val speedSource = Controls.mainXbox.getY(GenericHID.Hand.kLeft).withDeadband(deadband)
        private val rotationSource = Controls.mainXbox.getX(GenericHID.Hand.kLeft).withDeadband(deadband)
        private val quickTurnSource = Controls.mainXbox.getRawButton(kX)
        private val visionAssistSource = Controls.mainXbox.getRawButton(kA)
    }

    override suspend fun execute() {
        val location = VisionProcessing.currentlyTrackedObject.objectLocationRelativeToRobot.translation
        val angle = Rotation2d(location.x.value, location.y.value, true)
        if (visionAssistSource() && angle.degree.absoluteValue < 30.0) {
            DriveSubsystem.curvatureDrive(
                -speedSource(),
                -location.y.value,
                false
            )
        } else {
            DriveSubsystem.curvatureDrive(
                -speedSource(),
                rotationSource(),
                quickTurnSource()
            )
        }
    }
}
