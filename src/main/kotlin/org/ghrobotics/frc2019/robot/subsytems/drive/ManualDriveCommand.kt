/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.robot.subsytems.drive

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.robot.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getRawButton
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import org.ghrobotics.lib.wrappers.hid.kX

class ManualDriveCommand : FalconCommand(DriveSubsystem) {
    companion object {
        private const val deadband = 0.02
        private val speedSource = Controls.mainXbox.getY(GenericHID.Hand.kLeft).withDeadband(deadband)
        private val rotationSource = Controls.mainXbox.getX(GenericHID.Hand.kLeft).withDeadband(deadband)
        private val quickTurnSource = Controls.mainXbox.getRawButton(kX)
    }

    override suspend fun execute() {
        DriveSubsystem.curvatureDrive(
            -speedSource(),
            rotationSource(),
            quickTurnSource()
        )
    }
}
