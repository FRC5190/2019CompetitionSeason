/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.subsystems.drive

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getRawButton
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import org.ghrobotics.lib.wrappers.hid.kX

class ManualDriveCommand : FalconCommand(DriveSubsystem) {

    init {
        executeFrequency = 50
    }

    override suspend fun execute() {
//        DriveSubsystem.tankDrive(
//            -leftSource(),
//            -rightSource()
//        )
        DriveSubsystem.curvatureDrive(
            -speedSource(),
            rotationSource(),
            quickTurnSource()
        )
    }

    companion object {
        private const val kDeadband = 0.02
        val speedSource = Controls.driverXbox.getY(GenericHID.Hand.kLeft).withDeadband(kDeadband)
        val rightSource = Controls.driverXbox.getY(GenericHID.Hand.kRight).withDeadband(kDeadband)
        val leftSource = Controls.driverXbox.getY(GenericHID.Hand.kLeft).withDeadband(kDeadband)
        private val rotationSource = Controls.driverXbox.getX(GenericHID.Hand.kLeft).withDeadband(kDeadband)
        private val quickTurnSource = Controls.driverXbox.getRawButton(kX)
    }
}
