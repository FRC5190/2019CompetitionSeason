/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.subsystems.drive

import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getRawButton
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import org.ghrobotics.lib.wrappers.hid.kX

class ManualDriveCommand : FalconCommand(DriveSubsystem) {

    private var quickStopAccumulator = 0.0
    private var prevVelocity = DifferentialDrive.WheelState(0.0, 0.0)

    init {
        executeFrequency = 50
    }

    override suspend fun initialize() {
        prevVelocity = DifferentialDrive.WheelState(0.0, 0.0)
    }

    override suspend fun execute() {
        DriveSubsystem.curvatureDrive(
            -speedSource(),
            rotationSource(),
            quickTurnSource()
        )
    }

    companion object {
        private const val kDeadband = 0.02
        val speedSource = Controls.mainXbox.getY(GenericHID.Hand.kLeft).withDeadband(kDeadband)
        private val rotationSource = Controls.mainXbox.getX(GenericHID.Hand.kLeft).withDeadband(kDeadband)
        private val quickTurnSource = Controls.mainXbox.getRawButton(kX)
    }
}
