package org.ghrobotics.frc2019.subsystems.climb

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.wrappers.hid.getY

class ManualClimbCommand : FalconCommand(ClimbSubsystem) {

    override suspend fun execute() {
        if (Controls.isClimbing) {
            ClimbSubsystem.frontWinchPercentOutput = frontWinchSource()
            ClimbSubsystem.backWinchPercentOutput = backWinchSource()
        } else {
            ClimbSubsystem.frontWinchPercentOutput = 0.0
            ClimbSubsystem.backWinchPercentOutput = 0.0
        }
    }

    private companion object {
        val frontWinchSource = Controls.operatorXbox.getY(GenericHID.Hand.kLeft)
        val backWinchSource = Controls.operatorXbox.getY(GenericHID.Hand.kRight)
    }
}