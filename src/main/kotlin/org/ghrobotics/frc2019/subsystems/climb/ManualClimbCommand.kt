package org.ghrobotics.frc2019.subsystems.climb

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getY
import kotlin.math.absoluteValue

class ManualClimbCommand : FalconCommand(ClimbSubsystem) {

    override suspend fun initialize() {
        ClimbSubsystem.zeroOutputs()
    }

    override suspend fun execute() {
        if (Controls.isClimbing) {
            ClimbSubsystem.wantedWheelPercentOutput = wheelSource()

            if (frontWinchSource().absoluteValue > 0.1) {
                if (ClimbSubsystem.currentFrontWinchState !is ClimbSubsystem.ClimbLegState.OpenLoop) {
                    ClimbSubsystem.wantedFrontWinchState =
                        ClimbSubsystem.ClimbLegState.OpenLoop { -frontWinchSource() }
                }
            } else {
                if (ClimbSubsystem.currentFrontWinchState !is ClimbSubsystem.ClimbLegState.Position) {
                    ClimbSubsystem.wantedFrontWinchState =
                        ClimbSubsystem.ClimbLegState.Position(ClimbSubsystem.frontWinchPosition)
                }
            }

            if (backWinchSource().absoluteValue > 0.1) {
                if (ClimbSubsystem.currentBackWinchState !is ClimbSubsystem.ClimbLegState.OpenLoop) {
                    ClimbSubsystem.wantedBackWinchState =
                        ClimbSubsystem.ClimbLegState.OpenLoop { -backWinchSource() }
                }
            } else {
                if (ClimbSubsystem.currentBackWinchState !is ClimbSubsystem.ClimbLegState.Position) {
                    ClimbSubsystem.wantedBackWinchState =
                        ClimbSubsystem.ClimbLegState.Position(ClimbSubsystem.backWinchPosition)
                }
            }
        } else {
            ClimbSubsystem.wantedFrontWinchState = ClimbSubsystem.ClimbLegState.Nothing
            ClimbSubsystem.wantedBackWinchState = ClimbSubsystem.ClimbLegState.Nothing
            ClimbSubsystem.wantedWheelPercentOutput = 0.0
        }
    }

    private companion object {
        val frontWinchSource = Controls.operatorFalconXbox.getY(GenericHID.Hand.kLeft)
        val backWinchSource = Controls.operatorFalconXbox.getY(GenericHID.Hand.kRight)
        val wheelSource = Controls.driverFalconXbox.getY(GenericHID.Hand.kRight).withDeadband(0.1)
    }
}
