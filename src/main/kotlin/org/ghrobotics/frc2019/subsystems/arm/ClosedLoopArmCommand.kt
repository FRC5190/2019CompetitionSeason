package org.ghrobotics.frc2019.subsystems.arm

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Rotation2d

class ClosedLoopArmCommand(private val target: Rotation2d) : FalconCommand(ArmSubsystem) {
    init {
        finishCondition += {
            (ArmSubsystem.armPosition - target).absoluteValue < Constants.kArmClosedLoopTolerance
        }
    }

    override suspend fun initialize() {
        ArmSubsystem.armPosition = target
    }
}