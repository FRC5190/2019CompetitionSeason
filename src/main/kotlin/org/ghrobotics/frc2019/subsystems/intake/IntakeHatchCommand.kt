package org.ghrobotics.frc2019.subsystems.intake

import org.ghrobotics.lib.commands.FalconCommand

class IntakeHatchCommand(
    private val direction: IntakeSubsystem.Direction
) : FalconCommand(IntakeSubsystem) {

    init {
        finishCondition += { true }
    }

    override suspend fun initialize() {
        if (direction == IntakeSubsystem.Direction.HOLD) {
            IntakeSubsystem.extensionSolenoid.set(true)
        } else {
            IntakeSubsystem.extensionSolenoid.set(false)
        }
    }

}