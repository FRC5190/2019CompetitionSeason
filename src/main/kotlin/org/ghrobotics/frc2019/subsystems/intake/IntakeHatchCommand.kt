package org.ghrobotics.frc2019.subsystems.intake

import edu.wpi.first.wpilibj.DoubleSolenoid
import org.ghrobotics.lib.commands.FalconCommand

class IntakeHatchCommand(
    private val direction: IntakeSubsystem.Direction
) : FalconCommand(IntakeSubsystem) {

    init {
        finishCondition += { true }
    }

    override suspend fun initialize() {
        if (direction == IntakeSubsystem.Direction.HOLD) {
            IntakeSubsystem.extensionSolenoid.set(DoubleSolenoid.Value.kForward)
        } else {
            IntakeSubsystem.extensionSolenoid.set(DoubleSolenoid.Value.kReverse)
        }
    }

}