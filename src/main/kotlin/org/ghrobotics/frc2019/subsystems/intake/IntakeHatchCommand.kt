package org.ghrobotics.frc2019.subsystems.intake

import edu.wpi.first.wpilibj.DoubleSolenoid
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.millisecond

class IntakeHatchCommand(
    private val direction: IntakeSubsystem.Direction
) : FalconCommand(IntakeSubsystem) {

    init {
        withTimeout(300.millisecond)
    }

    override suspend fun initialize() {
        if (direction == IntakeSubsystem.Direction.HOLD) {
            IntakeSubsystem.extensionSolenoid.set(DoubleSolenoid.Value.kForward)
            IntakeSubsystem.percentOutput = 0.25
        } else {
            IntakeSubsystem.extensionSolenoid.set(DoubleSolenoid.Value.kReverse)
        }
    }

    override suspend fun dispose() {
        IntakeSubsystem.zeroOutputs()
    }

}