package org.ghrobotics.frc2019.subsystems.intake

import edu.wpi.first.wpilibj.DoubleSolenoid
import org.ghrobotics.lib.commands.FalconCommand

class IntakeCloseCommand : FalconCommand(IntakeSubsystem) {

    init {
        finishCondition += { true }
    }

    override suspend fun initialize() {
        IntakeSubsystem.extensionSolenoid.set(DoubleSolenoid.Value.kReverse)
        IntakeSubsystem.launcherSolenoid.set(false)
    }

}