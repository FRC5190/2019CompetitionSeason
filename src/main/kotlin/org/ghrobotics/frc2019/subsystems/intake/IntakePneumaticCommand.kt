/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.subsystems.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.BooleanSource

class IntakePneumaticCommand(val state: BooleanSource) : FalconCommand(IntakeSubsystem) {
    init {
        finishCondition += { true }
    }

    override suspend fun initialize() {
        IntakeSubsystem.solenoid.set(state())
        IntakeSubsystem.set(ControlMode.PercentOutput, 0.0)
    }
}