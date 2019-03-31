package org.ghrobotics.frc2019.subsystems.arm

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.degree
import kotlin.math.absoluteValue

object DefaultArmCommand : FalconCommand(ArmSubsystem) {
    private var lastHoldingCargoState = false

    override suspend fun initialize() {
        lastHoldingCargoState = IntakeSubsystem.isHoldingCargo

//        val currentState = ArmSubsystem.currentState
        val currentPosition = ArmSubsystem.position
//        val wantedPosition = if (currentState is ArmSubsystem.ArmState.SetPointState
//            && (currentState.position - currentPosition).value.absoluteValue <= Constants.kArmClosedLoopTolerance.value
//        ) {
//            currentState.position
//        } else {
//            currentPosition
//        }
        if(ArmSubsystem.currentState !is ArmSubsystem.ArmState.SetPointState) {
            ArmSubsystem.wantedState = ArmSubsystem.ArmState.MotionMagic(currentPosition)
        }
    }

    override suspend fun execute() {
//        val isHoldingCargo = IntakeSubsystem.isHoldingCargo
//        if (!lastHoldingCargoState && isHoldingCargo) {
//            ArmSubsystem.wantedState = ArmSubsystem.ArmState.Position(75.degree)
//        }
//        lastHoldingCargoState = isHoldingCargo
    }
}