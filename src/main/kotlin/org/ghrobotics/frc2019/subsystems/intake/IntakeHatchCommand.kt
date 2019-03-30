package org.ghrobotics.frc2019.subsystems.intake

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.millisecond

//class IntakeHatchCommand(
//    private val releasing: Boolean
//) : FalconCommand(IntakeSubsystem) {
//
//    override suspend fun initialize() {
//        if (releasing) {
//            IntakeSubsystem.wantedHoldHatchSolenoidState = IntakeSubsystem.HoldHatchSolenoidState.PLACE
//        } else {
//            IntakeSubsystem.wantedHoldHatchSolenoidState = IntakeSubsystem.HoldHatchSolenoidState.HOLD
//        }
//    }
//}

//class IntakeHatchCommand(
//    private val releasing: Boolean
//) : FalconCommand(IntakeSubsystem) {
//
//    init {
//        withTimeout(300.millisecond)
//    }
//
//    override suspend fun initialize() {
//        if (releasing) {
//            IntakeSubsystem.wantedExtensionSolenoidState = IntakeSubsystem.ExtensionSolenoidState.RETRACTED
//        } else {
//            IntakeSubsystem.wantedExtensionSolenoidState = IntakeSubsystem.ExtensionSolenoidState.EXTENDED
//            IntakeSubsystem.wantedPercentOutput = 0.25
//        }
//    }
//
//    override suspend fun dispose() {
//        IntakeSubsystem.zeroOutputs()
//    }
//}

class IntakeHatchCommand(
    private val releasing: Boolean
) : FalconCommand(IntakeSubsystem) {

    init {
//        withTimeout(300.millisecond)
    }

    override suspend fun initialize() {
        if (releasing) {
            IntakeSubsystem.wantedExtensionSolenoidState = IntakeSubsystem.ExtensionSolenoidState.EXTENDED
            IntakeSubsystem.wantedPercentOutput = 1.0
        } else {
            IntakeSubsystem.wantedExtensionSolenoidState = IntakeSubsystem.ExtensionSolenoidState.RETRACTED
            IntakeSubsystem.wantedPercentOutput = -1.0
        }
    }

    override suspend fun dispose() {
        if (releasing) {
            IntakeSubsystem.wantedExtensionSolenoidState = IntakeSubsystem.ExtensionSolenoidState.RETRACTED
        }
        IntakeSubsystem.zeroOutputs()
    }
}