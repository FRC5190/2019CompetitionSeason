package org.ghrobotics.frc2019.subsystems.intake

import org.ghrobotics.lib.commands.FalconCommand

class IntakeHatchCommand(
    private val releasing: Boolean
) : FalconCommand(IntakeSubsystem) {

    override suspend fun initialize() {
        if (releasing) {
            IntakeSubsystem.wantedHoldHatchSolenoidState = IntakeSubsystem.HoldHatchSolenoidState.PLACE
            IntakeSubsystem.wantedPushHatchSolenoidState = IntakeSubsystem.PushHatchSolenoidState.USEFUL
        } else {
            IntakeSubsystem.wantedHoldHatchSolenoidState = IntakeSubsystem.HoldHatchSolenoidState.HOLD
            IntakeSubsystem.wantedPushHatchSolenoidState = IntakeSubsystem.PushHatchSolenoidState.EXIST
        }
    }

    override suspend fun dispose() {
        IntakeSubsystem.wantedHoldHatchSolenoidState = IntakeSubsystem.HoldHatchSolenoidState.HOLD
        IntakeSubsystem.wantedPushHatchSolenoidState = IntakeSubsystem.PushHatchSolenoidState.EXIST
    }

}

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