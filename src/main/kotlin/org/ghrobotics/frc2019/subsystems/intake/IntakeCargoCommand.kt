package org.ghrobotics.frc2019.subsystems.intake

import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
import org.ghrobotics.lib.commands.ConditionalCommand
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.degree

//class IntakeCargoCommand(
//    private val releasing: Boolean
//) : FalconCommand(IntakeSubsystem) {
//
//    var gotCargo = false
//        private set
//
//    init {
//        if (!releasing) {
//            finishCondition += { gotCargo }
//        }
//    }
//
//    override suspend fun initialize() {
//        gotCargo = false
//        if (releasing) {
//            IntakeSubsystem.wantedPercentOutput = -1.0
//        } else {
//            IntakeSubsystem.wantedPercentOutput = 1.0
//        }
//    }
//
//    override suspend fun execute() {
//        if (!releasing) {
//            if (IntakeSubsystem.isSeeingCargo) {
//                gotCargo = true
//            }
//        }
//    }
//
//    override suspend fun dispose() {
//        IntakeSubsystem.zeroOutputs()
//    }
//
//}

class IntakeCargoCommand(
    private val releasing: Boolean
) : FalconCommand(IntakeSubsystem) {

    private var sensedBall = 0L

    init {
        if (!releasing) {
            finishCondition += { sensedBall != 0L && System.currentTimeMillis() - sensedBall > 500 }
        }
    }

    private var startTime = 0L

    override suspend fun initialize() {
        sensedBall = 0L
        startTime = System.currentTimeMillis()

        // Don't launch yet
        IntakeSubsystem.wantedLauncherSolenoidState = false

        if (releasing) {
            IntakeSubsystem.wantedPercentOutput = -1.0
            IntakeSubsystem.wantedExtensionSolenoidState = IntakeSubsystem.ExtensionSolenoidState.RETRACTED
        } else {
            IntakeSubsystem.wantedExtensionSolenoidState = IntakeSubsystem.ExtensionSolenoidState.EXTENDED
            IntakeSubsystem.wantedPercentOutput = 1.0
        }
    }

    override suspend fun execute() {
        when (releasing) {
            true -> {
                if ((System.currentTimeMillis() - startTime > 250 && !IntakeSubsystem.launcherSolenoidState)) {
                    IntakeSubsystem.wantedLauncherSolenoidState = true
                }
            }
            false -> {
                if (IntakeSubsystem.isSeeingCargo && sensedBall == 0L) {
                    IntakeSubsystem.wantedExtensionSolenoidState = IntakeSubsystem.ExtensionSolenoidState.RETRACTED
                    sensedBall = System.currentTimeMillis()
                }
            }
        }
    }

    override suspend fun dispose() {
        IntakeSubsystem.wantedLauncherSolenoidState = false
        IntakeSubsystem.wantedExtensionSolenoidState = IntakeSubsystem.ExtensionSolenoidState.RETRACTED
        IntakeSubsystem.zeroOutputs()
    }
}