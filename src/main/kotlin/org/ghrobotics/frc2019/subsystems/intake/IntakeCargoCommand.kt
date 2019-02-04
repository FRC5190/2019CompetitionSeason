package org.ghrobotics.frc2019.subsystems.intake

import org.ghrobotics.lib.commands.FalconCommand

class IntakeCargoCommand(
    private val direction: IntakeSubsystem.Direction
) : FalconCommand(IntakeSubsystem) {

    init {
        if (direction == IntakeSubsystem.Direction.HOLD) {
            finishCondition += IntakeSubsystem.isHoldingCargo
        }
    }

    private var startTime = 0L

    override suspend fun initialize() {
        startTime = System.currentTimeMillis()

        // Don't launch yet
        IntakeSubsystem.launcherSolenoid.set(false)

        if (direction == IntakeSubsystem.Direction.HOLD) {
            IntakeSubsystem.extensionSolenoid.set(true)
            IntakeSubsystem.percentOutput = 1.0
        } else {

            IntakeSubsystem.percentOutput = -1.0
        }
    }

    override suspend fun execute() {
        if (direction == IntakeSubsystem.Direction.RELEASE) {
            if (
                System.currentTimeMillis() - startTime > 250
                && !IntakeSubsystem.launcherSolenoid.get()
            ) {
                IntakeSubsystem.launcherSolenoid.set(true)
            }
        }
    }

    override suspend fun dispose() {
        IntakeSubsystem.launcherSolenoid.set(false)
        IntakeSubsystem.extensionSolenoid.set(false)
        IntakeSubsystem.zeroOutputs()
    }

}