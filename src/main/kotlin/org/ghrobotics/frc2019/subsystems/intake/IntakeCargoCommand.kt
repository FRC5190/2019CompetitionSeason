package org.ghrobotics.frc2019.subsystems.intake

import org.ghrobotics.lib.commands.FalconCommand

class IntakeCargoCommand(
    private val direction: IntakeSubsystem.Direction
) : FalconCommand(IntakeSubsystem) {

    private var sensedBall = 0L

    init {
        if (direction == IntakeSubsystem.Direction.HOLD) {
            finishCondition += { sensedBall != 0L && System.currentTimeMillis() - sensedBall > 500 }
        }
    }

    private var startTime = 0L

    override suspend fun initialize() {
        sensedBall = 0L
        startTime = System.currentTimeMillis()

        // Don't launch yet
        IntakeSubsystem.launcherSolenoid.set(false)

        if (direction == IntakeSubsystem.Direction.HOLD) {
            IntakeSubsystem.extensionSolenoid.set(true)
            IntakeSubsystem.percentOutput = 1.0
        } else {
            IntakeSubsystem.extensionSolenoid.set(true)
        }
    }

    override suspend fun execute() {
        if (direction == IntakeSubsystem.Direction.RELEASE) {
            if (
                System.currentTimeMillis() - startTime > 500
                && !IntakeSubsystem.launcherSolenoid.get()
            ) {
                IntakeSubsystem.percentOutput = -1.0
            }
            if (
                System.currentTimeMillis() - startTime > 1000
                && !IntakeSubsystem.launcherSolenoid.get()
            ) {
                IntakeSubsystem.extensionSolenoid.set(false)
                IntakeSubsystem.launcherSolenoid.set(true)
            }
        }
        if(IntakeSubsystem.isHoldingCargo() && sensedBall == 0L) {
            IntakeSubsystem.extensionSolenoid.set(false)
            sensedBall = System.currentTimeMillis()
        }
    }

    override suspend fun dispose() {
        IntakeSubsystem.launcherSolenoid.set(false)
        IntakeSubsystem.extensionSolenoid.set(false)
        IntakeSubsystem.zeroOutputs()
    }

}