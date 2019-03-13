package org.ghrobotics.frc2019.subsystems.arm

import org.ghrobotics.lib.commands.FalconCommand

class TuneArmVelocityCommand : FalconCommand(ArmSubsystem) {

    init {
        finishCondition += {
            ArmSubsystem._position.cos <= 0.0
        }
    }

    private var lastVelocity = 0.0

    private var accelerationSamples = mutableListOf<Double>()
    private var maxVelocity = 0.0

    override suspend fun initialize() {
        accelerationSamples.clear()
        maxVelocity = 0.0
    }

    override suspend fun execute() {
        ArmSubsystem.setPercentOutput(1.0, false)
        val arbFF = ArmSubsystem.arbitraryFeedForward
        if(arbFF >= 0.0) {
            val appliedVoltage = ArmSubsystem.voltage - ArmSubsystem.arbitraryFeedForward
            val velocity = ArmSubsystem.velocity.value
            if(velocity > maxVelocity) {
                maxVelocity = velocity
            }
            val acceleration = (velocity - lastVelocity) * 50
            accelerationSamples.add(acceleration)
            lastVelocity = velocity
            println("$appliedVoltage\t$velocity\t$acceleration")
        }
    }

    override suspend fun dispose() {
        ArmSubsystem.zeroOutputs()
        println("Velocity: $maxVelocity rad/s Acceleration: ${accelerationSamples.average()} rad/s/s")
    }

}