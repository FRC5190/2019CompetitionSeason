package org.ghrobotics.frc2019.subsystems.arm

import org.apache.commons.math3.fitting.HarmonicCurveFitter
import org.apache.commons.math3.fitting.WeightedObservedPoint
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand

class TuneKgArmCommand : FalconCommand(ArmSubsystem) {

    private val samples = mutableSetOf<Pair<Int, Double>>()

    private var voltageOutput = 0.0
    private var startEncoderValue = 0

    private var maxVoltageForward = 0.0
    private var maxVoltageEncoderForward = 0

    private var maxVoltageBackward = 0.0
    private var maxVoltageEncoderBackward = 0

    private var isFindingForward = false

    init {
        finishCondition += {
            !isFindingForward &&
                Math.abs(ArmSubsystem.rawEncoder - startEncoderValue) > Constants.kArmSensorUnitsPerRotation.value / 2.0
        }
    }

    override suspend fun initialize() {
        voltageOutput = 0.0
        startEncoderValue = ArmSubsystem.rawEncoder
        isFindingForward = true
        maxVoltageForward = 0.0
        maxVoltageEncoderForward = 0
        maxVoltageBackward = 0.0
        maxVoltageEncoderBackward = 0
        println("START ARM TUNE")
    }

    override suspend fun execute() {
        if (isFindingForward) {
            if (ArmSubsystem.velocity.value > 0.25) {
                samples.add(ArmSubsystem.rawEncoder to voltageOutput)
                voltageOutput -= voltageStepDown
            } else {
                voltageOutput += voltageStepUp
                if (voltageOutput > maxVoltageForward) {
                    maxVoltageForward = voltageOutput
                    maxVoltageEncoderForward = ArmSubsystem.rawEncoder
                    println("New Forward Kg of $maxVoltageForward at $maxVoltageEncoderForward")
                }
            }
            if (Math.abs(ArmSubsystem.rawEncoder - startEncoderValue) > Constants.kArmSensorUnitsPerRotation.value / 2.0) {
                isFindingForward = false
                voltageOutput = 0.0
                startEncoderValue = ArmSubsystem.rawEncoder
            }
        } else {
            if (ArmSubsystem.velocity.value < -0.25) {
                voltageOutput += voltageStepDown
            } else {
                voltageOutput -= voltageStepUp
                if (voltageOutput < maxVoltageBackward) {
                    maxVoltageBackward = voltageOutput
                    maxVoltageEncoderBackward = ArmSubsystem.rawEncoder
                    println("New Backward Kg of $maxVoltageBackward at $maxVoltageEncoderBackward")
                }
            }
        }
        ArmSubsystem.setPercentOutput(voltageOutput / 12.0, false)
    }

    override suspend fun dispose() {
        ArmSubsystem.zeroOutputs()
        println(
            "Found Kg of ${(maxVoltageForward + -maxVoltageBackward) / 2.0} ($maxVoltageForward,$maxVoltageBackward) " +
                "at $maxVoltageEncoderForward,$maxVoltageEncoderBackward native units\n" +
                "Found 90 degrees to be ${(maxVoltageEncoderForward + maxVoltageEncoderBackward) / 2.0} native units"
        )
    }

    companion object {
        private const val voltageStepUp = 0.05 / 50.0
        private const val voltageStepDown = 0.75 / 50.0
    }

}