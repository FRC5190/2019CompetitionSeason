package org.ghrobotics.frc2019.subsystems.arm

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.frc2019.subsystems.elevator.ElevatorSubsystem
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.nativeunits.STUPer100ms
import org.ghrobotics.lib.mathematics.units.nativeunits.fromModel
import org.ghrobotics.lib.mathematics.units.radian
import org.ghrobotics.lib.wrappers.ctre.FalconSRX

class ArmSubsystem : FalconSubsystem(), EmergencyHandleable {

    private val armMaster = FalconSRX(Constants.kArmId, Constants.kArmNativeUnitModel)

    var armPosition
        get() = armMaster.sensorPosition
        set(value) {
            var effectiveValue = value.fromModel(Constants.kArmNativeUnitModel).value
            if (effectiveValue < 0) effectiveValue += Constants.kArmSensorUnitsPerRotation.value

            val experiencedAcceleration = 9.81 + ElevatorSubsystem.acceleration.value

            val feedforward =
                Constants.kArmKg * armPosition.cos * experiencedAcceleration + Constants.kArmKa * acceleration.value

            armMaster.set(
                ControlMode.MotionMagic, effectiveValue,
                DemandType.ArbitraryFeedForward, feedforward
            )
        }

    private var previousTrajectoryVelocity = 0.0
    private var acceleration = 0.radian.acceleration

    init {
        armMaster.feedbackSensor = FeedbackDevice.Analog
        armMaster.encoderPhase = false

        armMaster.run {
            brakeMode = NeutralMode.Brake

            voltageCompensationSaturation = 12.volt
            voltageCompensationEnabled = true

            peakCurrentLimit = 0.amp
            peakCurrentLimitDuration = 0.millisecond
            continuousCurrentLimit = Constants.kArmCurrentLimit
            currentLimitingEnabled = true
        }
        setClosedLoopGains()
    }

    private fun setClosedLoopGains() {
        // Uncomment when phases are tested.
        /*
        armMaster.run {
            kP = Constants.kArmKp
        }
        */
    }

    private fun zeroClosedLoopGains() {
        armMaster.run {
            kP = 0.0
        }
    }

    override fun periodic() {
        val cruiseVelocity =
            Constants.kArmCruiseVelocity.fromModel(Constants.kArmNativeUnitModel).STUPer100ms

        val currentVelocity =
            armMaster.activeTrajectoryVelocity.toDouble()

        acceleration = when {
            currentVelocity epsilonEquals cruiseVelocity -> 0.radian.acceleration
            currentVelocity > previousTrajectoryVelocity -> Constants.kArmAcceleration
            else -> -Constants.kArmAcceleration
        }

        previousTrajectoryVelocity = currentVelocity
    }

    override fun activateEmergency() = zeroClosedLoopGains()
    override fun recoverFromEmergency() = setClosedLoopGains()
}
