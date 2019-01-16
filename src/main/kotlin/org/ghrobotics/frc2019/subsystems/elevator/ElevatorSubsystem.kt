package org.ghrobotics.frc2019.subsystems.elevator

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.wrappers.ctre.FalconSRX

object ElevatorSubsystem : FalconSubsystem(), EmergencyHandleable {
    private val elevatorMaster = FalconSRX(Constants.kElevatorMasterId, Constants.kElevatorNativeUnitModel)
    private val elevatorSlave1 = FalconSRX(Constants.kElevatorSlave1Id, Constants.kElevatorNativeUnitModel)
    private val elevatorSlave2 = FalconSRX(Constants.kElevatorSlave2Id, Constants.kElevatorNativeUnitModel)
    private val elevatorSlave3 = FalconSRX(Constants.kElevatorSlave3Id, Constants.kElevatorNativeUnitModel)

    private val allMotors = listOf(elevatorMaster, elevatorSlave1, elevatorSlave2, elevatorSlave3)

    init {
        elevatorSlave1.follow(elevatorMaster)
        elevatorSlave2.follow(elevatorMaster)
        elevatorSlave3.follow(elevatorMaster)

        elevatorMaster.feedbackSensor = FeedbackDevice.QuadEncoder
        elevatorMaster.encoderPhase = true

        allMotors.forEach { motor ->
            motor.brakeMode = NeutralMode.Brake

            motor.voltageCompensationSaturation = 12.volt
            motor.voltageCompensationEnabled = true

            motor.peakCurrentLimit = 0.amp
            motor.peakCurrentLimitDuration = 0.millisecond
            motor.continuousCurrentLimit = 15.amp // TODO Find Actual Value
            motor.currentLimitingEnabled = true
        }
        setClosedLoopGains()
        createEmergencyReadySystem()
    }

    private fun setClosedLoopGains() {
        allMotors.forEach { motor ->
            motor.kP = Constants.kElevatorKp
        }
    }

    private fun zeroClosedLoopGains() {
        allMotors.forEach { motor ->
            motor.kP = 0.0
        }
    }

    override fun activateEmergency() = zeroClosedLoopGains()
    override fun recoverFromEmergency() = setClosedLoopGains()
}
