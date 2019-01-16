package org.ghrobotics.frc2019.subsystems.elevator

import com.ctre.phoenix.motorcontrol.*
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.wrappers.ctre.FalconSRX

object ElevatorSubsystem : FalconSubsystem(), EmergencyHandleable {

    private val elevatorMaster = FalconSRX(Constants.kElevatorMasterId, Constants.kElevatorNativeUnitModel)
    private val elevatorSlave1 = FalconSRX(Constants.kElevatorSlave1Id, Constants.kElevatorNativeUnitModel)
    private val elevatorSlave2 = FalconSRX(Constants.kElevatorSlave2Id, Constants.kElevatorNativeUnitModel)
    private val elevatorSlave3 = FalconSRX(Constants.kElevatorSlave3Id, Constants.kElevatorNativeUnitModel)

    private val allMotors = listOf(elevatorMaster, elevatorSlave1, elevatorSlave2, elevatorSlave3)

    var elevatorPosition
        get() = elevatorMaster.sensorPosition
        set(value) {
            elevatorMaster.set(
                ControlMode.MotionMagic, value,
                DemandType.ArbitraryFeedForward, Constants.kElevatorKg
            )
        }

    var percentOutput
        get() = elevatorMaster.percentOutput
        set(value) {
            elevatorMaster.percentOutput = value
        }

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
            motor.continuousCurrentLimit = Constants.kElevatorCurrentLimit
            motor.currentLimitingEnabled = true

            motor.configForwardLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen
            )
            motor.configReverseLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen
            )

            motor.overrideLimitSwitchesEnable = true
        }

        defaultCommand = object : FalconCommand(this@ElevatorSubsystem) {
            override suspend fun initialize() {
                ElevatorSubsystem.elevatorPosition = elevatorPosition
            }
        }

        setClosedLoopGains()
        createEmergencyReadySystem()
    }

    private fun setClosedLoopGains() {
        // Uncomment when phases are tested.
        /*
        allMotors.forEach { motor ->
            motor.kP = Constants.kElevatorKp
        }
        */
    }

    private fun zeroClosedLoopGains() {
        allMotors.forEach { motor ->
            motor.kP = 0.0
        }
    }

    override fun periodic() {
        if (elevatorMaster.sensorCollection.isRevLimitSwitchClosed) elevatorMaster.sensorPosition = 0.inch
    }

    override fun activateEmergency() = zeroClosedLoopGains()
    override fun recoverFromEmergency() = setClosedLoopGains()
}
