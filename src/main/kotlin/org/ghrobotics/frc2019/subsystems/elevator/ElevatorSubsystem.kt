package org.ghrobotics.frc2019.subsystems.elevator

import com.ctre.phoenix.motorcontrol.*
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.kMainLoopDt
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.wrappers.ctre.AbstractFalconSRX
import org.ghrobotics.lib.wrappers.ctre.LinearFalconSRX
import org.ghrobotics.lib.wrappers.ctre.NativeFalconSRX

/**
 * Represents the elevator of the robot.
 */
object ElevatorSubsystem : FalconSubsystem(), EmergencyHandleable {

    // 4 motors that move the elevator up and down.
    private val elevatorMaster = LinearFalconSRX(
        Constants.kElevatorMasterId,
        Constants.kElevatorNativeUnitModel
    )

    // List of all motors.
    private val allMotors: List<AbstractFalconSRX<*>>

    var wantedState: ElevatorState = ElevatorState.Nothing
    var currentState: ElevatorState = ElevatorState.Nothing
        private set

    var wantedVisionMode = false

    // PERIODIC VALUES
    var position: Double = 0.0
        private set
    var velocity: Double = 0.0
        private set
    var acceleration: Double = 0.0
        private set
    var isBottomLimitSwitchPressed: Boolean = false
        private set
    var isZeroed: Boolean = false
        private set
    var arbitraryFeedForward: Double = 0.0
        private set

    // DEBUG VALUES
    var current: Double = 0.0
        private set
    var rawSensorPosition: Int = 0
        private set
    var voltage: Double = 0.0
        private set

    init {
        val slaveMotors = listOf(
            Constants.kElevatorSlave1Id,
            Constants.kElevatorSlave2Id,
            Constants.kElevatorSlave3Id
        ).map {
            NativeFalconSRX(it).apply {
                // Set slaves to follow master
                follow(elevatorMaster)
            }
        }

        allMotors = slaveMotors + elevatorMaster

        // Configure startup settings
        with(elevatorMaster) {

            // Configure feedback sensor and sensor phase
            feedbackSensor = FeedbackDevice.QuadEncoder
            encoderPhase = false

            // Limit switches
            forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector to LimitSwitchNormal.NormallyOpen
            reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector to LimitSwitchNormal.NormallyOpen
            overrideLimitSwitchesEnable = true

            // Clear position when at bottom
            clearPositionOnReverseLimitSwitch = true

            softLimitForward = 10850.nativeUnits
            softLimitForwardEnabled = true

            // Motion magic
            motionCruiseVelocity = Constants.kElevatorCruiseVelocity
            motionAcceleration = Constants.kElevatorAcceleration

            setStatusFramePeriod(StatusFrame.Status_2_Feedback0, kMainLoopDt.millisecond.toInt())
            configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_20Ms)

            selectProfileSlot(0, 0)
//            configMotionSCurveStrength(3)
        }

        allMotors.forEach { motor ->
            with(motor) {
                // Brake mode
                brakeMode = NeutralMode.Brake

                // Voltage compensation
                voltageCompensationSaturation = 12.volt
                voltageCompensationEnabled = true

                // Current limiting
                peakCurrentLimit = 0.amp
                peakCurrentLimitDuration = 0.millisecond
                continuousCurrentLimit = Constants.kElevatorCurrentLimit
                currentLimitingEnabled = true
            }
        }

        // Default command to hold the current position
        defaultCommand = DefaultElevatorCommand

        // Set closed loop gains
        setClosedLoopGains()
    }

    /**
     * Runs periodically.
     * Used to calculate the acceleration of the elevator.
     */
    override fun periodic() {
        // PERIODIC
        val previousVelocity = velocity

        position = elevatorMaster.sensorPosition.value
        velocity = elevatorMaster.sensorVelocity.value
        acceleration = (velocity - previousVelocity) / kMainLoopDt.value

        isBottomLimitSwitchPressed = elevatorMaster.sensorCollection.isRevLimitSwitchClosed
        if (isBottomLimitSwitchPressed) isZeroed = true

        arbitraryFeedForward =
            if (position >= Constants.kElevatorSwitchHeight.value || Robot.emergencyActive) {
                Constants.kElevatorAfterSwitchKg
            } else {
                Constants.kElevatorBelowSwitchKg
            }

        // DEBUG PERIODIC
        if (Robot.shouldDebug) {
            current = elevatorMaster.outputCurrent
            voltage = elevatorMaster.motorOutputPercent * 12.0
            rawSensorPosition = elevatorMaster.getSelectedSensorPosition(0)
        }

        // UPDATE STATE
        var wantedState = this.wantedState

        if (wantedState is ElevatorState.SetPointState && wantedVisionMode
            && wantedState.position in Constants.kElevatorBlockingCameraRange
        ) {
            wantedState = ElevatorState.MotionMagic(Constants.kElevatorVisionPosition.value)
        }

        this.currentState = wantedState

        when (wantedState) {
            is ElevatorState.Nothing -> {
                elevatorMaster.set(ControlMode.Disabled, 0.0)
            }
            is ElevatorState.MotionMagic -> {
                elevatorMaster.set(
                    ControlMode.Disabled,
                    Constants.kElevatorNativeUnitModel.toNativeUnitPosition(wantedState.position),
                    DemandType.ArbitraryFeedForward,
                    arbitraryFeedForward
                )
            }
            is ElevatorState.OpenLoop -> {
                if (wantedState.useFeedForward) {
                    elevatorMaster.set(
                        ControlMode.Disabled,
                        wantedState.output(),
                        DemandType.ArbitraryFeedForward,
                        arbitraryFeedForward
                    )
                } else {
                    elevatorMaster.set(ControlMode.Disabled, wantedState.output())
                }
            }
        }
    }

    /**
     * Configures closed loop gains for the elevator.
     */
    private fun setClosedLoopGains() {
        allMotors.forEach { motor ->
            motor.config_kP(0, Constants.kElevatorKp)
            motor.config_kD(0, Constants.kElevatorKd)
            motor.config_kF(0, Constants.kElevatorKf)
        }
    }

    /**
     * Zeros all feedback gains for the elevator.
     */
    private fun zeroClosedLoopGains() {
        allMotors.forEach { motor ->
            motor.config_kP(0, 0.0)
            motor.config_kD(0, 0.0)
        }
    }

    override fun zeroOutputs() {
        wantedState = ElevatorState.Nothing
    }

    // Emergency Management
    override fun activateEmergency() {
        zeroOutputs()
        zeroClosedLoopGains()
    }

    override fun recoverFromEmergency() = setClosedLoopGains()

    sealed class ElevatorState {
        object Nothing : ElevatorState()
        abstract class SetPointState(val position: Double) : ElevatorState()
        class MotionMagic(position: Double) : SetPointState(position)
        class OpenLoop(val output: Source<Double>, val useFeedForward: Boolean) : ElevatorState()
    }
}