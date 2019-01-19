package org.ghrobotics.frc2019.subsystems.elevator

import com.ctre.phoenix.motorcontrol.*
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.nativeunits.STUPer100ms
import org.ghrobotics.lib.mathematics.units.nativeunits.fromModel
import org.ghrobotics.lib.wrappers.ctre.FalconSRX

/**
 * Represents the elevator of the robot.
 */
object ElevatorSubsystem : FalconSubsystem(), EmergencyHandleable {

    // 4 motors that move the elevator up and down.
    private val elevatorMaster = FalconSRX(Constants.kElevatorMasterId, Constants.kElevatorNativeUnitModel)
    private val elevatorSlave1 = FalconSRX(Constants.kElevatorSlave1Id, Constants.kElevatorNativeUnitModel)
    private val elevatorSlave2 = FalconSRX(Constants.kElevatorSlave2Id, Constants.kElevatorNativeUnitModel)
    private val elevatorSlave3 = FalconSRX(Constants.kElevatorSlave3Id, Constants.kElevatorNativeUnitModel)

    // List of all motors.
    private val allMotors = listOf(elevatorMaster, elevatorSlave1, elevatorSlave2, elevatorSlave3)

    // Used to retrieve the current elevator position and to set the desired elevator position.
    var elevatorPosition
        get() = elevatorMaster.sensorPosition
        set(value) {
            elevatorMaster.set(
                ControlMode.Disabled, value,
                DemandType.ArbitraryFeedForward, Constants.kElevatorKg
            )
        }

    // Current draw per motor.
    val current
        get() = elevatorMaster.outputCurrent

    // Raw encoder value.
    val rawEncoder
        get() = elevatorMaster.getSelectedSensorPosition(0)

    // Voltage draw per motor.
    val voltage
        get() = elevatorMaster.motorOutputPercent * 12.0

    // Velocity of the elevator.
    val velocity
        get() = elevatorMaster.sensorVelocity

    // Used to retrieve the percent output of each motor and to set the desired percent output.
    var percentOutput
        get() = elevatorMaster.percentOutput
        set(value) {
            elevatorMaster.percentOutput = value
        }

    // Acceleration of the elevator.
    var acceleration = 0.inch.acceleration
        private set

    // Used as a storage variable to compute acceleration.
    private var previousTrajectoryVelocity = 0.0

    init {
        // Set slaves to follow master
        elevatorSlave1.follow(elevatorMaster)
        elevatorSlave2.follow(elevatorMaster)
        elevatorSlave3.follow(elevatorMaster)

        // Configure feedback sensor and sensor phase
        elevatorMaster.feedbackSensor = FeedbackDevice.QuadEncoder
        elevatorMaster.encoderPhase = true

        // Configure startup settings
        allMotors.forEach { motor ->
            // Brake mode
            motor.brakeMode = NeutralMode.Brake

            // Voltage compensation
            motor.voltageCompensationSaturation = 12.volt
            motor.voltageCompensationEnabled = true

            // Current limiting
            motor.peakCurrentLimit = 0.amp
            motor.peakCurrentLimitDuration = 0.millisecond
            motor.continuousCurrentLimit = Constants.kElevatorCurrentLimit
            motor.currentLimitingEnabled = true

            // Limit switches
            motor.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector to LimitSwitchNormal.NormallyOpen
            motor.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector to LimitSwitchNormal.NormallyOpen
            motor.overrideLimitSwitchesEnable = true

            // Clear position when at bottom
            motor.clearPositionOnReverseLimitSwitch = true

            // Motion magic
            motor.motionCruiseVelocity = Constants.kElevatorCruiseVelocity
            motor.motionAcceleration = Constants.kElevatorAcceleration
        }

        // Default command to hold the current position
        defaultCommand = object : FalconCommand(this@ElevatorSubsystem) {
            override suspend fun initialize() {
                elevatorPosition = elevatorPosition
            }
        }

        // Set closed loop gains
        setClosedLoopGains()
    }

    /**
     * Configures closed loop gains for the elevator.
     */
    private fun setClosedLoopGains() {
        // Uncomment when phases are tested.
        /*
        allMotors.forEach { motor ->
            motor.kP = Constants.kElevatorKp
        }
        */
    }

    /**
     * Zeros all feedback gains for the elevator.
     */
    private fun zeroClosedLoopGains() {
        allMotors.forEach { motor ->
            motor.kP = 0.0
        }
    }

    /**
     * Runs periodically.
     * Used to calculate the acceleration of the elevator.
     */
    override fun periodic() {
        val cruiseVelocity =
            Constants.kElevatorCruiseVelocity.fromModel(Constants.kElevatorNativeUnitModel).STUPer100ms

        acceleration = if (elevatorMaster.controlMode == ControlMode.MotionMagic) {
            val currentVelocity =
                elevatorMaster.activeTrajectoryVelocity.toDouble()
            when {
                currentVelocity epsilonEquals cruiseVelocity -> 0.meter.acceleration
                currentVelocity > previousTrajectoryVelocity -> Constants.kElevatorAcceleration
                else -> -Constants.kElevatorAcceleration
            }.also { previousTrajectoryVelocity = currentVelocity }
        } else {
            0.meter.acceleration
        }
    }

    // Emergency Management
    override fun activateEmergency() = zeroClosedLoopGains()
    override fun recoverFromEmergency() = setClosedLoopGains()
}
