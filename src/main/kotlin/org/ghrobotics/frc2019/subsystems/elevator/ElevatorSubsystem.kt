package org.ghrobotics.frc2019.subsystems.elevator

import com.ctre.phoenix.motorcontrol.*
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.nativeunits.STU
import org.ghrobotics.lib.wrappers.ctre.NativeFalconSRX

/**
 * Represents the elevator of the robot.
 */
object ElevatorSubsystem : FalconSubsystem(), EmergencyHandleable {

    // 4 motors that move the elevator up and down.
    private val elevatorMaster =
        SpringCascadingFalconSRX(Constants.kElevatorMasterId, Constants.kElevatorNativeUnitModel)
    private val elevatorSlave1 = NativeFalconSRX(Constants.kElevatorSlave1Id)
    private val elevatorSlave2 = NativeFalconSRX(Constants.kElevatorSlave2Id)
    private val elevatorSlave3 = NativeFalconSRX(Constants.kElevatorSlave3Id)

    // List of all motors.
    private val allMotors = listOf(elevatorMaster, elevatorSlave1, elevatorSlave2, elevatorSlave3)

    // Used to retrieve the current elevator position and to set the desired elevator position.
    var elevatorPosition
        get() = elevatorMaster.sensorPosition
        set(value) {
            elevatorMaster.set(
                ControlMode.MotionMagic, value,
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

    // Checks if the limit switch is engaged
    val limitSwitch
        get() = elevatorMaster.sensorCollection.isRevLimitSwitchClosed

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
    private var previousTrajectoryVelocity = 0.meter.velocity

    init {
        // Set slaves to follow master
        elevatorSlave1.follow(elevatorMaster)
        elevatorSlave2.follow(elevatorMaster)
        elevatorSlave3.follow(elevatorMaster)

        // Configure feedback sensor and sensor phase
        elevatorMaster.feedbackSensor = FeedbackDevice.QuadEncoder
        elevatorMaster.encoderPhase = false

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
        }

        // Configure startup settings
        elevatorMaster.also { motor ->
            // Limit switches
            motor.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector to LimitSwitchNormal.NormallyOpen
            motor.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector to LimitSwitchNormal.NormallyOpen
            motor.overrideLimitSwitchesEnable = true

            // Clear position when at bottom
//            motor.clearPositionOnReverseLimitSwitch = true

            motor.softLimitForward = 10850.STU
            motor.softLimitForwardEnabled = true

            // Motion magic
            motor.motionCruiseVelocity = Constants.kElevatorCruiseVelocity
            motor.motionAcceleration = Constants.kElevatorAcceleration

            motor.kF = Constants.kElevatorKf
        }

        // Default command to hold the current position
        defaultCommand = object : FalconCommand(this@ElevatorSubsystem) {
            override suspend fun initialize() {
                ElevatorSubsystem.elevatorPosition = elevatorPosition
            }
        }

        // Set closed loop gains
        setClosedLoopGains()
    }

    /**
     * Configures closed loop gains for the elevator.
     */
    private fun setClosedLoopGains() {
        allMotors.forEach { motor ->
            motor.kP = Constants.kElevatorKp
        }
    }

    /**
     * Zeros all feedback gains for the elevator.
     */
    private fun zeroClosedLoopGains() {
        allMotors.forEach { motor ->
            motor.kP = 0.0
        }
    }

    override fun zeroOutputs() {
        elevatorMaster.set(ControlMode.PercentOutput, 0.0)
    }

    /**
     * Runs periodically.
     * Used to calculate the acceleration of the elevator.
     */
    override fun periodic() {

        if (limitSwitch) {
            elevatorMaster.sensorPosition = 0.meter
        }

        val cruiseVelocity = Constants.kElevatorCruiseVelocity

        acceleration = if (elevatorMaster.controlMode == ControlMode.MotionMagic) {
            val currentVelocity = elevatorMaster.activeTrajectoryVelocity
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