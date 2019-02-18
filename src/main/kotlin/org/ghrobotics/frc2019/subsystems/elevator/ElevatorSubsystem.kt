package org.ghrobotics.frc2019.subsystems.elevator

import com.ctre.phoenix.motorcontrol.*
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.nativeunits.STU
import org.ghrobotics.lib.utils.DeltaTime
import org.ghrobotics.lib.wrappers.ctre.AbstractFalconSRX
import org.ghrobotics.lib.wrappers.ctre.LinearFalconSRX
import org.ghrobotics.lib.wrappers.ctre.NativeFalconSRX
import kotlin.concurrent.fixedRateTimer

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

    // Store the state so we can dynamically update the feedforward
    private val closedLoopSync = Any()
    private var isClosedLoop = false
    private var closedLoopGoal = Length(0.0)

    // Used to retrieve the current elevator position and to set the desired elevator position.
    var elevatorPosition
        get() = elevatorMaster.sensorPosition
        set(value) = synchronized(closedLoopSync) {
            closedLoopGoal = value
            isClosedLoop = true
        }

    // Current draw per motor.
    val current get() = elevatorMaster.outputCurrent

    // Raw encoder value.
    val rawEncoder get() = elevatorMaster.getSelectedSensorPosition(0)

    // Voltage draw per motor.
    val voltage get() = elevatorMaster.motorOutputPercent * 12.0

    // Velocity of the elevator.
    val velocity get() = elevatorMaster.sensorVelocity

    // Checks if the limit switch is engaged
    val isBottomLimitSwitchPressed get() = elevatorMaster.sensorCollection.isRevLimitSwitchClosed

    var isZeroed = false
        private set

    // Used to retrieve the percent output of each motor and to set the desired percent output.
    var percentOutput
        get() = elevatorMaster.percentOutput
        set(value) = synchronized(closedLoopSync) {
            isClosedLoop = false
            elevatorMaster.percentOutput = value
        }

    // Acceleration of the elevator.
    var actualAcceleration = 0.inch.acceleration
        private set

    init {
        val elevatorSlave1 = NativeFalconSRX(Constants.kElevatorSlave1Id)
        val elevatorSlave2 = NativeFalconSRX(Constants.kElevatorSlave2Id)
        val elevatorSlave3 = NativeFalconSRX(Constants.kElevatorSlave3Id)

        allMotors = listOf(elevatorMaster, elevatorSlave1, elevatorSlave2, elevatorSlave3)

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
            motor.clearPositionOnReverseLimitSwitch = true

            motor.softLimitForward = 10850.STU
            motor.softLimitForwardEnabled = true

            // Motion magic
            motor.motionCruiseVelocity = Constants.kElevatorCruiseVelocity
            motor.motionAcceleration = Constants.kElevatorAcceleration

            motor.kF = Constants.kElevatorKf

            motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000 / 40)
            motor.configMotionSCurveStrength(3)
        }

        // Default command to hold the current position
        defaultCommand = object : FalconCommand(this@ElevatorSubsystem) {
            override suspend fun initialize() {
                ElevatorSubsystem.elevatorPosition = elevatorPosition
            }
        }

        // Set closed loop gains
        setClosedLoopGains()

        var previousVelocity = 0.meter.velocity
        val deltaTime = DeltaTime()
        fixedRateTimer(period = 1000 / 20) {
            val dt = deltaTime.updateTime(System.currentTimeMillis().millisecond)
            val newVelocity = elevatorMaster.sensorVelocity
            if (dt.value > 0) {
                actualAcceleration = (newVelocity - previousVelocity) / dt
            }
            previousVelocity = newVelocity
        }
    }

    /**
     * Configures closed loop gains for the elevator.
     */
    private fun setClosedLoopGains() {
        allMotors.forEach { motor ->
            motor.kP = Constants.kElevatorKp
            motor.kD = Constants.kElevatorKd
        }
    }

    /**
     * Zeros all feedback gains for the elevator.
     */
    private fun zeroClosedLoopGains() {
        allMotors.forEach { motor ->
            motor.kP = 0.0
            motor.kD = 0.0
        }
    }

    override fun zeroOutputs() {
        percentOutput = 0.0
    }

    /**
     * Runs periodically.
     * Used to calculate the actualAcceleration of the elevator.
     */
    override fun periodic() {
        if(isBottomLimitSwitchPressed) {
            isZeroed = true
        }

        synchronized(closedLoopSync) {
            if (isClosedLoop) {
                val feedforward =
                    if (elevatorPosition < Constants.kElevatorSwitchHeight) {
                        Constants.kElevatorBelowSwitchKg
                    } else {
                        Constants.kElevatorAfterSwitchKg
                    }
                elevatorMaster.set(
                    ControlMode.MotionMagic, closedLoopGoal,
                    DemandType.ArbitraryFeedForward, feedforward
                )
            }
        }
    }

    // Emergency Management
    override fun activateEmergency() = zeroClosedLoopGains()

    override fun recoverFromEmergency() = setClosedLoopGains()
}