package org.ghrobotics.frc2019.subsystems.arm

import com.ctre.phoenix.motorcontrol.*
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.frc2019.subsystems.elevator.ElevatorSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.nativeunits.toNativeUnitPosition
import org.ghrobotics.lib.wrappers.ctre.FalconSRX

/**
 * Represents the arm of the robot.
 */
object ArmSubsystem : FalconSubsystem(), EmergencyHandleable {

    // One arm motors rotates the arm
    private val armMaster = FalconSRX(Constants.kArmId, Constants.kArmNativeUnitModel)

    // Store the state so we can dynamically update the feedforward
    private val closedLoopSync = Any()
    private var isClosedLoop = false
    private var closedLoopGoal = Rotation2d(0.0)

    // Used to retrieve the current arm position and to set the arm elevator position.
    var _position = armMaster.sensorPosition
        private set

    // Velocity of the arm.
    var velocity = armMaster.sensorVelocity
        private set

    // Current draw per motor.
    val current get() = armMaster.outputCurrent

    // Raw encoder value.
    val rawEncoder get() = armMaster.getSelectedSensorPosition(0)

    // Voltage draw per motor.
    var voltage = 0.0
        private set

    var arbitraryFeedForward = 0.0
        private set

    init {
        // Configure startup settings
        armMaster.run {
            // Configure feedback sensor and sensor phase
            feedbackSensor = FeedbackDevice.Analog
            encoderPhase = true

            // Brake mode
            brakeMode = NeutralMode.Brake

            // Voltage compensation
            voltageCompensationSaturation = 12.volt
            voltageCompensationEnabled = true

            // Current limiting
            peakCurrentLimit = 0.amp
            peakCurrentLimitDuration = 0.millisecond
            continuousCurrentLimit = Constants.kArmCurrentLimit
            currentLimitingEnabled = true

            // Motion magic
            motionCruiseVelocity = Constants.kArmCruiseVelocity
            motionAcceleration = Constants.kArmAcceleration

            // Analog encoder hackery
            configFeedbackNotContinuous(true, Constants.kCTRETimeout)

            softLimitForward = 220.degree.toNativeUnitPosition(Constants.kArmNativeUnitModel)
            softLimitForwardEnabled = false

            softLimitReverse = (-40).degree.toNativeUnitPosition(Constants.kArmNativeUnitModel)
            softLimitReverseEnabled = false

            setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10)

            kF = Constants.kArmKf
        }
        defaultCommand = object : FalconCommand(this@ArmSubsystem) {
            override suspend fun initialize() {
                synchronized(closedLoopSync) {
                    val lockedPosition = when {
                        isClosedLoop && (_position - closedLoopGoal).absoluteValue < Constants.kArmClosedLoopTolerance -> closedLoopGoal
                        armMaster.controlMode == ControlMode.MotionMagic -> armMaster.activeTrajectoryPosition
                        else -> _position
                    }
                    setPosition(lockedPosition)
                }
            }
        }
        setClosedLoopGains()
    }

    fun setPosition(newPosition: Rotation2d) =
        synchronized(closedLoopSync) {
            isClosedLoop = true
            closedLoopGoal = newPosition
        }

    fun setPercentOutput(newOutput: Double, applyFeedForward: Boolean = true) =
        synchronized(closedLoopSync) {
            isClosedLoop = false
            if (applyFeedForward) {
                armMaster.set(
                    ControlMode.PercentOutput,
                    newOutput,
                    DemandType.ArbitraryFeedForward,
                    arbitraryFeedForward
                )
            } else {
                armMaster.set(ControlMode.PercentOutput, newOutput)
            }
        }

    /**
     * Configures closed loop gains for the arm.
     */
    private fun setClosedLoopGains() {
        armMaster.run {
            kP = Constants.kArmKp
            kD = Constants.kArmKd
        }
    }

    /**
     * Zeros all feedback gains for the arm.
     */
    private fun zeroClosedLoopGains() {
        armMaster.run {
            kP = 0.0
            kD = 0.0
        }
    }

    /**
     * Runs periodically.
     * Used to calculate the acceleration of the arm.
     */
    override fun periodic() {
        this.voltage = armMaster.motorOutputPercent * 12.0

        this._position = armMaster.sensorPosition
        this.velocity = armMaster.sensorVelocity

        arbitraryFeedForward = if (!Robot.emergencyActive) {
            val experiencedAcceleration = Constants.kAccelerationDueToGravity +
                ElevatorSubsystem.acceleration.value

            val Kg = if (IntakeSubsystem.isHoldingHatch()) {
                Constants.kArmHatchKg
            } else Constants.kArmEmptyKg

            Kg * _position.cos * experiencedAcceleration
        } else {
           0.0
        }

        synchronized(closedLoopSync) {
            if (isClosedLoop) {
                armMaster.set(
                    ControlMode.MotionMagic, closedLoopGoal,
                    DemandType.ArbitraryFeedForward, arbitraryFeedForward
                )
            }
        }
    }

    override fun zeroOutputs() {
        setPercentOutput(0.0)
    }

    // Emergency Management
    override fun activateEmergency() {
        zeroOutputs()
        zeroClosedLoopGains()
    }

    override fun recoverFromEmergency() = setClosedLoopGains()
}
