package org.ghrobotics.frc2019.subsystems.arm

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.frc2019.subsystems.elevator.ElevatorSubsystem
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
    var armPosition
        get() = armMaster.sensorPosition
        set(value) = synchronized(closedLoopSync) {
            isClosedLoop = true
            closedLoopGoal = value
        }

    // Current draw per motor.
    val current get() = armMaster.outputCurrent

    // Raw encoder value.
    val rawEncoder get() = armMaster.getSelectedSensorPosition(0)

    // Voltage draw per motor.
    val voltage get() = armMaster.motorOutputPercent * 12.0

    // Velocity of the arm.
    val velocity get() = armMaster.sensorVelocity

    // Used to retrieve the percent output of each motor and to set the desired percent output.
    var percentOutput
        get() = armMaster.percentOutput
        set(value) = synchronized(closedLoopSync) {
            isClosedLoop = false
            armMaster.percentOutput = value
        }

    init {
        // Configure feedback sensor and sensor phase
        armMaster.feedbackSensor = FeedbackDevice.Analog
        armMaster.encoderPhase = true

        // Configure startup settings
        armMaster.run {
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

            softLimitForward = 200.degree.toNativeUnitPosition(Constants.kArmNativeUnitModel)
            softLimitForwardEnabled = true

            softLimitReverse = (-20).degree.toNativeUnitPosition(Constants.kArmNativeUnitModel)
            softLimitReverseEnabled = true

            kF = Constants.kArmKf
        }
        defaultCommand = object : FalconCommand(this@ArmSubsystem) {
            override suspend fun initialize() {
                armPosition = armPosition
            }
        }
        setClosedLoopGains()
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
     * Used to calculate the actualAcceleration of the arm.
     */
    override fun periodic() {
        synchronized(closedLoopSync) {
            if (isClosedLoop) {
                val experiencedAcceleration = Constants.kAccelerationDueToGravity +
                    ElevatorSubsystem.actualAcceleration.value

                val feedforward = Constants.kArmKg * armPosition.cos * experiencedAcceleration

                armMaster.set(
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
