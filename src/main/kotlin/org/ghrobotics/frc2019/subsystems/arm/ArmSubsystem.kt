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

/**
 * Represents the arm of the robot.
 */
object ArmSubsystem : FalconSubsystem(), EmergencyHandleable {

    // One arm motors rotates the arm
    private val armMaster = FalconSRX(Constants.kArmId, Constants.kArmNativeUnitModel)

    // Used to retrieve the current arm position and to set the arm elevator position.
    var armPosition
        get() = armMaster.sensorPosition
        set(value) {
            var effectiveValue = value.fromModel(Constants.kArmNativeUnitModel).value
            if (effectiveValue < 0) effectiveValue += Constants.kArmSensorUnitsPerRotation.value

            val experiencedAcceleration = 9.81 + ElevatorSubsystem.acceleration.value

            val feedforward =
                Constants.kArmKg * armPosition.cos * experiencedAcceleration + Constants.kArmKa * acceleration.value

            armMaster.set(
                ControlMode.Disabled, effectiveValue,
                DemandType.ArbitraryFeedForward, feedforward
            )
        }

    // Current draw per motor.
    val current
        get() = armMaster.outputCurrent

    // Raw encoder value.
    val rawEncoder
        get() = armMaster.getSelectedSensorPosition(0)

    // Voltage draw per motor.
    val voltage
        get() = armMaster.motorOutputPercent * 12.0

    // Velocity of the arm.
    val velocity
        get() = armMaster.sensorVelocity

    // Used to retrieve the percent output of each motor and to set the desired percent output.
    var percentOutput
        get() = armMaster.percentOutput
        set(value) {
            armMaster.percentOutput = value
        }

    // Acceleration of the elevator.
    private var acceleration = 0.radian.acceleration

    // Used as a storage variable to compute acceleration.
    private var previousTrajectoryVelocity = 0.0

    init {
        // Configure feedback sensor and sensor phase
        armMaster.feedbackSensor = FeedbackDevice.Analog
        armMaster.encoderPhase = false

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
        }
        setClosedLoopGains()
    }

    /**
     * Configures closed loop gains for the arm.
     */
    private fun setClosedLoopGains() {
        // Uncomment when phases are tested.
        /*
        armMaster.run {
            kP = Constants.kArmKp
        }
        */
    }

    /**
     * Zeros all feedback gains for the arm.
     */
    private fun zeroClosedLoopGains() {
        armMaster.run {
            kP = 0.0
        }
    }

    /**
     * Runs periodically.
     * Used to calculate the acceleration of the arm.
     */
    override fun periodic() {
        val cruiseVelocity =
            Constants.kArmCruiseVelocity.fromModel(Constants.kArmNativeUnitModel).STUPer100ms

        acceleration = if (armMaster.controlMode == ControlMode.MotionMagic) {
            val currentVelocity = armMaster.activeTrajectoryVelocity.toDouble()
            when {
                currentVelocity epsilonEquals cruiseVelocity -> 0.radian.acceleration
                currentVelocity > previousTrajectoryVelocity -> Constants.kArmAcceleration
                else -> -Constants.kArmAcceleration
            }.also { previousTrajectoryVelocity = currentVelocity }
        } else {
            0.radian.acceleration
        }
    }

    // Emergency Management
    override fun activateEmergency() = zeroClosedLoopGains()

    override fun recoverFromEmergency() = setClosedLoopGains()
}
