package org.ghrobotics.frc2019.subsystems.arm

import com.ctre.phoenix.motorcontrol.*
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.frc2019.subsystems.elevator.ElevatorSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.nativeunits.toNativeUnitPosition
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.wrappers.ctre.FalconSRX

/**
 * Represents the arm of the robot.
 */
object ArmSubsystem : FalconSubsystem(), EmergencyHandleable {

    // One arm motors rotates the arm
    private val armMaster = FalconSRX(Constants.kArmId, Constants.kArmNativeUnitModel)

    var wantedState: ArmState = ArmState.Nothing
    var currentState: ArmState = ArmState.Nothing
        private set

    // PERIODIC
    var position = 0.degree
        private set
    var velocity = 0.degree.velocity
        private set
    var arbitraryFeedForward = 0.0
        private set

    // DEBUG PERIODIC
    var voltage = 0.0
        private set
    var current = 0.0
        private set
    var rawSensorPosition = 0
        private set

    init {
        // Configure startup settings
        with(armMaster) {
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

            selectProfileSlot(0, 0)
        }
        defaultCommand = DefaultArmCommand

        setClosedLoopGains()
    }


    /**
     * Configures closed loop gains for the arm.
     */
    private fun setClosedLoopGains() {
        armMaster.run {
            config_kP(0, Constants.kArmKp)
            config_kD(0, Constants.kArmKd)
            config_kF(0, Constants.kArmKf)
        }
    }

    /**
     * Zeros all feedback gains for the arm.
     */
    private fun zeroClosedLoopGains() {
        armMaster.run {
            config_kP(0, 0.0)
            config_kD(0, 0.0)
        }
    }

    /**
     * Runs periodically.
     * Used to calculate the acceleration of the arm.
     */
    override fun periodic() {
        // PERIODIC
        position = armMaster.sensorPosition
        velocity = armMaster.sensorVelocity

        arbitraryFeedForward = if (!Robot.emergencyActive) {
            val experiencedAcceleration = Constants.kAccelerationDueToGravity + ElevatorSubsystem.acceleration

            val Kg = if (IntakeSubsystem.isHoldingHatch) {
                Constants.kArmHatchKg
            } else Constants.kArmEmptyKg

            Kg * position.cos * experiencedAcceleration
        } else {
            0.0
        }

        // DEBUG PERIODIC
        if (Robot.shouldDebug) {
            voltage = armMaster.motorOutputPercent * 12.0
            current = armMaster.outputCurrent
            rawSensorPosition = armMaster.getSelectedSensorPosition(0)
        }

        // UPDATE STATE
        val wantedState = this.wantedState
        this.currentState = wantedState
        when (wantedState) {
            is ArmState.Nothing -> {
                armMaster.set(ControlMode.Disabled, 0.0)
            }
            is ArmState.MotionMagic -> {
                armMaster.set(
                    ControlMode.MotionMagic,
                    Constants.kArmNativeUnitModel.toNativeUnitPosition(wantedState.position.value),
                    DemandType.ArbitraryFeedForward,
                    arbitraryFeedForward
                )
            }
            is ArmState.OpenLoop -> {
                if (wantedState.useFeedForward) {
                    armMaster.set(
                        ControlMode.PercentOutput,
                        wantedState.output(),
                        DemandType.ArbitraryFeedForward,
                        arbitraryFeedForward
                    )
                } else {
                    armMaster.set(ControlMode.PercentOutput, wantedState.output())
                }
            }
        }
    }

    override fun zeroOutputs() {
        wantedState = ArmSubsystem.ArmState.Nothing
    }

    // Emergency Management
    override fun activateEmergency() {
        zeroOutputs()
        zeroClosedLoopGains()
    }

    override fun recoverFromEmergency() = setClosedLoopGains()

    sealed class ArmState {
        object Nothing : ArmState()
        abstract class SetPointState(val position: Rotation2d) : ArmState()
        class MotionMagic(position: Rotation2d) : SetPointState(position)
        class OpenLoop(val output: Source<Double>, val useFeedForward: Boolean) : ArmState()
    }
}
