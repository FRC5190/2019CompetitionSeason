package org.ghrobotics.frc2019.subsystems.intake

import com.ctre.phoenix.motorcontrol.NeutralMode
import edu.wpi.first.wpilibj.*
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.utils.and
import org.ghrobotics.lib.utils.greaterThan
import org.ghrobotics.lib.utils.not
import org.ghrobotics.lib.wrappers.ctre.NativeFalconSRX

object IntakeSubsystem : FalconSubsystem() {
    val intakeMaster = NativeFalconSRX(Constants.kIntakeLeftId)

    val extensionSolenoid = DoubleSolenoid(
        Constants.kPCMId,
        Constants.kIntakeExtensionSolenoidForwardId,
        Constants.kIntakeExtensionSolenoidReverseId
    )
    val launcherSolenoid = Solenoid(Constants.kPCMId, Constants.kIntakeLauncherSolenoidId)

    private val sensor1 = AnalogInput(Constants.kLeftBallSensorId)
    private val sensor2 = AnalogInput(Constants.kRightBallSensorId)

    private val dio = DigitalOutput(Constants.kIntakeExtensionLimitSwitch)

    val isSeeingCargo = sensor2::getAverageVoltage.greaterThan(1.7)

    val isFullyExtended = dio::get

    val isHoldingCargo = { extensionSolenoid.get() == DoubleSolenoid.Value.kReverse } and isSeeingCargo
    val isHoldingHatch = { extensionSolenoid.get() == DoubleSolenoid.Value.kForward } and !isFullyExtended

    var percentOutput
        get() = intakeMaster.percentOutput
        set(value) {
            intakeMaster.percentOutput = value
        }


    init {
        val intakeSlave = NativeFalconSRX(Constants.kIntakeRightId)

        intakeSlave.follow(intakeMaster)

        intakeMaster.inverted = true
        intakeSlave.inverted = false

        listOf(intakeMaster, intakeSlave).forEach { motor ->
            motor.voltageCompensationSaturation = 12.volt
            motor.voltageCompensationEnabled = true

            motor.continuousCurrentLimit = 18.amp
            motor.currentLimitingEnabled = true

            motor.brakeMode = NeutralMode.Coast
        }
    }

    override fun zeroOutputs() {
        percentOutput = 0.0
    }

    enum class Direction { HOLD, RELEASE }
}
