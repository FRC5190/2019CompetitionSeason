package org.ghrobotics.frc2019.subsystems.intake

import com.ctre.phoenix.motorcontrol.NeutralMode
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.utils.and
import org.ghrobotics.lib.utils.greaterThan
import org.ghrobotics.lib.wrappers.ctre.NativeFalconSRX

object IntakeSubsystem : FalconSubsystem() {
    private val intakeMaster = NativeFalconSRX(Constants.kIntakeLeftId)

    val extensionSolenoid = Solenoid(Constants.kPCMId, Constants.kIntakeExtensionSolenoidId)
    val launcherSolenoid = Solenoid(Constants.kPCMId, Constants.kIntakeLauncherSolenoidId)

    val isHoldingCargo = AnalogInput(Constants.kLeftBallSensorId)::getAverageVoltage.greaterThan(0.9) and
        AnalogInput(Constants.kRightBallSensorId)::getAverageVoltage.greaterThan(0.9)

    val isFullyExtended = DigitalInput(Constants.kIntakeExtensionLimitSwitch)::get

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
