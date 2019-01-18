package org.ghrobotics.frc2019.subsystems.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.wrappers.ctre.NativeFalconSRX

object IntakeSubsystem : FalconSubsystem() {
    private val intakeMaster = NativeFalconSRX(Constants.kIntakeLeftId)
    private val intakeSlave = NativeFalconSRX(Constants.kIntakeRightId)

    val solenoid = Solenoid(Constants.kPCMId, Constants.kIntakeSolenoidId)

    init {

        intakeMaster.inverted = true

        intakeSlave.follow(intakeMaster)
        intakeSlave.inverted = false

        intakeMaster.run {
            voltageCompensationSaturation = 12.volt
            voltageCompensationEnabled = true

            continuousCurrentLimit = 18.amp
            currentLimitingEnabled = true

            brakeMode = NeutralMode.Coast
        }
    }

    fun set(controlMode: ControlMode, output: Double) = intakeMaster.set(controlMode, output)

    override fun zeroOutputs() {
        set(ControlMode.PercentOutput, 0.0)
    }

    enum class Direction { IN, OUT }
}
