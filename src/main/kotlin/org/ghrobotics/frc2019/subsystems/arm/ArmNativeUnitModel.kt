package org.ghrobotics.frc2019.subsystems.arm

import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitModel

/**
 * @param invertPhase When you invert a absolute encoder on a TalonSRX this value may become negative
 */
class ArmNativeUnitModel(
    armSampleNativeUnit: NativeUnit,
    armSampleAngle: Rotation2d,
    sensorResolution: NativeUnit,
    private val invertPhase: Boolean
) : NativeUnitModel<Rotation2d>(Rotation2d(0.0)) {

    private val armZero: Double
    private val armMinAngle: Double
    private val armMaxAngle: Double
    private val sensorResolution = sensorResolution.value

    init {
        val nativeUnitOffset = (armSampleAngle.degree / 360.0 * sensorResolution.value)
        armZero = armSampleNativeUnit.value - nativeUnitOffset
        armMinAngle = -armZero / sensorResolution.value * 2.0 * Math.PI
        armMaxAngle = 2.0 * Math.PI + armMinAngle
    }

    override fun fromNativeUnitPosition(nativeUnits: Double): Double {
        var validNativeUnit = nativeUnits
        // flip phase if needed
        validNativeUnit = boundNativeUnit(validNativeUnit)
        val result = boundArmAngle((validNativeUnit - armZero) / sensorResolution * 2.0 * Math.PI)
        // flip phase back if needed
        return if (invertPhase) result - 2 * Math.PI else result
    }


    override fun toNativeUnitPosition(modelledUnit: Double): Double {
        var validAngle = modelledUnit
        // flip phase if needed
        validAngle = boundArmAngle(validAngle)
        val result = boundNativeUnit((validAngle / 2.0 / Math.PI * sensorResolution) + armZero)
        // flip phase back if needed
        return if (invertPhase) result - sensorResolution else result
    }

    private fun boundNativeUnit(nativeUnit: Double): Double {
        var boundNativeUnit = nativeUnit
        while (boundNativeUnit < 0) boundNativeUnit += sensorResolution
        while (boundNativeUnit >= sensorResolution) boundNativeUnit -= sensorResolution
        return boundNativeUnit
    }

    private fun boundArmAngle(angle: Double): Double {
        var boundArmAngle = angle
        if (boundArmAngle > armMaxAngle) boundArmAngle -= 2.0 * Math.PI
        if (boundArmAngle < armMinAngle) boundArmAngle += 2.0 * Math.PI
        return boundArmAngle
    }

    override fun toNativeUnitError(modelledUnit: Double): Double =
        (modelledUnit / (Math.PI * 2.0)) * sensorResolution

    override fun fromNativeUnitVelocity(nativeUnitVelocity: Double): Double =
        (nativeUnitVelocity / sensorResolution) * (Math.PI * 2.0)

    override fun toNativeUnitVelocity(modelledUnitVelocity: Double): Double =
        (modelledUnitVelocity / (Math.PI * 2.0)) * sensorResolution

}