package org.ghrobotics.frc2019.subsystems.arm

import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitModel
import kotlin.math.withSign

class ArmNativeUnitModel(
    armOffsetNativeUnits: NativeUnit,
    armOffsetAngle: Rotation2d,
    sensorResolution: NativeUnit
) : NativeUnitModel<Rotation2d>(Rotation2d(0.0)) {

    private val armOffsetNativeUnits = armOffsetNativeUnits.value
    private val armOffsetAngle = armOffsetAngle.radian
    private val sensorResolution = sensorResolution.value

    override fun fromNativeUnitPosition(nativeUnits: Double): Double {
        val valueFromOffset = boundNativeUnit(nativeUnits - armOffsetNativeUnits)
        val angleFromOffset = 2.0 * Math.PI * (valueFromOffset / sensorResolution)
        return angleFromOffset + armOffsetAngle
    }

    override fun toNativeUnitPosition(modelledUnit: Double): Double {
        val valueFromOffset = modelledUnit - armOffsetAngle
        val nativeUnitsFromOffset = (valueFromOffset / (2.0 * Math.PI)) * sensorResolution
        return boundNativeUnit(nativeUnitsFromOffset + armOffsetNativeUnits)
    }

    private fun boundNativeUnit(nativeUnit: Double): Double {
        var boundNativeUnit = nativeUnit
        while(boundNativeUnit < 0) boundNativeUnit += sensorResolution
        while(boundNativeUnit > sensorResolution) boundNativeUnit -= sensorResolution
        return boundNativeUnit.withSign(armOffsetNativeUnits)
    }

    override fun toNativeUnitError(modelledUnit: Double): Double =
        (modelledUnit / (Math.PI * 2.0)) * sensorResolution

    override fun fromNativeUnitVelocity(nativeUnitVelocity: Double): Double =
        (nativeUnitVelocity / sensorResolution) * (Math.PI * 2.0)

    override fun toNativeUnitVelocity(modelledUnitVelocity: Double): Double =
        (modelledUnitVelocity / (Math.PI * 2.0)) * sensorResolution

}