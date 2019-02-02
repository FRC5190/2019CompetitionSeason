package org.ghrobotics.frc2019.subsystems.arm

import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.STU
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits
import org.junit.Test

class ArmNativeUnitModelTest {

    private val model1 = ArmNativeUnitModel(
        armSampleNativeUnit = 512.nativeUnits,
        armSampleAngle = 90.degree,
        sensorResolution = 1024.nativeUnits,
        invertPhase = false
    )

    private val model2 = ArmNativeUnitModel(
        armSampleNativeUnit = (-512).nativeUnits,
        armSampleAngle = 90.degree,
        sensorResolution = 1024.nativeUnits,
        invertPhase = true
    )

    private val model3 = ArmNativeUnitModel(
        armSampleNativeUnit = (-500).nativeUnits,
        armSampleAngle = 90.degree,
        sensorResolution = 1024.nativeUnits,
        invertPhase = true
    )

    private val model4 = ArmNativeUnitModel(
        armSampleNativeUnit = 500.nativeUnits,
        armSampleAngle = 90.degree,
        sensorResolution = 1024.nativeUnits,
        invertPhase = false
    )

    /*@Test
    fun testToNativeUnitVelocity1() {
        val one = 0.degree.velocity
        val two = 90.degree.velocity
        val three = 180.degree.velocity

        val oneNativeUnit = model1.toNativeUnitVelocity(one)
        val twoNativeUnit = model1.toNativeUnitVelocity(two)
        val threeNativeUnit = model1.toNativeUnitVelocity(three)

        assert(oneNativeUnit.value epsilonEquals 0.0)
        assert(twoNativeUnit.value epsilonEquals 256.0)
        assert(threeNativeUnit.value epsilonEquals 512.0)
    }

    @Test
    fun testToNativeUnit() {
        fun test(model: ArmNativeUnitModel, angle: Rotation2d, expected: NativeUnit) {
            val actual = model.toNativeUnitPosition(angle)
            println("ARM MODEL: ${angle.degree} deg -> ${actual.value} native units")
            assert(actual epsilonEquals expected)
        }

        println("MODEL 1")
        test(model1, (-180).degree, 768.nativeUnits)
        test(model1, (-90).degree, 0.nativeUnits)
        test(model1, (0).degree, 256.nativeUnits)
        test(model1, (90).degree, 512.nativeUnits)
        test(model1, (180).degree, 768.nativeUnits)

        println("MODEL 2")
        test(model2, (-180).degree, (-256).nativeUnits)
        test(model2, (-90).degree, 0.nativeUnits)
        test(model2, (0).degree, (-768).nativeUnits)
        test(model2, (90).degree, (-512).nativeUnits)
        test(model2, (180).degree, (-256).nativeUnits)

        println("MODEL 3")
        test(model3, (-180).degree, (-268).nativeUnits)
        test(model3, (-90).degree, (-12).nativeUnits)
        test(model3, (0).degree, (-780).nativeUnits)
        test(model3, (90).degree, (-524).nativeUnits)
        test(model3, (180).degree, (-268).nativeUnits)

        println("MODEL 4")
        test(model4, (-180).degree, 756.nativeUnits)
        test(model4, (-90).degree, 1012.nativeUnits)
        test(model4, (0).degree, 244.nativeUnits)
        test(model4, (90).degree, 500.nativeUnits)
        test(model4, (180).degree, 756.nativeUnits)
    }

    @Test
    fun testFromNativeUnitVelocity1() {
        val one = 256.STU.velocity
        val two = 512.STU.velocity
        val three = 768.STU.velocity

        val oneAngle = model1.fromNativeUnitVelocity(one)
        val twoAngle = model1.fromNativeUnitVelocity(two)
        val threeAngle = model1.fromNativeUnitVelocity(three)

        assert(oneAngle.value epsilonEquals Math.PI * 0.5)
        assert(twoAngle.value epsilonEquals Math.PI)
        assert(threeAngle.value epsilonEquals Math.PI * 1.5)
    }

    @Test
    fun testFromNativeUnit() {
        fun test(model: ArmNativeUnitModel, nativeUnits: NativeUnit, expected: Rotation2d) {
            val actual = model.fromNativeUnitPosition(nativeUnits)
            println("ARM MODEL: ${nativeUnits.value} native units -> ${actual.degree} deg")
            assert(actual epsilonEquals expected)
        }

        println("MODEL 1")
        test(model1, 0.nativeUnits, (-90).degree)
        test(model1, 256.nativeUnits, 0.degree)
        test(model1, 512.nativeUnits, 90.degree)
        test(model1, 768.nativeUnits, 180.degree)
        test(model1, 1024.nativeUnits, 270.degree)

        println("MODEL 2")
        test(model2, 0.nativeUnits, (-90).degree)
        test(model2, (-256).nativeUnits, (-180).degree)
        test(model2, (-512).nativeUnits, 90.degree)
        test(model2, (-768).nativeUnits, 0.degree)
        test(model2, (-1024).nativeUnits, 270.degree)

        println("MODEL 3")
        test(model3, (-12).nativeUnits, (-90).degree)
        test(model3, (-268).nativeUnits, (-180).degree)
        test(model3, (-524).nativeUnits, 90.degree)
        test(model3, (-780).nativeUnits, 0.degree)
        test(model3, (-1036).nativeUnits, 270.degree)

        println("MODEL 4")
        test(model4, (-12).nativeUnits, (-90).degree)
        test(model4, 244.nativeUnits, 0.degree)
        test(model4, 500.nativeUnits, 90.degree)
        test(model4, 756.nativeUnits, 180.degree)
        test(model4, 1012.nativeUnits, 270.degree)
    }*/

}