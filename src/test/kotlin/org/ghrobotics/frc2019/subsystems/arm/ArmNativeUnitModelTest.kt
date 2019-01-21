package org.ghrobotics.frc2019.subsystems.arm

import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.nativeunits.STU
import org.junit.Test

class ArmNativeUnitModelTest {

    private val model1 = ArmNativeUnitModel(
        armOffsetNativeUnits = 512.STU,
        armOffsetAngle = 90.degree,
        sensorUnitsPerRotation = 1024.STU
    )

    private val model2 = ArmNativeUnitModel(
        armOffsetNativeUnits = 500.STU,
        armOffsetAngle = 90.degree,
        sensorUnitsPerRotation = 1024.STU
    )

    @Test
    fun testToNativeUnit1() {
        val one = 0.degree
        val two = 90.degree
        val three = 180.degree

        val oneNativeUnit = model1.toNativeUnit(one)
        val twoNativeUnit = model1.toNativeUnit(two)
        val threeNativeUnit = model1.toNativeUnit(three)

        assert(oneNativeUnit.value epsilonEquals 256.0)
        assert(twoNativeUnit.value epsilonEquals 512.0)
        assert(threeNativeUnit.value epsilonEquals 768.0)
    }

    @Test
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
    fun testToNativeUnit2() {
        val one = 0.degree
        val two = 90.degree
        val three = 180.degree

        val oneNativeUnit = model2.toNativeUnit(one)
        val twoNativeUnit = model2.toNativeUnit(two)
        val threeNativeUnit = model2.toNativeUnit(three)

        assert(oneNativeUnit.value epsilonEquals 244.0)
        assert(twoNativeUnit.value epsilonEquals 500.0)
        assert(threeNativeUnit.value epsilonEquals 756.0)
    }

    @Test
    fun testFromNativeUnit1() {
        val one = 256.STU
        val two = 512.STU
        val three = 768.STU

        val oneAngle = model1.fromNativeUnit(one)
        val twoAngle = model1.fromNativeUnit(two)
        val threeAngle = model1.fromNativeUnit(three)

        assert(oneAngle.degree epsilonEquals 0.0)
        assert(twoAngle.degree epsilonEquals 90.0)
        assert(threeAngle.degree epsilonEquals 180.0)
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
    fun testFromNativeUnit2() {
        val one = 244.STU
        val two = 500.STU
        val three = 756.STU

        val oneAngle = model2.fromNativeUnit(one)
        val twoAngle = model2.fromNativeUnit(two)
        val threeAngle = model2.fromNativeUnit(three)

        assert(oneAngle.degree epsilonEquals 0.0)
        assert(twoAngle.degree epsilonEquals 90.0)
        assert(threeAngle.degree epsilonEquals 180.0)
    }

}