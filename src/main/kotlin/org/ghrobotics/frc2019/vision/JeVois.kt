@file:Suppress("MatchingDeclarationName")

package org.ghrobotics.frc2019.vision

import com.fazecast.jSerialComm.SerialPort
import com.fazecast.jSerialComm.SerialPortDataListener
import com.fazecast.jSerialComm.SerialPortEvent
import com.github.salomonbrys.kotson.fromJson
import com.google.gson.Gson
import com.google.gson.JsonObject
import com.google.gson.JsonParseException
import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.second

class JeVois(
    private val serialPort: SerialPort,
    private val processData: (VisionData) -> Unit
) {

    private var state = State.NONE
    private var timeOffset = 0.second

    init {
        serialPort.openPort()
        serialPort.addDataListener(object : SerialPortDataListener {
            private var byteBuffer = ByteArray(1024)
            private var bufferIndex = 0

            override fun serialEvent(event: SerialPortEvent) {
                if (event.eventType != SerialPort.LISTENING_EVENT_DATA_AVAILABLE)
                    return
                val newData = ByteArray(serialPort.bytesAvailable())
                serialPort.readBytes(newData, newData.size.toLong())
                for (newByte in newData) {
                    if (newByte.toChar() != '\n') {
                        byteBuffer[bufferIndex++] = newByte
                        continue
                    }
                    onStringReceived(String(byteBuffer, 0, bufferIndex).trim())
                    while (bufferIndex > 0) {
                        byteBuffer[--bufferIndex] = 0
                    }
                }
            }

            override fun getListeningEvents() = SerialPort.LISTENING_EVENT_DATA_AVAILABLE
        })
        state = State.SET_OUTPUT
        serialPort.writeString("setpar serout USB\n")
    }

    private fun onStringReceived(receivedString: String) {
        when (state) {
            State.NONE -> {
                println("[JeVois] [STATE: NONE] Got Data: $receivedString")
            }
            State.SET_OUTPUT -> {
                if (receivedString == "OK") {
                    println("[JeVois] [STATE: SET_OUTPUT] Set Serial Output on JeVois to USB")
                    state = State.SET_DATE
                    serialPort.writeString("date 0101000070\n")
                } else {
                    println("[JeVois] [STATE: SET_OUTPUT] Failed to set Serial Output on JeVois to USB")
                }
            }
            State.SET_DATE -> {
                if (receivedString == "OK") {
                    println("[JeVois] [STATE: SET_DATE] Synced Time on JeVois")
                    timeOffset = Timer.getFPGATimestamp().second
                    state = State.START_STREAMING
                    serialPort.writeString("streamon\n")
                } else {
                    println("[JeVois] [STATE: SET_DATE] Failed to Sync time on JeVois")
                }
            }
            State.START_STREAMING -> {
                if (receivedString == "OK") {
                    println("[JeVois] [STATE: START_STREAMING] Started streaming data from JeVois...")
                    state = State.RUNNING
                } else {
                    println("[JeVois] [STATE: START_STREAMING] Failed to start Streaming data from JeVois")
                }
            }
            State.RUNNING -> {
                if (!receivedString.startsWith('{')) return
                try {
                    val jsonData = kJevoisGson.fromJson<JsonObject>(receivedString)

                    val timestamp = jsonData["Epoch Time"].asDouble.second + timeOffset
                    val contours = jsonData["Targets"].asJsonArray
                        .filterIsInstance<JsonObject>()

                    processData(VisionData(timestamp, contours))
                } catch (e: JsonParseException) {
                    e.printStackTrace()
                    println("[JeVois] Got Invalid Data: $receivedString")
                }
            }
        }
    }

    private fun SerialPort.writeString(data: String) = writeBytes(data.toByteArray())
    private fun SerialPort.writeBytes(data: ByteArray) = writeBytes(data, data.size.toLong())

    private enum class State {
        NONE,
        SET_OUTPUT,
        SET_DATE,
        START_STREAMING,
        RUNNING
    }

    companion object {
        private val kJevoisGson = Gson()
    }
}

data class VisionData(
    val timestamp: Time,
    val targets: List<JsonObject>
)