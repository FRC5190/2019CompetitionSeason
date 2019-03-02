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
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.second

class JeVois(
    private val serialPort: SerialPort,
    private val processData: (VisionData) -> Unit
) {

    private var lastPingSent = 0.second
    var lastPingReceived = 0.second
        private set
    var ping = 0.second
        private set

    init {
        serialPort.openPort()
        serialPort.addDataListener(object : SerialPortDataListener {
            private var byteBuffer = ByteArray(1024)
            private var bufferIndex = 0

            override fun serialEvent(event: SerialPortEvent) {
                try {
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
                } catch (e: Throwable) {
                    e.printStackTrace()
                }
            }

            override fun getListeningEvents() = SerialPort.LISTENING_EVENT_DATA_AVAILABLE
        })
        lastPingReceived = Timer.getFPGATimestamp().second
    }

    private fun onStringReceived(receivedString: String) {
        if (receivedString.equals("ALIVE", true)) {
            lastPingReceived = Timer.getFPGATimestamp().second
            ping = lastPingReceived - lastPingSent
            println("[JeVois] Got Ping. Took ${ping.millisecond}ms")
            return
        }
        if (!receivedString.startsWith('{')) return
        try {
            val jsonData = kJevoisGson.fromJson<JsonObject>(receivedString)

            val timestamp = (Timer.getFPGATimestamp() - jsonData["capture_ago"].asDouble).second
            val contours = jsonData["targets"].asJsonArray
                .filterIsInstance<JsonObject>()

            processData(VisionData(timestamp, contours))
        } catch (e: JsonParseException) {
            e.printStackTrace()
            println("[JeVois] Got Invalid Data: $receivedString")
        }
    }

    fun update() {
        val currentTime = Timer.getFPGATimestamp().second
        if (currentTime - lastPingSent > Constants.kVisionCameraPing) {
            lastPingSent = currentTime
            serialPort.writeString("ping\n")
        }

        if (lastPingSent - lastPingReceived > Constants.kVisionCameraTimeout) {
            println("[JeVois] stopped communicating!")
        }
    }

    private fun SerialPort.writeString(data: String) = writeBytes(data.toByteArray())
    private fun SerialPort.writeBytes(data: ByteArray) = writeBytes(data, data.size.toLong())

    companion object {
        private val kJevoisGson = Gson()
    }
}

data class VisionData(
    val timestamp: Time,
    val targets: List<JsonObject>
)