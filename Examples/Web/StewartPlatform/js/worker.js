// 1 (1 byte for sync) = 0xEA
// 1 (1 byte for data length from this point) = 17 (16 + 1 command byte)
// 1 (1 byte for command) = 0x21
// 16 (4 bytes per float, 4 floats for quaternion)
// 2 (2 bytes for checksum)
const payloadLength = 16;
const bufferSize = 1 + 1 + 1 + payloadLength + 2;
let passRate;

let buffer;

let port;
let reader;

addEventListener("message", async (evt) => {
  const { type, ...body } = evt.data;

  switch (evt.data.type) {
    case "CONNECT":
      connect(body.baudRate);
      break;
    case "DISCONNECT":
      disconnect();
      break;
  }
});

/**
 * @name connect
 * Opens a Web Serial connection to a micro:bit and sets up the input and
 * output stream.
 */
async function connect(baudRate) {
  try {
    [port] = await navigator.serial.getPorts();
    await port.open({ baudRate });
    port.addEventListener("disconnect", disconnect);

    buffer = new ArrayBuffer(bufferSize);
    reader = port.readable.getReader();

    readLoop().catch((e) => {
      console.log(e);
    });
    self.postMessage({ type: "CONNECTED" });
  } catch (err) {
    self.close();
  }
}

/**
 * @name disconnect
 * Closes the Web Serial connection.
 */
async function disconnect(err) {
  console.log(err);

  if (reader) {
    await reader.cancel();
    reader = null;
    inputDone = null;
  }

  await port.forget();
  self.postMessage({ type: "DISCONNECTED" });
}

/**
 * @name readLoop
 * Reads data from the input stream and displays it on screen.
 */
async function readLoop() {
  // let lastPass = 0;
  passRate = {
    received: 0,
    passed: 0,
    get rate() {
      if (this.received == 0) return "No packets received";
      return (this.passed / this.received) * 100;
    },
  };

  let trimmedBuffer = [];

  while (true) {
    const { value, done } = await reader.read();

    if (done) {
      console.log("[readLoop] DONE", done);
      reader.releaseLock();
      break;
    }

    if (value.length < 1) continue;

    const totalBuffer = [...trimmedBuffer, ...value];
    trimmedBuffer = [];

    for (let start = 0; start < totalBuffer.length; ) {
      if (totalBuffer[start] != 0xea) continue;

      const buffer = [0xea];

      start += 1;
      for (; start < totalBuffer.length; start++) {
        if (totalBuffer[start] === 0xea) break;
        buffer.push(totalBuffer[start]);
      }

      if (buffer.length < bufferSize) {
        trimmedBuffer = buffer;
        continue;
      }

      try {
        const [, totalLength, command, ...data] = buffer;

        ++passRate.received;

        if (totalLength !== payloadLength + 1) {
          throw new Error(`Total Length Failed Given = ${totalLength}`);
        }

        if (command !== 0x21) {
          throw new Error(`Command Failed Given = ${command}`);
        }

        const positionBytes = data.slice(0, payloadLength);

        const expectedChecksum = new Uint16Array(
          new Uint8Array(data.slice(payloadLength, payloadLength + 2)).buffer
        )[0];
        const calculatedChecksum =
          command + positionBytes.reduce((acc, cur) => acc + cur, 0);

        if (calculatedChecksum != expectedChecksum) {
          throw new Error(
            `Checksum Failed... C=${calculatedChecksum} E=${expectedChecksum}`
          );
        }

        ++passRate.passed;

        // console.log(passRate.received - lastPass); // Calculates no. of packets since last complete packet
        // lastPass = passRate.received;

        self.postMessage({
          type: "DATA_READ",
          value: [
            ...new Float32Array(new Uint8Array(positionBytes).buffer),
            0,
            0,
            0,
          ],
        });
      } catch (error) {
        console.groupCollapsed("Failed");
        console.log(buffer);
        console.log(error.message);
        console.groupEnd();
      }
    }
  }
}
