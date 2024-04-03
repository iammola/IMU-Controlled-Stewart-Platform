// 1 (1 byte for sync) = 0xEA
// 1 (1 byte for total length from this point) = 30
// 1 (1 byte for command) = 0x21
// 1 (1 byte for actual data length) = 28
// 28 (4 bytes per float, 4 floats for quaternion, 3 floats for translation)
// 2 (2 bytes for checksum)
const bufferSize = 1 + 1 + 1 + 1 + 28 + 2;

let buffer;

let port;
let reader;

let orientation = [0, 0, 0];
let quaternion = [1, 0, 0, 0];

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
  [port] = await navigator.serial.getPorts();
  await port.open({ baudRate, bufferSize });
  port.addEventListener("disconnect", disconnect);

  buffer = new ArrayBuffer(bufferSize);
  reader = port.readable.getReader({ mode: "byob" });

  readLoop().catch((e) => {
    console.log(e);
  });
  self.postMessage({ type: "CONNECTED" });
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
  while (true) {
    const { value, done } = await reader.read(new Uint8Array(buffer));

    if (done) {
      console.log("[readLoop] DONE", done);
      reader.releaseLock();
      break;
    }

    buffer = value.buffer;
    const [syncWord, totalLength, command, payloadLength, ...data] = value;
    // console.log({ syncWord, totalLength, command, payloadLength, data });

    if (syncWord !== 0xea) continue;
    // console.log("Passed Sync");
    if (totalLength !== payloadLength + 2) continue;
    // console.log("Passed Total Length");
    if (command !== 0x21) continue;
    // console.log("Passed Command");

    const positionBytes = data.slice(0, payloadLength);

    const expectedChecksum = new Uint16Array(
      new Uint8Array(data.slice(payloadLength)).buffer
    )[0];
    const calculatedChecksum =
      command +
      payloadLength +
      positionBytes.reduce((acc, cur) => acc + cur, 0);

    if (calculatedChecksum != expectedChecksum) {
      console.groupCollapsed("Checksum Failed");
      console.log(value);
      console.groupEnd();
      // console.log({ expectedChecksum, calculatedChecksum });
      continue;
    }
    // console.log("Passed Checksum");

    self.postMessage({
      type: "DATA_READ",
      value: new Float32Array(new Uint8Array(positionBytes).buffer),
    });
  }
}
