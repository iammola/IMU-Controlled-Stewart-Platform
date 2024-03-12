let platform;
let quaternion = Quaternion.ONE;
const canvas = document.getElementById("canvas");
const rotation = { active: false, x: 0, y: 0, cx: 0, cy: 0 };

function startViewAngleChange(ev) {
  rotation.cx = ev.pageX - rotation.x;
  rotation.cy = ev.pageY - rotation.y;
  rotation.active = true;
}

function stopViewAngleChange() {
  rotation.active = false;
}

function moveViewAngleChange(ev) {
  if (!rotation.active) return;

  rotation.x = ev.pageX - rotation.cx;
  rotation.y = ev.pageY - rotation.cy;
}

addEventListener("DOMContentLoaded", () => {
  addEventListener("mouseup", stopViewAngleChange);
  addEventListener("mousemove", moveViewAngleChange);
  canvas.addEventListener("mousedown", startViewAngleChange);

  addEventListener(
    "dblclick",
    async () => {
      await navigator.serial.requestPort();
      worker.postMessage({ type: "CONNECT", baudRate: 115200 });
    },
    { once: true }
  );

  if ("serial" in navigator) {
    const notSupported = document.getElementById("notSupported");
    notSupported.classList.add("hidden");
  }

  if (isWebGLAvailable()) {
    const webGLnotSupported = document.getElementById("webGLnotSupported");
    webGLnotSupported.classList.add("hidden");
  }

  worker = new Worker("./js/worker.js");
  worker.addEventListener("message", async ({ data }) => {
    const { type, ...body } = data;

    switch (type) {
      case "DATA_READ":
        try {
          quaternion = new Quaternion(body.value);
        } catch (err) {
          console.log(body.value);
        }
        break;
      default:
        break;
    }
  });

  const sketch = function (p) {
    p.setup = function () {
      p.createCanvas(window.innerWidth, window.innerHeight, p.WEBGL);

      p.camera(
        100.0,
        -window.innerHeight / 2 + 10,
        p.height / 2.0 / Math.tan(Math.PI / 6),
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0
      );

      platform = new Stewart();

      platform.initHexagonal({
        baseRadius: 60,
        baseRadiusOuter: 83.2665,
        platformRadius: 45,
        platformRadiusOuter: 70.946,
        platformTurn: true,
        rodLength: 75,
        hornLength: 38.1,
        hornDirection: 0,
        shaftDistance: 14.825,
        anchorDistance: 22.5,
        servoRange: [0, Math.PI],
        servoRangeVisible: true,
      });
    };
    p.draw = function () {
      p.background(255);

      p.push();

      p.translate(50, -70, 200);
      p.rotateX(Math.PI / 2 - rotation.y / 400); // Work in correct X-Y-Z plane
      p.rotateY(rotation.x / 400);

      platform.update([0, 0, 0], quaternion);
      platform.draw(p); // Draw the updated platform

      p.pop();
    };
  };

  new p5(sketch, "canvas");
});

let isWebGLAvailable = function () {
  try {
    var canvas = document.createElement("canvas");
    return !!(
      window.WebGLRenderingContext &&
      (canvas.getContext("webgl") || canvas.getContext("experimental-webgl"))
    );
  } catch (e) {
    return false;
  }
};
