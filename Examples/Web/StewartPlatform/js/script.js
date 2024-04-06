let platform;

const images = document.getElementById("images");
const canvas = document.getElementById("canvas");

const rotation = { active: false, x: 0, y: 0, cx: 0, cy: 0 };

const SVGS = [
  {
    // arrow
    path: "M 480.00,320.00L 256.00,320.00L 256.00,480.00L 32.00,256.00L 256.00,32.00L 256.00,192.00L 480.00,192.00 z",
    box: { x: 0, y: 0, width: 512, height: 512 },
  },
  {
    // tesla
    path: "M12 5.362l2.475-3.026s4.245.09 8.471 2.054c-1.082 1.636-3.231 2.438-3.231 2.438-.146-1.439-1.154-1.79-4.354-1.79L12 24 8.619 5.034c-3.18 0-4.188.354-4.335 1.792 0 0-2.146-.795-3.229-2.43C5.28 2.431 9.525 2.34 9.525 2.34L12 5.362l-.004.002H12v-.002zm0-3.899c3.415-.03 7.326.528 11.328 2.28.535-.968.672-1.395.672-1.395C19.625.612 15.528.015 12 0 8.472.015 4.375.61 0 2.349c0 0 .195.525.672 1.396C4.674 1.989 8.585 1.435 12 1.46v.003z",
    box: { x: 0, y: 0, width: 24, height: 24 },
  },
  {
    // airbnb
    path: "M11.998 18.267c-1.352-1.696-2.147-3.182-2.412-4.455-.263-1.026-.159-1.847.291-2.464.477-.71 1.187-1.055 2.12-1.055s1.642.345 2.119 1.062c.446.61.558 1.432.286 2.464-.291 1.298-1.085 2.784-2.411 4.456zm9.597 1.14c-.185 1.245-1.034 2.278-2.2 2.782-2.251.98-4.48-.583-6.388-2.703 3.155-3.95 3.738-7.025 2.384-9.014-.795-1.14-1.933-1.695-3.393-1.695-2.943 0-4.561 2.49-3.925 5.38.37 1.564 1.351 3.342 2.915 5.33-.98 1.084-1.909 1.855-2.73 2.332-.636.344-1.245.557-1.828.608-2.677.399-4.776-2.198-3.823-4.877.132-.345.395-.98.845-1.961l.025-.053C4.94 12.36 6.717 8.75 8.759 4.746l.053-.132.58-1.115c.45-.822.635-1.19 1.351-1.643.345-.209.769-.314 1.245-.314.954 0 1.697.557 2.015 1.006.158.239.345.557.582.953l.558 1.088.08.159c2.04 4.002 3.819 7.605 5.276 10.789l.026.025.533 1.22.318.764c.243.612.294 1.221.213 1.857zm1.219-2.389c-.186-.583-.504-1.271-.9-2.093v-.03c-1.887-4.005-3.64-7.605-5.304-10.84l-.111-.162C15.313 1.461 14.464 0 11.998 0 9.56 0 8.524 1.694 7.465 3.897l-.081.16c-1.668 3.234-3.42 6.839-5.301 10.842v.053l-.558 1.219c-.21.504-.317.768-.345.847-1.35 3.712 1.432 6.972 4.8 6.972.027 0 .132 0 .264-.027h.372c1.75-.213 3.553-1.325 5.382-3.316 1.828 1.988 3.633 3.103 5.38 3.316h.372c.132.027.238.027.264.027 3.368.003 6.15-3.26 4.8-6.972z",
    box: { x: 0, y: 0, width: 24, height: 24 },
  },
  {
    // npm
    path: "M0 7.334v8h6.666v1.332H12v-1.332h12v-8H0zm6.666 6.664H5.334v-4H3.999v4H1.335V8.667h5.331v5.331zm4 0v1.336H8.001V8.667h5.334v5.332h-2.669v-.001zm12.001 0h-1.33v-4h-1.336v4h-1.335v-4h-1.33v4h-2.671V8.667h8.002v5.331zM10.665 10H12v2.667h-1.335V10z",
    box: { x: 0, y: 0, width: 24, height: 24 },
  },
  {
    // github
    path: "M12 .297c-6.63 0-12 5.373-12 12 0 5.303 3.438 9.8 8.205 11.385.6.113.82-.258.82-.577 0-.285-.01-1.04-.015-2.04-3.338.724-4.042-1.61-4.042-1.61C4.422 18.07 3.633 17.7 3.633 17.7c-1.087-.744.084-.729.084-.729 1.205.084 1.838 1.236 1.838 1.236 1.07 1.835 2.809 1.305 3.495.998.108-.776.417-1.305.76-1.605-2.665-.3-5.466-1.332-5.466-5.93 0-1.31.465-2.38 1.235-3.22-.135-.303-.54-1.523.105-3.176 0 0 1.005-.322 3.3 1.23.96-.267 1.98-.399 3-.405 1.02.006 2.04.138 3 .405 2.28-1.552 3.285-1.23 3.285-1.23.645 1.653.24 2.873.12 3.176.765.84 1.23 1.91 1.23 3.22 0 4.61-2.805 5.625-5.475 5.92.42.36.81 1.096.81 2.22 0 1.606-.015 2.896-.015 3.286 0 .315.21.69.825.57C20.565 22.092 24 17.592 24 12.297c0-6.627-5.373-12-12-12",
    box: { x: 0, y: 0, width: 24, height: 24 },
  },
  {
    //heart
    path: "M333.1064,35.7615c-25.5879-25.4733-65.1964-32.2185-97.864-16.8292 c-16.2944,7.6759-29.9368,20.5657-41.1835,34.4202c-4.4318,5.4589-8.631,13.7229-13.0043,19.5697 C171.9642,33.148,134.9549,1.2761,94.2291,0C57.5233,0.3392,22.1679,19.2918,7.685,54.0688 c-14.6627,35.2102-7.5398,75.8396,14.5368,106.237c24.4552,33.6715,60.2457,51.0753,95.8722,70.2787 c19.9611,10.7595,38.9875,21.1357,49.8667,41.7977c4.9961,9.488,8.7819,20.0864,10.6461,30.6843 c0.5258,2.9889,5.2417,2.2312,5.2167-0.7055c-0.4458-52.9605,58.1166-70.7511,95.457-92.9536 c31.9315-18.9867,63.4365-43.6359,75.2526-80.3814C364.9381,96.6696,357.4074,59.953,333.1064,35.7615z",
    box: { x: 0, y: 0, width: 359.0891, height: 304.9693 },
  },
  {
    path: "M 200 150 A 50 50 0 0 1 114 185 M 217 150 A 67 67 0 0 1 83 150 M 234 150 A 84 84 0 1 1 81 101 M 251 150 A 101 101 0 1 1 115 55 M 268 150 A 118 118 0 1 1 149 32",
    box: { x: 0, y: 0, width: 300, height: 300 },
  },
  {
    path: "M292.5 103.7l-120.6 16.1-21.9-119.6-21.9 119.6-120.6-16.1 107 57.8-52.6 109.7 88.1-83.9 88.1 83.9-52.6-109.7 107-57.8",
    box: { x: 0, y: 0, width: 300, height: 300 },
  },
];

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

function startAnimation(e) {
  if (e.keyCode === 32) {
    animation.toggleVisiblePath();
    e.preventDefault();
    return;
  }

  animation.start(String.fromCharCode(e.keyCode | 32));
  //animation.moveTo([0,0,120], Quaternion.ONE, 1000, null);
}

function setupPlatform() {
  new p5((p) => {
    p.setup = function () {
      const width = window.innerWidth - 100;
      const height = window.innerHeight - 100;
      p.createCanvas(width, height, p.WEBGL);

      p.camera(
        width / 4,
        -height / 1.5,
        p.height / 2 / Math.tan(Math.PI / 6),
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0
      );

      platform = new Stewart();
      animation = new Animation(platform);

      const range = (90 * Math.PI) / 180;
      platform.initHexagonal({
        baseRadius: 60,
        baseRadiusOuter: 83.2665,
        platformRadius: 45,
        platformRadiusOuter: 70.946,
        platformTurn: false,
        rodLength: 150,
        hornLength: 31.75,
        hornDirection: 0,
        shaftDistance: 20,
        anchorDistance: 22.225,
        servoRange: [-range, range],
        servoRangeVisible: true,
      });
    };

    p.draw = function () {
      p.background("#646364");

      p.push();

      p.translate(50, -70, 200);
      p.rotateX(Math.PI / 2 - rotation.y / 400); // Work in correct X-Y-Z plane
      p.rotateY(rotation.x / 400);

      // Draw motion path
      animation.drawPath(p);

      // Tick animation
      animation.update(p);

      // Draw the updated platform
      platform.draw(p);

      // Send to servos
      // console.log(platform.getServoAngles().map(i => i * 180 / Math.PI));

      p.pop();
    };
  }, "canvas");
}

addEventListener("DOMContentLoaded", () => {
  setupPlatform();
  addEventListener("mouseup", stopViewAngleChange);
  addEventListener("mousemove", moveViewAngleChange);
  addEventListener("keydown", startAnimation);
  canvas.addEventListener("mousedown", startViewAngleChange);

  function createSVGImage(id, root, d, box) {
    const xmlns = "http://www.w3.org/2000/svg";
    const svg = document.createElementNS(xmlns, "svg");
    svg.setAttribute("viewBox", `${box.x} ${box.y} ${box.width} ${box.height}`);
    svg.setAttribute("width", 35);
    svg.setAttribute("height", 35);
    svg.setAttribute("style", "cursor: pointer");

    svg.addEventListener("click", () =>
      animation._start(Animation.SVG(SVGS[id].path, SVGS[id].box), null)
    );

    var path = document.createElementNS(xmlns, "path");
    path.setAttribute("stroke", "#000000");
    path.setAttribute("stroke-width", 0.001);

    path.setAttribute("d", d);
    svg.appendChild(path);

    root.appendChild(svg);
  }

  for (var i = 0; i < SVGS.length; i++) {
    createSVGImage(i, images, SVGS[i].path, SVGS[i].box);
  }
});
