const canvas = document.getElementById("thermalCanvas");
const ctx = canvas.getContext("2d");
const frameValue = document.getElementById("frameValue");
const bodyValue = document.getElementById("bodyValue");
const insideValue = document.getElementById("insideValue");

const cols = 32;
const rows = 24;
let frame = 0;
let inside = 3;

const people = [
  { x: 6, y: 10, vx: 0.045, vy: 0.022, heat: 1.0, size: 3.1 },
  { x: 20, y: 13, vx: -0.033, vy: -0.014, heat: 0.92, size: 2.7 },
  { x: 27, y: 8, vx: -0.018, vy: 0.026, heat: 0.78, size: 2.3 }
];

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function palette(t) {
  const stops = [
    [6, 8, 26],
    [20, 35, 92],
    [28, 155, 168],
    [142, 232, 92],
    [255, 173, 58],
    [255, 72, 92],
    [255, 244, 218]
  ];
  const scaled = clamp(t, 0, 1) * (stops.length - 1);
  const i = Math.floor(scaled);
  const f = scaled - i;
  const a = stops[i];
  const b = stops[Math.min(i + 1, stops.length - 1)];
  return [
    Math.round(a[0] + (b[0] - a[0]) * f),
    Math.round(a[1] + (b[1] - a[1]) * f),
    Math.round(a[2] + (b[2] - a[2]) * f)
  ];
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * dpr);
  canvas.height = Math.round(rect.height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function updatePeople() {
  for (const person of people) {
    person.x += person.vx;
    person.y += person.vy;

    if (person.x < 3 || person.x > cols - 4) {
      person.vx *= -1;
      inside = clamp(inside + (person.x < 3 ? 1 : -1), 0, 9);
    }

    if (person.y < 4 || person.y > rows - 4) {
      person.vy *= -1;
    }
  }
}

function thermalValue(x, y) {
  let value = 0.11 + 0.04 * Math.sin((x + frame * 0.025) * 0.6) + 0.03 * Math.cos(y * 0.7);

  for (const person of people) {
    const dx = x - person.x;
    const dy = y - person.y;
    const dist = (dx * dx + dy * dy) / (person.size * person.size);
    value += person.heat * Math.exp(-dist);
    value += person.heat * 0.22 * Math.exp(-dist * 0.25);
  }

  return clamp(value, 0, 1);
}

function drawGrid(width, height, cellW, cellH) {
  ctx.save();
  ctx.globalAlpha = 0.28;
  ctx.strokeStyle = "rgba(255,255,255,0.16)";
  ctx.lineWidth = 1;

  for (let x = 0; x <= cols; x += 4) {
    ctx.beginPath();
    ctx.moveTo(x * cellW, 0);
    ctx.lineTo(x * cellW, height);
    ctx.stroke();
  }

  for (let y = 0; y <= rows; y += 4) {
    ctx.beginPath();
    ctx.moveTo(0, y * cellH);
    ctx.lineTo(width, y * cellH);
    ctx.stroke();
  }

  ctx.restore();
}

function drawTracking(cellW, cellH) {
  ctx.save();
  ctx.lineWidth = 2;
  ctx.strokeStyle = "rgba(255,255,255,0.72)";
  ctx.fillStyle = "rgba(8,9,13,0.72)";
  ctx.font = "700 12px system-ui, sans-serif";

  people.forEach((person, index) => {
    const x = person.x * cellW;
    const y = person.y * cellH;
    const r = person.size * cellW * 0.72;

    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.stroke();

    const label = `ID ${index + 1}`;
    const labelW = ctx.measureText(label).width + 12;
    ctx.fillRect(x - labelW / 2, y - r - 26, labelW, 20);
    ctx.strokeRect(x - labelW / 2, y - r - 26, labelW, 20);
    ctx.fillStyle = "#f4f7fb";
    ctx.fillText(label, x - labelW / 2 + 6, y - r - 12);
    ctx.fillStyle = "rgba(8,9,13,0.72)";
  });

  ctx.restore();
}

function draw() {
  frame += 1;
  updatePeople();

  const rect = canvas.getBoundingClientRect();
  const width = rect.width;
  const height = rect.height;
  const cellW = width / cols;
  const cellH = height / rows;

  ctx.clearRect(0, 0, width, height);

  for (let row = 0; row < rows; row += 1) {
    for (let col = 0; col < cols; col += 1) {
      const value = thermalValue(col, row);
      const [r, g, b] = palette(value);
      ctx.fillStyle = `rgb(${r}, ${g}, ${b})`;
      ctx.fillRect(col * cellW, row * cellH, Math.ceil(cellW), Math.ceil(cellH));
    }
  }

  drawGrid(width, height, cellW, cellH);

  ctx.save();
  ctx.strokeStyle = "rgba(66,232,244,0.85)";
  ctx.setLineDash([8, 8]);
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(16 * cellW, 0);
  ctx.lineTo(16 * cellW, height);
  ctx.stroke();
  ctx.restore();

  drawTracking(cellW, cellH);

  frameValue.textContent = String(frame).padStart(4, "0");
  bodyValue.textContent = String(people.length);
  insideValue.textContent = String(inside);

  requestAnimationFrame(draw);
}

window.addEventListener("resize", resizeCanvas);
resizeCanvas();
draw();
