// Numerical verification of VMC_calc.c analytic velocity formulas (lines 96-107)
// against central finite differences of the same forward kinematics.
const D2R = Math.PI / 180;
const l1 = 0.215, l2 = 0.254, l3 = 0.254, l4 = 0.215, l5 = 0.0;

// forward kinematics exactly as in VMC_calc.c
function FK(p1, p4) {
  const YD = l4 * Math.sin(p4), YB = l1 * Math.sin(p1);
  const XD = l5 + l4 * Math.cos(p4), XB = l1 * Math.cos(p1);
  const lBD = Math.hypot(XD - XB, YD - YB);
  const A0 = 2 * l2 * (XD - XB), B0 = 2 * l2 * (YD - YB);
  const C0 = l2 * l2 + lBD * lBD - l3 * l3;
  const phi2 = 2 * Math.atan2(B0 + Math.sqrt(A0 * A0 + B0 * B0 - C0 * C0), A0 + C0);
  const phi3 = Math.atan2(YB - YD + l2 * Math.sin(phi2), XB - XD + l2 * Math.cos(phi2));
  const XC = l1 * Math.cos(p1) + l2 * Math.cos(phi2);
  const YC = l1 * Math.sin(p1) + l2 * Math.sin(phi2);
  const L0 = Math.hypot(XC - l5 / 2, YC);
  const phi0 = Math.atan2(YC, XC - l5 / 2);
  return { phi2, phi3, XC, YC, L0, phi0 };
}

// analytic velocities exactly as in VMC_calc.c
function analytic(p1, p4, d1, d4) {
  const { phi2, phi3, XC, YC, L0 } = FK(p1, p4);
  const A1 = (l1 * d1 * Math.sin(p1 - phi3) + l4 * d4 * Math.sin(phi3 - p4)) / Math.sin(phi3 - phi2);
  const dxb = -l1 * d1 * Math.sin(p1);
  const dyb = l1 * d1 * Math.cos(p1);
  const xc = XC - l5 / 2, yc = YC;
  const dL0 = (yc * (dyb + A1 * Math.cos(phi2)) + xc * (dxb - A1 * Math.sin(phi2))) / L0;
  const dphi0 = (xc * (dyb + A1 * Math.cos(phi2)) - yc * (dxb - A1 * Math.sin(phi2))) / (L0 * L0);
  return { A1, dphi2: A1 / l2, dL0, dphi0 };
}

const h = 1e-6;
function test(name, p1deg, p4deg, d1, d4) {
  const p1 = p1deg * D2R, p4 = p4deg * D2R;
  // central differences
  const f2p = FK(p1 + h, p4).phi2, f2m = FK(p1 - h, p4).phi2;
  const f2q = FK(p1, p4 + h).phi2, f2r = FK(p1, p4 - h).phi2;
  const dphi2_fd = ((f2p - f2m) / (2 * h)) * d1 + ((f2q - f2r) / (2 * h)) * d4;
  const L0p = FK(p1 + h, p4).L0, L0m = FK(p1 - h, p4).L0;
  const L0q = FK(p1, p4 + h).L0, L0r = FK(p1, p4 - h).L0;
  const dL0_fd = ((L0p - L0m) / (2 * h)) * d1 + ((L0q - L0r) / (2 * h)) * d4;
  const p0p = FK(p1 + h, p4).phi0, p0m = FK(p1 - h, p4).phi0;
  const p0q = FK(p1, p4 + h).phi0, p0r = FK(p1, p4 - h).phi0;
  const dphi0_fd = ((p0p - p0m) / (2 * h)) * d1 + ((p0q - p0r) / (2 * h)) * d4;

  const a = analytic(p1, p4, d1, d4);
  const fmt = (x) => x.toFixed(6);
  console.log(`--- ${name}: phi1=${p1deg} phi4=${p4deg} dphi1=${d1} dphi4=${d4}`);
  console.log(`  dphi2 : analytic=${fmt(a.dphi2)}  FD=${fmt(dphi2_fd)}  diff=${Math.abs(a.dphi2 - dphi2_fd).toExponential(2)}`);
  console.log(`  dL0   : analytic=${fmt(a.dL0)}    FD=${fmt(dL0_fd)}    diff=${Math.abs(a.dL0 - dL0_fd).toExponential(2)}`);
  console.log(`  dphi0 : analytic=${fmt(a.dphi0)}  FD=${fmt(dphi0_fd)}  diff=${Math.abs(a.dphi0 - dphi0_fd).toExponential(2)}`);
}

test("stance near vertical", 100, 80, 2.0, -1.5);
test("crouch (larger angles)", 125, 55, 1.0, 1.0);
test("slow small motion", 92, 88, 0.5, -0.5);
test("asymmetric stance", 110, 70, -1.2, 0.8);
test("extended legs", 85, 95, 1.5, 1.5);
