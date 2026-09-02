// Verify VMC_calc_2 Jacobian j11..j22 via virtual work:
//   work conjugacy: F0 <-> dL0, Tp <-> dphi0
//   => tau1 = dL0/dphi1 * F0 + dphi0/dphi1 * Tp  (should equal j11*F0 + j12*Tp)
//   => tau4 = dL0/dphi4 * F0 + dphi0/dphi4 * Tp  (should equal j21*F0 + j22*Tp)
const D2R = Math.PI / 180;
const l1 = 0.215, l2 = 0.254, l3 = 0.254, l4 = 0.215, l5 = 0.0;

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
  return { phi2, phi3, L0, phi0 };
}

// their jacobian exactly as in VMC_calc.c
function jacobian_theirs(p1, p4) {
  const { phi2, phi3, L0, phi0 } = FK(p1, p4);
  const j11 = l1 * Math.sin(phi0 - phi3) * Math.sin(p1 - phi2) / Math.sin(phi3 - phi2);
  const j12 = l1 * Math.cos(phi0 - phi3) * Math.sin(p1 - phi2) / (L0 * Math.sin(phi3 - phi2));
  const j21 = l4 * Math.sin(phi0 - phi2) * Math.sin(phi3 - p4) / Math.sin(phi3 - phi2);
  const j22 = l4 * Math.cos(phi0 - phi2) * Math.sin(phi3 - p4) / (L0 * Math.sin(phi3 - phi2));
  return { j11, j12, j21, j22 };
}

const h = 1e-6;
function test(name, p1deg, p4deg) {
  const p1 = p1deg * D2R, p4 = p4deg * D2R;
  // FD jacobian of (L0, phi0) wrt (phi1, phi4)
  const dL0_d1 = (FK(p1 + h, p4).L0 - FK(p1 - h, p4).L0) / (2 * h);
  const dL0_d4 = (FK(p1, p4 + h).L0 - FK(p1, p4 - h).L0) / (2 * h);
  const dp0_d1 = (FK(p1 + h, p4).phi0 - FK(p1 - h, p4).phi0) / (2 * h);
  const dp0_d4 = (FK(p1, p4 + h).phi0 - FK(p1, p4 - h).phi0) / (2 * h);

  const t = jacobian_theirs(p1, p4);
  console.log(`--- ${name}: phi1=${p1deg} phi4=${p4deg}`);
  console.log(`  j11 vs dL0/dphi1 : theirs=${t.j11.toFixed(6)}  FD=${dL0_d1.toFixed(6)}`);
  console.log(`  j12 vs dphi0/dphi1: theirs=${t.j12.toFixed(6)}  FD=${dp0_d1.toFixed(6)}`);
  console.log(`  j21 vs dL0/dphi4 : theirs=${t.j21.toFixed(6)}  FD=${dL0_d4.toFixed(6)}`);
  console.log(`  j22 vs dphi0/dphi4: theirs=${t.j22.toFixed(6)}  FD=${dp0_d4.toFixed(6)}`);
}

test("stance near vertical", 100, 80);
test("crouch", 125, 55);
test("asymmetric", 110, 70);
test("extended", 85, 95);
