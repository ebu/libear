let [wasm_file] = process.argv.slice(process.argv.indexOf("--") + 1);

import {
  EARLibrary,
  GainCalculatorObjects,
  ObjectsTypeMetadata,
} from "../src/libear.mjs";
import fs from "node:fs";
import { test } from "node:test";
import assert from "node:assert";

/// check that two arrays are the same size and the same within a small tolerance
function assertArraysClose(a, b) {
  assert.strictEqual(a.length, b.length);

  for (let i = 0; i < a.length; i++) {
    assert.ok(Math.abs(a[i] - b[i]) < 1e-6);
  }
}

function assertClose(a, b) {
  assert.ok(Math.abs(a - b) < 1e-6);
}

test("load library then", async (t) => {
  const wasm_buf = fs.readFileSync(wasm_file);
  let lib = await EARLibrary.from_buffer(wasm_buf);

  await test("layout", (t) => {
    let layout = lib.get_layout("0+5+0");
    assert.equal(layout.num_channels(), 6);
  });

  await test("get_layout throw", (t) => {
    assert.throws(() => lib.get_layout("0+5+1"), {
      name: "RangeError",
      message: "unknown layout: 0+5+1",
    });
  });

  await test("decorrelator", (t) => {
    let layout = lib.get_layout("0+5+0");

    assert.equal(lib.decorrelator_compensation_delay(), 255);

    let dec = lib.design_decorrelator(layout, 1);
    assert.equal(dec.length, 512);

    assertClose(dec[0], 0.06860811);
    assertClose(dec[256], 0.111747);
    assertClose(dec[511], -0.01473256);
  });

  await test("calculate objects gains", (t) => {
    let layout = lib.get_layout("0+5+0");
    let gain_calc = new GainCalculatorObjects(lib, layout);

    let otm = new ObjectsTypeMetadata(lib);
    otm.set_polar_position(15.0, 0.0, 1.0);
    otm.set_gain(0.5);
    otm.set_diffuse(0.25);

    let gains = gain_calc.calculate(otm);
    assertArraysClose(
      gains["direct"],
      [0.3061862178478973, 0, 0.3061862178478973, 0, 0, 0],
    );
    assertArraysClose(
      gains["diffuse"],
      [0.1767766952966369, 0, 0.1767766952966369, 0, 0, 0],
    );
  });
});
