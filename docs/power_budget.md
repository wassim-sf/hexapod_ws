# Power budget and servo power switch

Run [`power_budget.py`](../scripts/power_budget.py) for the numbers below. They use the estimated robot mass of 2192 g, so recompute once the robot is weighed. Wiring reference: MakeYourPet's [wiring diagram](https://github.com/MakeYourPet/hexapod/blob/main/wiring-diagram-servo2040.png).

## Torque margin

Static figures at 6 V: load = weight / feet on the ground, MG996R stall 1.08 N·m.

| Standing tripod, foot from hip | Femur | Tibia |
|---|---|---|
| 100 mm out, 85 mm down | 38% of stall | 13% |
| **120 mm out, 85 mm down (standby)** | **51%** | **24%** |
| 140 mm out, 85 mm down | 64% | 34% |
| 160 mm out, 85 mm down | 78% | 44% |

| Default gait, worst moment | Femur | Tibia |
|---|---|---|
| Tripod | 54% | 26% |
| Ripple | 43% | 21% |
| Wave | 35% | 17% |

Accelerations and foot impacts add to these, and a wider stance quickly uses up the margin. Each extra 40 g adds about 1% of femur stall.

## Current

Estimated from stance torque and running current, so treat it as ±50%, and measure it on the real robot.

| Case at 6 V | Current | XL4016 (8 A) |
|---|---|---|
| Walking, tripod, mean | about 11 A (66 W) | **1.4x its rating** |
| All servos running, seller figures | 9–16 A | Over its rating |
| All stalled | 45 A (270 W) | **5.6x its rating** |

The XL4016's [8 A rating](https://www.alldatasheet.com/datasheet-pdf/pdf/1134370/XLSEMI/XL4016E1.html) comes from a datasheet mirror and may need derating for heat. MakeYourPet's config warns at 8 A and shuts off at 10 A.

## Battery

3S at 11.1 V, regulators at 90%, using 80% of capacity. Walking draws about 7.5 A from the pack; the peak is about 27 A, at the BEC's 40 A limit with the Pi at full load.

| Capacity | Walking time | C rating for the peak |
|---|---|---|
| 2200 mAh | 14 min | 12C |
| 3300 mAh | 21 min | 8C |
| 4000 mAh | 26 min | 7C |
| 5000 mAh | 32 min | 5.3C |

**For about 20 minutes, use 3S 3300–4000 mAh at 20C or higher.** The margin in C covers cell ageing and the ±50% uncertainty. A 3300 mAh pack adds about 95 g, or about 2% of femur stall.

## What the original wiring does

MakeYourPet's diagram has a 2S pack feeding the servo rail directly, through a hand switch and then a relay. The controller switches the relay from pin A0, which turns the servos on and off in software. Touch sensors use NC (ground), NO (3V3) and C (signal).

The Robot Lk guide adds a buck converter set to 5.5–6 V, because it uses a 3S pack.

| Part | Original job | Rating | In this robot |
|---|---|---|---|
| Buck converter | 3S down to 5.5–6 V for the servos | 8 A | **Replaced** by the 6 V BEC |
| 5 V relay module | Software on/off for servo power | 10 A contacts | **Not reused:** walking already exceeds it. The function is kept, below |
| Hand switch | Manual servo power | Small toggle | **Kept, on the relay coil**, so that it can't be asked to carry servo current |

## Power layout

```text
3S LiPo ── 40 A fuse ─┬─ servo power switch ── 6 V BEC (≥20 A, ~40 A peak) ── servo rail, 18 x MG996R
                      │        ▲
                      │        └─ ESP32 GPIO 32, HIGH = on, off by default
                      │
                      └─ 5 V 5 A regulator ── Pi 5 ── UART ── ESP32 ── I2C ── 2 x PCA9685
                                                               └─ GPIO 33 ── PCA9685 OE, LOW = outputs on
```

The switch is on the BEC's input, so at stall it carries about 27 A, not 45 A. The rating below is still well above 45 A, as asked. The Pi and ESP32 stay powered when the servos are off, so they can log the fault and restart cleanly.

## Servo power switch options

| | **A. Automotive relay (recommended)** | B. Solid-state relay | C. D4184 MOSFET module |
|---|---|---|---|
| Part | [JD2912-1H 12 VDC, 80 A](https://www.noco-evco.com/images/products/JD2912/JD2912_datasheet.pdf), SPST-NO | [Crydom D1D80](https://www.trustedparts.com/en/part/crydom/D1D80), SPST-NO | [D4184 module](https://protosupplies.com/product/d4184-mosfet-control-module/), AOD4184A |
| Rating | 80 A **at 1% duty**, 75 VDC max; continuous rating not stated | 80 A, 0–100 VDC (distributor listing) | 40 V; 50 A claimed, and vendors warn it overheats well before that |
| Switches | Positive line | Positive line | **Negative line only** |
| Control | Coil ≤ 1.8 W, about 150 mA; pulls in at ≤ 60% of 12 V | 3.5–32 VDC, 15 mA max | Logic input, 3.3 or 5 V claimed |
| ESP32 3.3 V | Needs a transistor | Needs a 5 V level: 3.3 V is below its 3.5 V minimum | Direct, though the MOSFET's RDS(on) is specified at 4.5 V gate drive |
| Size, mass | About 28 x 28 x 25 mm, 35 g | Not verified; needs a heatsink | 23 x 16 mm, a few grams |
| Main drawback | Coil draws power constantly, and contacts wear | Bulk, heat, cost | Lifts the BEC's ground off the ESP32's when off, sending current through signal wires |

**Why A:** it's simple, cheap and fits. It carries about 7 A while walking and about 27 A only briefly in a stall, far under its 80 A nameplate.

**Check before relying on A:**

- **Continuous rating.** The 80 A is at 1% duty, so measure the contact temperature at 10–15 A for 20 minutes. If it runs hot, use a relay with a stated continuous rating.
- **Contact resistance.** The datasheet gives 100 mΩ at 6 V / 80 A, which looks high. Measure the voltage drop at load.
- **Inrush.** Closing onto the BEC's input capacitors causes a current spike. Check whether your BEC specifies a limit.

**Why not C:** low-side switching is the problem. With the servo ground disconnected, return current can flow through the PCA9685 and signal lines. It's only worth considering if the BEC's input and output grounds are truly isolated, which hobby BECs usually aren't.

**Relay wiring:**

- Coil from the pack side of the fuse, through the hand switch, to a logic-level N-MOSFET (for example IRLZ44N) on the low side.
- GPIO 32 to the MOSFET gate through 100 Ω, with a 10 kΩ pull-down, so that the relay stays open while the ESP32 boots or if it resets.
- A 1N4007 flyback diode across the coil.

**PCA9685 OE:** connect both boards' OE pins to GPIO 33 with a 10 kΩ pull-up to 3V3, so that outputs are off at boot. Pulling it LOW enables them. This stops the servo signal only and doesn't cut power. Enable order: relay first, send the current pose to the PCA9685, then OE, so that servos don't jump.

## Where it goes

[`check_bays.py`](../scripts/check_bays.py) places the relay on the top cover's rear bar: x −66 to −38, y ±14, z 28.5 to 53.5 mm, clear of the Pi and the rest of the body. The floor has only 16 mm of height under the Pi tray, and the under-body pockets are full.

- **Fuse:** next to the pack connector, under the body.
- **Wires:** at least 14 AWG from the pack through the fuse and relay to the BEC, routed down at the rear of the body. Keep this run short.
- **Conflict:** a lid or phone on top would need a cut-out or a different relay position.

Option B would not fit here without a new mount, as it needs a heatsink.

## Sources and confidence

MG996R figures come from seller pages ([components101](https://components101.com/motors/mg996r-servo-motor-datasheet), [Tinkered](https://www.tinkered.ai/components/servo-mg996r-straight), [ESPBoards](https://www.espboards.dev/sensors/mg996r/)), and clones vary, so measure stall current on your servos. Check the JD2912's, D1D80's and D4184's real datasheets against the exact parts you buy: the D1D80 figures here are from a distributor listing, and seller listings for the D4184 disagree with each other.
