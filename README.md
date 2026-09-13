# gearbox — NEMA17 16:1 axis bring-up

Bench firmware and a reusable library for one axis of the antenna/dish
rotator: the [wireless-actuator](https://github.com/curiosity-creates/wireless-actuator)
16:1 belt gearbox, driven by a TMC2209, with an **AS5048A** magnetic encoder on
the output shaft and an **ICM-20948** IMU on the arm. This build plays the
**elevation** axis: 0° is one horizon, 90° zenith, 180° the other horizon.

| Folder | What |
|---|---|
| [GearboxAxis/](GearboxAxis/) | The library. `AS5048A` (SPI encoder), `ArmIMU` (arm angle from gravity), `GearboxAxis` (ramped closed-loop axis). Junctioned into `Documents\Arduino\libraries` so the IDE sees it. |
| [ElevationSweep/](ElevationSweep/) | The sketch: bring-up, self-calibration, level, sweep, serial console. Pins in [config.h](ElevationSweep/config.h). |
| [build.ps1](build.ps1) | Compile / upload / monitor from a terminal using the arduino-cli bundled with Arduino IDE. |
| [talk.ps1](talk.ps1) | Send a scripted sequence of serial commands and print the replies. |
| [runsweep.ps1](runsweep.ps1) | Staged horizon-to-horizon sweep, recording both sensors at every station. |
| [runwave.ps1](runwave.ps1) | Continuous horizon-to-horizon wave at three increasing speeds. |

## Hardware

| Part | Adafruit / vendor | Chip | Bus |
|---|---|---|---|
| Feather ESP32-S3 4MB flash / 2MB PSRAM | #5477 | ESP32-S3 | — |
| 9-DoF IMU | #4554 | ICM-20948 @ `0x69` | I2C (STEMMA QT) |
| 1.12" 128x128 OLED | #5297 | SH1107 @ `0x3D` | I2C (STEMMA QT) |
| Magnetic encoder | ams AS5048A adapter board | AS5048A | SPI |
| Stepper | STEPPERONLINE 17HS19-2004S1 | 1.8°, 2 A/phase, bipolar 4-wire | via TMC2209 |
| Driver | Adafruit TMC2209 breakout #6121 | TMC2209, StealthChop, 2 A max | STEP/DIR/EN (UART optional) |

The board is the **2MB PSRAM** variant (`adafruit_feather_esp32s3`), not the
8MB no-PSRAM board the antenna-tracker prototype uses. It has quad PSRAM, so
the header SPI pins GPIO35/36/37 are free to use (the octal-PSRAM warning in
the tracker's wiring notes does not apply here).

## Wiring

Everything below is what `config.h` assumes. Wire by the **silkscreen** column.

### Power — read first

```
12 V 6 A supply (+) ──────────── TMC2209 terminal block "+"   (board has 22 µF on it already)
12 V 6 A supply (−) ──┬───────── TMC2209 terminal block "−"
                      └───────── Feather GND     ← the one shared ground
Feather 3V ───────────┬───────── TMC2209 VDD   (logic level, 3.3 V)
                      ├───────── TMC2209 MS1 and MS2   (both HIGH = 1/16 microstep)
                      └───────── AS5048A 3V3 **and** 5V (see encoder note)
Feather USB ───────────────────── ESP32 only. 12 V never touches the Feather.
```

* **Never plug or unplug the motor while the 12 V is on.** Any stepper driver can die from that.
* **MS1 and MS2 must both go to 3V.** Left open the driver runs 1/8 microstep and every commanded angle comes out half size. The firmware assumes 1/16.
* The TMC2209 has no RESET/SLEEP pins to jumper, unlike the A4988.
* Set the current before the first move — see below.

### Feather → TMC2209 breakout

| Feather silkscreen | GPIO | TMC2209 pin |
|---|---|---|
| **5** | 5 | STEP |
| **6** | 6 | DIR |
| **9** | 9 | EN (high = outputs off; firmware drives it) |
| **A2** | 16 | DIAG (optional; goes high on over-temp / short. Not read by the firmware yet) |
| **3V** | — | VDD, MS1, MS2 |
| **GND** | — | GND (the header pin; same net as the "−" terminal) |
| — | — | UART, INDEX: leave open for now |

Adafruit's example wires DIR to pin 5 and STEP to pin 6, the opposite of this
table. Follow this table; it matches `config.h`.

Reserved for the azimuth driver later: STEP/DIR/EN on **10 / 11 / 12**
(GPIO10/11/12), DIAG on **A3**. If UART control is added later (software
current, 1/256 microsteps, StallGuard), both drivers share **TX** (GPIO39,
through a 1 kΩ resistor) and **RX** (GPIO38), and MS1/MS2 then become the
UART address pins instead of microstep selects.

The three LEDs are a free bring-up aid: **S** (yellow) flickers with STEP,
**F**/**B** show DIR. In StealthChop the motor is nearly silent, so the LEDs
are how you tell "not moving" from "not being told to move".

### TMC2209 → motor (terminal block)

| TMC2209 | 17HS19-2004S1 lead |
|---|---|
| 1A | black |
| 1B | green |
| 2A | red |
| 2B | blue |

Black+green is one coil, red+blue the other. Confirm with a meter before
trusting the colours: each coil pair reads about 1.4 Ω, a coil to the other
coil reads open. If the motor buzzes and does not turn, one wire of one pair
is swapped with the other pair.

### Feather → AS5048A adapter board

| Feather silkscreen | GPIO | AS5048A board pin |
|---|---|---|
| **SCK** | 36 | CLK |
| **MO** | 35 | MOSI |
| **MI** | 37 | MISO |
| **A0** | 18 | CSn |
| **3V** | — | **3V3 and 5V — both** |
| **GND** | — | GND |
| — | — | PWM: leave open |

**Both supply pins to 3.3 V.** The datasheet (§ Connections for 5V and 3.3V,
[c:\work\docs](c:/work/docs)) says: in 3 V operation VDD3V must be shorted to
VDD5V. That also keeps MISO at 3.3 V logic; the ESP32-S3 is not 5 V tolerant.
Reserved for the azimuth encoder's CSn later: **A1** (GPIO17).

Magnet: diametrically magnetised, centred on the output shaft axis, 0.5–2.5 mm
above the chip. The firmware prints AGC and the too-close / too-far flags at
boot and in `s`; aim for AGC somewhere in the middle of 0–255.

### I2C (already wired)

Feather STEMMA QT → IMU → OLED, any order. GPIO7 powers that connector and the
firmware drives it high before touching the bus.

### TMC2209 current setting

The Adafruit board sets current with the onboard potentiometer only. There is
no VREF test point and Adafruit publishes no voltage formula: fully clockwise
is the 2 A maximum, and the scale is roughly linear from there. The motor is
rated 2 A/phase, but the 16:1 gearbox does not need that for this arm, and
the driver has no heatsink.

* Start with the pot at about **half travel (~1 A)**.
* If the arm skips or the stall trip fires under load, turn it up a little.
* Driver and motor should end up warm, not too hot to touch. Add the small
  heatsink Adafruit suggests if you go past ~1.2 A.

Holding torque stays on while the board is powered (`setAutoEnable(false)`),
so the driver runs warm even when the arm is still. That is deliberate; an arm
with a dish on it should not go limp. The TMC2209 automatically drops to a
lower standstill current after a moment, which the A4988 could not do.

Leave the **SPRD** jumper open (StealthChop). It is quiet and has plenty of
torque at the few motor-rev/s this axis uses. Close it (SpreadCycle) only if
the motor loses steps at the top speed, which shows up as the stall fault.

## Build and flash

The ESP32 core (3.3.11) and every library are installed already. Two ways:

**Arduino IDE 2.x** — open [ElevationSweep/ElevationSweep.ino](ElevationSweep/ElevationSweep.ino),
board *Adafruit Feather ESP32-S3 2MB PSRAM*, port = the Feather, Upload. The
`GearboxAxis` library shows up because of the junction in `Documents\Arduino\libraries`.

**Terminal / VS Code** — no extension needed:

```powershell
.\build.ps1              # compile
.\build.ps1 upload       # compile + flash, auto-detects the Feather's port
.\build.ps1 monitor      # 115200 baud
```

The Feather changes COM port when it drops into its bootloader
(`239A:xxxx` running → `303A:1001` bootloader). If an upload fails with
"port busy / doesn't exist", run it again with the port it is on now. If it
stays silent after flashing, press RESET once.

If you want a serial console inside VS Code, Microsoft's **Serial Monitor**
extension (`ms-vscode.vscode-serial-monitor`) is enough. The Arduino IDE's
own monitor also works. Nothing else is required.

## Talking to the board, and the reset trap

**Closing a serial session resets this board into ROM download mode, where
your firmware is not running at all.** Measured 2026-09-13, and it cost
most of a bring-up session before it was understood, because every symptom
pointed at the mechanics instead:

| USB ID | What it means |
|---|---|
| `239A:811B` | the sketch is running |
| `303A:1001` | ROM USB-Serial/JTAG, i.e. parked in download mode, nothing running |

The board also will not send a single byte until the host asserts DTR, so
the two failure modes compose into something very confusing: hold DTR and
you get data but kill the board on the way out; leave DTR low and the board
survives but says nothing. A board that has been knocked into download mode
looks exactly like a mechanical fault, because commands appear to be
accepted (they are not: nothing is listening) and the arm never moves.

Recovery is a RESET press, or:

```powershell
& "$env:LOCALAPPDATA\Arduino15\packages\esp32\tools\esptool_py\5.3.1\esptool.exe" --port COM8 run
```

Two scripts wrap all of that:

```powershell
.\talk.ps1 -Script "s|1500"              # send commands, print replies, leave it running
.\talk.ps1 -Script "r|300|m 800|8000"    # numbers are waits in ms
.\runsweep.ps1 -To 180                   # staged sweep: arm it, reboot into it, watch
.\runwave.ps1 -Speed 30                  # wave horizon to horizon at 30/45/60 deg/s
```

The consequence for firmware design is that **anything long-running must be
startable without a serial session.** That is what the one-shot boot actions
are for: `O` arms an oscillation and `W` arms a level-and-sweep, each stored
in flash, run on the next boot, and cleared immediately so a reset part way
through cannot leave the axis cycling forever. Results stay in RAM, and
opening a session to read them with `P` does not disturb the board.

## The one thing the sensors cannot work out for themselves

The encoder knows the output shaft angle exactly. The IMU knows which way is
down exactly. Neither knows **how the IMU board is bolted to the arm**, and
without that the pair cannot tell you where the horizon is.

This is not a gap that more sensing closes. Rotating about a horizontal hinge
moves the arm through a vertical circle, and gravity measurements are
identical for any two mountings that differ by a rotation in that circle's
plane. One external statement is required, exactly once, and then everything
else follows.

`E <deg>` is that statement. Park the arm where you can see its true
elevation, and say so:

```
E 0     the arm is at the horizon right now
E 90    the arm is pointing at zenith right now
```

It back-rotates the measured gravity vector about the hinge axis to derive
the level reference, and shifts the encoder zero to match, so it works at
any angle and with the board mounted any way round. Stored in flash
afterwards.

**The automatic `l` leveling is a guess, and it was wrong on this bench.**
It assumes the board sits square to the arm with one face down at level, so
it drives whichever board axis is nearest vertical to vertical. Measured
2026-09-13: with the gearbox laid on its side on the table, that position was
**zenith, not the horizon**, and the firmware happily called it 0°. The first
horizon-to-horizon sweep therefore ran from straight up to straight down.
Every number in it was correct and the frame was off by 90°.

Two changes came out of that. `l` now says out loud that it is guessing when
no reference is stored, and it trusts a stored reference over re-guessing
once one exists. Use `E` on any new mount and the guess never runs.

For the real tracker this is the same fact the prototype already handles by
asking the operator to park the antenna level once. The gearbox axis is no
different, and `E` is the seam where that answer goes in.

### What the arm's shape does and does not affect

A **bent** arm costs nothing. Rotation about the hinge turns every rigid part
of the assembly by the same angle, whatever its shape, so a dog-leg between
the shaft and the IMU is a constant offset and `E` absorbs it completely.

A **flexible** arm is different, because the gravity moment changes with
elevation, so the board's tilt and the shaft's angle differ by an amount that
varies with angle. That one does not come out in a constant, and it is a good
reason to re-run the axis fit after rebuilding the arm.

## Fitting the hinge axis from a sweep

`c` learns the hinge axis from a single small jog, which is enough to get
moving and not much more. `k` and `K` refit it against a whole sweep.

Rotation about the hinge cannot change gravity's component along the hinge, so
across a sweep the measured gravity vectors trace a circle in the plane normal
to the axis. The axis is that plane's normal, recovered by least squares from
every station at once, with the encoder supplying the angle each sample was
taken at. The level reference falls out of the same fit as the average of
every sample back-rotated to 0°, which also removes any constant bias.

```
S 180     sweep, recording gravity at each station
k         report the fit and what it would change, without adopting it
K         adopt and store it, but only if the residual actually improves
```

The sweep log lives in RAM, so `k` must be run **in the same session as the
sweep**: closing a serial session resets the board and the log goes with it.
`K` refuses a fit that is not better than what is stored.

## Measured, 2026-09-13

First full bring-up of the elevation axis, bare arm, no antenna load.

Horizon to horizon, 12 stations of 15° out to 180° and back:

| Measurement | Result |
|---|---|
| Encoder vs commanded target, worst of 24 stations | 0.15° |
| Encoder vs IMU arm angle, worst of 24 stations | 0.88° |
| Return to zero after 360° of total travel | 0.02° |
| AS5048A read errors over the whole run | 1 |
| Stall / slip trips | none |

IMU leveling converges in three iterations from 20° out, landing inside
0.2°. Magnet gap is good: AGC 58, magnitude 4636, neither too-close nor
too-far flag set.

Repeatability, six arrivals at the same angle, three from each direction:

| Measurement | Result |
|---|---|
| Encoder spread over all six arrivals | 0.20° |
| Encoder bias, approaching up vs down | 0.08° |
| IMU spread over all six arrivals | 0.29° |
| IMU bias, approaching up vs down | 0.21° |

The encoder repeats to better than its own 0.1° settle tolerance. The IMU's
0.21° direction bias is real lost motion between the output shaft and the arm
tip, since the encoder sits ahead of it and shows only 0.08°.

Hinge axis refit from a full sweep, 24 stations:

| Geometry | Hinge axis | Worst | Residual RMS |
|---|---|---|---|
| From the 4° discovery jog | −0.067, −0.104, 0.992 | 1.48° | 0.79° |
| Fitted over a sweep | −0.021, −0.034, 0.999 | 1.22° | 0.55° |

The discovery jog had the axis about 7° off the board's Z; the fit puts it
at 2.4°. Two independent sweeps produced the same fit to within 0.002 per
component, and refitting the adopted axis returns it unchanged with an
identical residual, so the geometry has converged rather than been overfitted.

What is left is a repeatable, angle-dependent ±0.5° with roughly a 90° period.
No rigid-hinge model can absorb a term like that, so it lives inside one of
the two sensors, or in flex of the improvised arm this was measured with.
Being repeatable it could be tabulated and corrected, but at half a degree it
is already well inside the beamwidth of any dish this mount will carry, so it
is recorded rather than chased. Re-run `K` after the real arm is built.

The encoder is the position truth here and it is very good. The IMU's ~0.55°
RMS is ample for what the IMU is for: an independent second opinion that
catches lost steps and a slipped belt.

### Speed headroom, bare arm

Full horizon-to-horizon legs, acceleration set equal to speed throughout, so
every tier reaches full speed in one second:

| Speed | Result |
|---|---|
| 30 °/s | clean, every leg within 0.11 s of the theoretical trapezoid time |
| 45 °/s | clean, within 0.01 s |
| 60 °/s | clean, within 0.05 s |
| 70 °/s | clean, within 0.06 s |
| 85 °/s | **belt skipped teeth** 84° into the first leg |
| 105 °/s | **belt skipped teeth** 44° in |

Matching the predicted time that closely means the motor is tracking its pulse
train exactly, with nothing lost. The ceiling is **belt tension, not motor
torque**: it is clearly audible when it goes, and the tensioner is the fix.

**A skipped belt costs nothing here but time.** After the 85 °/s trip the
encoder read 84.09° and the IMU agreed at 84.52°, so position was never in
doubt. That is the payoff for putting an absolute encoder on the output shaft
instead of counting steps from a home switch: there is no accumulated position
to lose, and no re-homing to do. The stall guard noticed, stopped, cut the
driver, and the arm held where it was.

One caveat on reading the logs: the IMU is briefly meaningless during a hard
stop, because an accelerometer cannot tell gravity from deceleration. The
raw-stream line printed at the moment of the trip showed a wild value; the
settled reading a second later agreed with the encoder to 0.4°.

Defaults in [config.h](ElevationSweep/config.h) are now 40 °/s and 25 °/s²,
which is a comfortable margin under the bare-arm ceiling. Expect that ceiling
to fall once an antenna is on the arm. Re-measure with
`.\runwave.ps1 -Speed 70` after adding load or tightening the belt.

**With the driver disabled the arm held at 20° instead of sagging**, so the
16:1 reduction is not easily backdriven. That matters for carrying a dish.

## First power-up

Before applying motor power: place the arm **roughly horizontal** (within
~45°) and make sure it can swing a full half-turn without hitting anything.

`SAFE_BOOT` in [config.h](ElevationSweep/config.h) is **true**, so the board
comes up with the driver disabled and moves nothing on its own. Leave it that
way while the mechanics are unproven: this board resets whenever a serial
session closes, and you do not want each of those resets to start a sweep.
Type `B` to run the bring-up, or arm a boot action with `O` / `W`.

Prove the mechanics before trusting any of it:

1. `t`, then turn the arm through 30° or more by hand. The encoder and the
   IMU should both move, together, by the same amount.
2. `m 800` moves the motor open loop by 5.6° at the output and prints what
   each sensor saw. If the encoder follows but the IMU does not, the arm is
   not attached to the shaft the magnet is on.
3. `.\runsweep.ps1` once both of those look right.

The old automatic sequence still exists behind `B`:

1. Flash, open the monitor. Boot prints one line per device; all four should say ok.
2. **Discovery** (first boot only): the arm jogs +4° open-loop, the firmware
   reads how the encoder and IMU responded, and jogs back. It learns the
   encoder sign and the IMU hinge axis.
3. **Level**: using the IMU it drives the arm to the horizon in a few small
   moves and sets that as 0°. This assumes the IMU board is mounted square
   to the arm (flat, on edge, or on end all work; yours currently reads
   gravity on its X axis, so it is on edge) and that the arm starts within
   45° of level. If the board is mounted at an odd angle, jog by hand (`j`)
   and press `z` at level instead.
4. **Up question** (first boot only): the arm moves to +10° and the OLED and
   serial ask *did the tip go UP?* Answer `y` or `n`. `n` flips motor, encoder
   and IMU signs together and returns to 0. Stored forever after.
5. **Sweep**: ramps 0 → 180°, dwells 3 s, ramps back to 0, then idles.

Every later boot loads the stored calibration, moves to 0 if it is not there,
and sweeps. `F` wipes the calibration and reboots.

Motion is ramped: default 15°/s max, 8°/s². Change live with `v` and `a`.

### Serial commands

```
h        help                  s        status (encoder, magnet AGC, IMU, stepper, cal)
r        raw stream 10 Hz      P        print the last staged sweep's readings
g <deg>  go to                 j <deg>  jog by
v <d/s>  max speed             a <d/s2> accel
x        stop, keep holding    X        e-stop, driver off
e / d    driver on / off       z        set 0 here
l        re-level with IMU     E <deg>  "the arm is at this elevation NOW"
B        run the whole bring-up
w        quick sweep           S <deg>  staged sweep now, logging enc vs imu
o <deg>  oscillate until 'x'   O <deg>  oscillate on the NEXT boot
W <deg>  level + staged sweep on the NEXT boot
V <d/s>  wave horizon to horizon on the NEXT boot, speeding up each tier
t        hand test: driver off, turn the arm by hand, watch enc vs imu
m <n>    open-loop move of n microsteps, no encoder, no limits
c        re-run discovery      u        flip 'up'
y / n    which way is up       !  clear fault     F  wipe cal + reboot
```

### Diagnostics worth knowing

`t` is the test that proves the sensing chain without involving the motor:
driver off, turn the arm by hand, and both the encoder and the IMU should
move together by the same amount. It names which one failed if they do not.

`m` drives the motor open loop, ignoring the encoder and the soft limits, so
it works before any calibration exists. 3200 microsteps is one motor
revolution, which is 22.5° at the output.

`S` and `W` sweep in stations instead of one long slew, pausing at each to
let the IMU settle and recording what both sensors say. The per-station
numbers are the linearity data the real tracker wants, and a shorter leg
reaches a lower peak speed, so anything the arm fouls is met gently.

### Safety built in

* Soft limits −5…185° on every move request.
* Ramped speed on every move, including corrections.
* Stall / slip / collision trip: if the encoder and the step count disagree by
  more than 12° mid-move, the motor is stopped and disabled and the OLED says
  FAULT. `!` clears it.
* `x` is checked even while a blocking calibration move is in progress.

## Using it as a library later

`GearboxAxis` is one axis: give it pins and mechanics, call `begin(engine)`,
then `moveTo(deg)` / `jog(deg)` and pump `update()` from `loop()`. It keeps
the encoder as truth, plans every move as a delta from the measured position,
and settles the residual. Two axes are two instances on one
`FastAccelStepperEngine` sharing the SPI bus with separate CS pins. `ArmIMU`
is independent and can stay on the elevation arm in the final tracker as the
second opinion — its `armAngleDeg()` compared with the encoder is the
lost-steps / slipped-belt detector.
