
# Intro

PID tuning lab to practice process control with several different plant process dynamics. My main objective at first was as a training tool but I would also like to extend to preliminary studies of new process dynamics or quick tests.

### Disclaimer 
This is clearly all Claude-coded as I needed to have something quick to practice with. I will be spot-checking to make sure the process dynamics are accurate.

# Loop 101 — a PID tuning bench

Two builds, generated from the same source, so they cannot drift apart:

- **`pid-lab.html`** — one static file, ~37 kB. Open it locally or drop it on any web host.
  No server, no build step, no CDN, no network of any kind. Everything below applies except
  the OpenPLC bridge, which needs a runtime that can speak Modbus TCP.
- **`pid-lab-flow.json`** — the Node-RED flow, for running on the Pi alongside OpenPLC.

The plant model, the controller, and the process library are byte-identical between them —
the static build emits the same six function bodies and serves `/state`, `/cmd` and `/plants`
locally instead of over HTTP.

A simulated plant, an honest ISA-form PID, and a strip chart, running entirely on core
Node-RED nodes. No palette installs, no dashboard dependency, no CDN. Open a browser at
`/pidlab` and start bumping the loop.

The second file hands the controller over to OpenPLC across Modbus TCP, so you can tune
against a real PLC scan cycle instead of a JavaScript function.

---

## Install — static

Open `pid-lab.html`. That's it. To publish it, copy the single file anywhere that serves
static content.

The simulation runs on a 200 ms browser timer, one scan per tick, with the sim clock
advancing by `cfg.ts`. A background tab pauses rather than fast-forwarding, which is the
same as unplugging the trend pen — nothing is lost, it just stops.

## Install — Node-RED

1. **Delete any earlier PID Lab tab first** (tab menu → Delete), then Deploy. Two copies both
   register `GET /pidlab`, the older one answers first, and you get the old page with no
   plant dropdown. The current tab is labelled **PID Lab v2** so duplicates are obvious.
2. Menu (top right) → **Import** → **select a file to import** → `pid-lab-flow.json`
3. Import as **new flow**. Deploy.
4. Browse to `http://<your-pi>:1880/pidlab` and hard-refresh (Ctrl/Cmd-Shift-R).

The header shows `v2, 6 plants` once the library loads. If it says `plant library FAILED`,
open `http://<your-pi>:1880/pidlab/plants` directly — a 404 means an older flow is still
deployed and answering first.

The loop starts in **manual at 30 % output**, which is where you'd actually start on a
live plant. Bump it, watch the response, identify the plant, then go to auto.

To change the scan period, edit `ts` in the **init context** node *and* the repeat rate on
the **scan 200 ms** inject. They have to match — nothing enforces it.

---

## What the simulation actually does

### The loop, laid out as a loop

The lower half of the page is the control loop drawn left to right:

```
  ┌─ Process ──┐  PV  ┌─ Controller ─┐  OP  ┌─ Final control element ─┐
  │  identity  │ ───► │  SP, Kc, Ti  │ ───► │  MV name, slew, limits  │
  │  dynamics  │      │  Td, bias    │      │  actuator position      │
  └────────────┘      └──────────────┘      └────────────┬────────────┘
         ▲                                               │
         └──────────── actuator position ────────────────┘
```

The connectors carry live values, so you can watch a number leave the controller as OP and
arrive at the process as a different number once the actuator has rate-limited it.

**Change process…** opens a picker grid. Each tile carries a glyph, the plant's role in one
phrase, its dynamics, and a difficulty rating in pips. Difficulty is *derived*, not stored:
dead-time dominance (via the half rule), integrating behaviour, inverse response, a resonant
mode, and a rate-limited actuator each add to it. A plant you add yourself gets scored the
same way. Current spread: Bench 1, PV volt/VAR 3, superheat 3, wind 4, drum level 5.

The **final control element** card is where the abstraction ends and hardware starts. It
names the manipulated variable and the device actually being driven, and it holds the things
that belong to the actuator rather than the process: slew limit and output travel limits. It
warns you when the output is on a limit, or when the actuator is falling behind the command —
which is exactly what a too-aggressive tune on a rate-limited drive looks like.

The definitions live in the **process library** function node, not in the web page — see
[Adding your own plant](#adding-your-own-plant).

| Preset | What it is | What it teaches |
|---|---|---|
| Bench FOPDT | generic self-regulating loop | the mechanics, with nothing exotic in the way |
| PV plant volt/VAR at the POI | plant controller trimming inverter VARs to hold POI voltage | dead-time dominance: θ/τ ≈ 2.5, so there is very little gain available |
| PV volt/VAR, weak grid | same plant, weak interconnection | process gain *is* grid strength — your gain margin is your grid-strength margin |
| Boiler drum level | integrating with shrink and swell | inverse response, and why reacting to the first few seconds is exactly wrong |
| Boiler superheat steam temp | long lag, long dead time, second thermal lag | the half rule, and why real units cascade |
| Wind turbine collective pitch | fast speed loop, rate-limited actuator, tower mode | you don't lose the speed loop, you excite the structure |

Slow plants run compressed. Superheat uses a 5 s scan, so the trend advances 25× faster
than real life — a 90 s time constant is otherwise unusable to tune by hand. The panel
tells you the current factor. The wind preset runs the other way, at 2× slow motion.

**Plant, self-regulating mode.** First order plus dead time, discretised with an exact
zero-order hold, so the response is right regardless of scan rate:

```
PV[k] = a·PV[k-1] + (1-a)·(K·u[k-θ] + bias),   a = exp(-Ts/τ)
```

**Plant, integrating mode.** `dPV/dt = Ki·(u - 50)`. Half output holds level, anything else
ramps. This is the mode where P-only leaves offset and where high gain will walk you into
a limit cycle.

**Inverse response** (`lead`, seconds). Adds a right-half-plane zero, so the PV moves the
wrong way first and crosses back through its starting value at roughly `lead` seconds. This
is drum level shrink and swell. It behaves like extra dead time and the tuning panel counts
it as such.

**Second-order stage** (`tau2`, `zeta2`). At ζ₂ = 1 it's a second thermal lag. Below about
0.3 it's a structural mode — tower fore-aft, drive train torsion — sitting inside the loop.
Discretised with an exact zero-order hold rather than Euler, because Euler gets the ring
frequency wrong by a third at these step sizes, and where the mode sits is the whole point.

**Actuator slew limit** (`rate`, %/s). Pitch drives, large valves, tap changers. A rate
limit turns an otherwise stable loop into a limit cycle if you push gain, and no amount of
tuning arithmetic predicts it.

**Controller.** ISA (dependent) form, `Ti` and `Td` in **seconds**:

- Derivative acts on the measurement, not the error, so a setpoint step produces no kick.
- Derivative is filtered at `Tf = Td/N`, `N = 10`.
- Anti-windup by back-calculation: when the output clamps, the integral is rewritten to
  match the clamped value rather than being frozen.
- Manual mode tracks, so manual→auto is bumpless.
- Separate **bias** (manual reset) alongside the integral (automatic reset). See below.
- Load disturbance enters at the valve, the same place the output does — which is why a
  load change and a setpoint change do *not* look the same on the trend.

### Bias, and the equation panel

The controller panel shows the output equation live: the general form, the same thing with
your current constants substituted, and a running breakdown of what each term is actually
contributing, with a bar showing the split. If a loop is misbehaving, that bar usually tells
you which term is responsible before the trend does.

`OP = Kc·e + (Kc/Ti)∫e dt − Kc·Td·dPVf/dt + b`

The integral is *automatic* reset; `b` is *manual* reset. They interact in a way worth
understanding, because it trips people up:

- **With integral action (Ti > 0), bias does nothing lasting.** Change it and the output
  bumps, then the integral absorbs the change within a few Ti and the loop returns to exactly
  the same steady state. Verified: bias 0 → 25 leaves PV at 50.00 and shifts I by −25.0.
- **With Ti = 0, bias is the only thing setting where the output sits.** This is manual reset
  in the original sense, and it's how you remove proportional offset without integral action.
  On the drum-level plant with a 10 % load change, `PV = 10 + bias` exactly, so bias = 40
  puts it dead on setpoint.

To keep the displayed equation honest, setting Ti = 0 folds whatever the integral was holding
into bias and zeroes the integral. The transfer is bumpless, and `P + I + D + b` always sums
to OP exactly — the panel isn't a re-derivation, it's the actual terms.

Many PLC blocks express integral as **repeats per minute** or `Ti` in **minutes**. If you
port tuning constants out of here, check the units before you trust them.

### Verified behaviour

Run against the flow's own function nodes:

| Check | Result |
|---|---|
| Bump test identification | recovers K=1.60, θ≈3.2 s, τ≈11.8 s from a true 1.6 / 3 / 12 plant |
| Derivative kick on SP step (Td=4) | none — output ramps 50.22, 50.70, 51.18 |
| Manual → auto transfer | 35.00 → 34.86, bumpless |
| Anti-windup | SP driven past reachable PV, returns without a wind-down tail |
| Ultimate gain / period, P-only search | Ku ≈ 4.10, Pu ≈ 11.4 s vs analytic 4.34 / 11.0 s for that FOPDT |
| Second-order stage, free ring | 0.288 Hz and ζ = 0.080 measured against 0.288 / 0.08 configured; 16.3 % overshoot at ζ = 0.5 against 16.3 % from theory |
| Slew limit | peak 6.00 %/s against a 6 %/s limit; 10.0 s to cross a 60 % step |
| Drum level inverse response | dips 12.4 % on a 15 % feedwater step, crosses back at 8.5 s against `lead` 6 s plus 2 s dead time |
| Bias with integral running | steady state unchanged (50.00 → 50.00); integral absorbs it exactly (I shifts −25.0) |
| Bias with Ti = 0 | `PV = 10 + bias` to within 0.2 % across bias 30–60; integral stays at zero |
| Bumpless transfer, bias set | 35.00 → 34.90 at both bias 0 and bias 30 |
| Term arithmetic | `P + I + D + b` equals OP to 1e-6, and `P = Kc·e` to 1e-9 |
| Static build | same regression run against `pid-lab.html` in a fake DOM: identical open-loop step (63.99), slew cap (6.00 %/s), inverse response (dips to 37.6, crosses at 9 s), bias offset removal, and term arithmetic |
| Deploy chain | firing the seed inject and following the wires leaves `cfg`, `st`, `hist` and `plants` all set |
| Weak-grid retune | strong-grid IMC tune hunts at 6.8 % on the weak grid; retuning removes it |

The gap in that last row is the half-scan of zero-order-hold lag, which is real and would
be there on a PLC too. The loop can also be driven genuinely unstable, and the instability
converges as scan rate increases (9.667 → 9.357 % peak-to-peak from Ts = 0.2 s to 0.02 s),
so it's dynamics rather than a numerical artifact.

---

## Adding your own plant

Open the **process library** node and add an entry. Every field is optional — anything you
leave out keeps its current value.

```js
my_loop: {
  name: 'Cooling tower basin level',
  note: 'Shows in the panel when this plant is selected.',
  ts: 0.5, window: 600,       // scan period, s;  trend width, s
  model: 'integrating',       // or 'fopdt'
  Ki: 0.2,                    // integrating ramp rate, %PV/s per %OP
  theta: 4,                   // dead time, s
  lead: 0,                    // inverse response, s
  tau2: 0, zeta2: 1,          // second-order stage
  rate: 0,                    // actuator slew limit, %/s
  pv0: 50,                    // where PV sits at the starting output
  noise: 0.5,
  opMan: 50, sp: 50,

  // shown on the tile and in the chain cards
  icon: 'drum',                       // hx | sun | sunweak | drum | coil | wind | generic
  role: 'Integrating, no self-regulation',
  pv:  'Basin level, % of span',      // labels the Process -> Controller connector
  mv:  'Makeup valve position, %',    // headline of the final control element card
  fce: 'Butterfly valve, electric actuator',
  fceNote: 'One line on how the device actually behaves.'
}
```

Deploy and it appears in the dropdown. Selecting a plant always resets the loop to manual
with the plant settled at `pv0`, so you start every session the way you'd start on site.

Time compression is `ts / 0.2`. If a plant's time constant is in minutes, raise `ts` rather
than sitting and watching — the controller is genuinely executing at that scan period, which
for a slow thermal loop is realistic anyway.

## Exercises

Work these in order. Each one has a failure mode you will meet in the field.

**1. Bump test and identification.** Manual, let it settle, press *Bump test +15*. From the
trend alone, read off dead time (flat spot before anything moves), gain (ΔPV/ΔOP), and time
constant (63 % of the way to the new value, measured from the end of the dead time). Check
yourself against the plant sliders. Do it again with noise at 1.0 % and see how much harder
θ gets to read — that's the real-world case.

**2. IMC vs hand tuning.** Apply *IMC PI* from the rules table, step the setpoint, note the
IAE. Now beat it by hand. On the default plant IMC PI gives IAE ≈ 195 with about 10 %
overshoot; anything under that without slamming the valve is a good tune.

**3. Why Ziegler-Nichols is a starting point, not an answer.** Apply *Z-N PI* and step the
setpoint. Expect roughly 47 % overshoot and a saturated valve. It's a quarter-amplitude-decay
rule; almost nobody wants quarter-amplitude decay on real equipment.

**4. Setpoint step vs load step.** Tune for a clean setpoint response, then press
*Bump load +10* without changing anything. A tune optimised for setpoint tracking usually
handles load rejection badly. This is the single most common tuning mistake, because people
tune with the setpoint knob and then the plant lives on load disturbances.

**5. Dead time dominance.** Set θ = 12, τ = 5. Watch the achievable gain collapse. There is
no tuning that fixes this — the answer is to attack the dead time in the process, or go to a
Smith predictor or MPC.

**6. Level control.** Switch to integrating, set Ti = 0. Bump the load and watch the
proportional offset park the level away from setpoint. Add integral and watch it return, but
slowly, and with a cycle if you push Kc. Level loops usually want *averaging* control — low
gain, long Ti, let the vessel absorb the swing — not tight control.

**7. Derivative and noise.** Set Td = 5 with noise at 0, then 0.5 %. Valve travel over the
same window goes from about 1256 % to 3345 % of stroke. That is packing on your positioner
for no control benefit. Then push Td past θ/2 and watch the loop ring and go unstable —
with the default plant, Td = 5 against θ = 3 is enough.

**8. Bias and manual reset.** Set Ti = 0 on the drum-level plant and put it in auto. Bump the
load and watch proportional offset appear. Now walk `bias` until the offset is gone — that's
manual reset, and it's what everyone did before integral action was cheap. Then set Ti back
to 20 and change bias again: this time the offset comes straight back out on its own, and the
bias bar in the equation panel goes quiet while the I bar grows. That contrast is the clearest
way to see what integral action is actually for.

**9. Windup.** Set the setpoint above what the valve can physically reach and leave it there
a while, then bring it back. Then set `N = 50` in the init node and re-run exercise 7 to see
how much of derivative's usability is really the filter.

### Per-plant exercises

**PV volt/VAR.** Bump test it. Note θ/τ ≈ 2.5 and how little gain IMC will give you
(Kc ≈ 0.19). Tune it until you're happy on the strong grid. Now switch to the weak-grid
preset *without changing the tuning* — the same loop hunts at about 7 %. Retune, and note
you've given up roughly two thirds of your gain. This is the argument for tuning plant
controllers against the weakest expected system condition rather than the one you happened
to commission on.

**Drum level.** Put it in manual and bump feedwater. Watch the level go *down* for six
seconds. Now try to tune it like a normal integrating loop and watch a Kc of 2 walk the unit
into the wall. Then compare SIMC against the averaging row in the rules table: averaging
gives you a bigger level swing and a much smaller feedwater swing. On a real unit the
downstream boiler cares far more about steady feedwater than about level sitting exactly on
setpoint, which is why level loops are detuned on purpose.

**Superheat.** Tune it once ignoring the second lag, then again using the half-rule values
the panel shows. Overshoot goes from about 90 % to about 45 %. Then notice that even the good
tune takes several minutes of process time to settle, which is the honest argument for
cascade rather than better PID constants.

**Wind pitch.** Start at Kc 0.4 and work up. Around 0.8 you have a good tune. At 1.2 the
loop starts a sustained 0.26 Hz oscillation that no longer decays — that's the tower, not
the speed loop, and tightening the controller makes it worse. Watch the OP trace hit its
6 %/s slew limit on the way in. This is the one where the tuning arithmetic in the panel is
actively misleading, because none of those rules know the structure is there.

---

## Handing the loop to OpenPLC

`pid-lab-openplc-bridge.json` replaces the internal controller with your PLC, so PV goes
out over Modbus TCP and OP comes back — with real comms latency and real 16-bit register
quantisation in the path.

**Import.** Open the **PID Lab** tab first, then Import → paste → choose **current flow**
(these nodes share the tab's flow context). Needs `node-red-contrib-modbus`.

Keep the plant on `ts = 0.2` when the bridge is driving. Process time advances one `ts` per
200 ms tick, so a compressed preset would run the simulation faster than Modbus can answer.

**Enable.** `curl -X POST localhost:1880/pidlab/cmd -H 'Content-Type: application/json' -d '{"extPID":true}'`
The plant, trend, and scoring keep running; only the controller changes hands.

**Registers.** OpenPLC v3 maps `%MW0..%MW1023` to holding registers **1024..2047**, and
`%QW0..%QW1023` to holding registers **0..1023**. The bridge defaults to `%MW0` for PV and
`%MW1` for OP. There are reports of `%MW` not reading back correctly inside the runtime on
some builds where `%QW` worked — if you hit that, change `PV_REG`/`OP_REG` to 0 and 1 at
the top of the two function nodes and relocate your PLC variables. Newer OpenPLC Edge
builds let you configure the buffer mapping directly, so check your version's address page
rather than trusting these numbers.

Values are carried as 0–10000 for 0.00–100.00 %, giving 0.01 % resolution.

**Controller.** Structured Text equivalent of the JS block. Call it from a task at 200 ms
to match `TS`:

```iecst
FUNCTION_BLOCK PID_ISA
VAR_INPUT
  PV      : REAL;
  SP      : REAL;
  KC      : REAL := 1.0;
  TI      : REAL := 20.0;   (* seconds; 0 disables integral *)
  TD      : REAL := 0.0;    (* seconds *)
  TS      : REAL := 0.2;    (* must equal the task interval *)
  AUTO    : BOOL := TRUE;
  OP_MAN  : REAL := 30.0;
  OP_MIN  : REAL := 0.0;
  OP_MAX  : REAL := 100.0;
END_VAR
VAR_OUTPUT
  OP : REAL;
END_VAR
VAR
  I_TERM, PVF, PVF_PREV : REAL;
  E, P_TERM, D_TERM, OP_RAW, TF : REAL;
  N : REAL := 10.0;
  FIRST : BOOL := TRUE;
END_VAR

IF FIRST THEN
  PVF := PV; PVF_PREV := PV; I_TERM := OP_MAN; FIRST := FALSE;
END_IF;

E := SP - PV;                          (* reverse acting *)

IF TD > 0.0 THEN TF := TD / N; ELSE TF := TS; END_IF;
PVF := PVF + (TS / (TF + TS)) * (PV - PVF);
IF TD > 0.0 THEN
  D_TERM := -1.0 * KC * TD * (PVF - PVF_PREV) / TS;   (* D on PV, no kick *)
ELSE
  D_TERM := 0.0;
END_IF;
PVF_PREV := PVF;

P_TERM := KC * E;

IF AUTO AND (TI > 0.0) THEN
  I_TERM := I_TERM + KC * (TS / TI) * E;
END_IF;

OP_RAW := P_TERM + I_TERM + D_TERM;

IF NOT AUTO THEN
  I_TERM := OP_MAN - P_TERM - D_TERM;                 (* track for bumpless *)
  OP := OP_MAN;
ELSIF OP_RAW > OP_MAX THEN
  OP := OP_MAX;  I_TERM := OP_MAX - P_TERM - D_TERM;  (* back-calculation *)
ELSIF OP_RAW < OP_MIN THEN
  OP := OP_MIN;  I_TERM := OP_MIN - P_TERM - D_TERM;
ELSE
  OP := OP_RAW;
END_IF;
END_FUNCTION_BLOCK
```

Wrap it in a program that reads PV from `%MW0` (divide by 100), calls the block, and writes
`OP * 100` to `%MW1`.

---

## HTTP interface

| Endpoint | Purpose |
|---|---|
| `GET /pidlab` | the page |
| `GET /pidlab/state?since=<ms>` | config, metrics, and trend points newer than `since` |
| `GET /pidlab/plants` | the process library as JSON |
| `POST /pidlab/cmd` | `{"preset":"boiler_drum"}`, `{"sp":60}`, `{"Kc":1.4,"Ti":12,"Td":0}`, `{"auto":true}`, `{"dist":10}`, `{"reset":true}`, `{"extPID":true}` |

Anything you can set in the UI you can set from a script, so you can drive repeatable test
sequences or log runs into TimescaleDB alongside the rest of your instrumentation.

## What the static build gives up

Only the OpenPLC bridge. A browser cannot open a Modbus TCP socket, so if you want the PLC
in the loop you need the Node-RED build (or a small WebSocket-to-Modbus relay, which puts a
server back in the picture and defeats the point).

Everything else is intact, and a few things are better: no polling latency, no Pi required,
and it works offline.

## Worth adding later

Cascade — an inner flow loop inside an outer temperature loop — is the obvious next
structure, and it's the one that matters for tracker and BESS work where a fast inner
current loop sits under a slow outer voltage or power loop. It needs a second plant and a
second controller instance; the context layout here already supports it.
