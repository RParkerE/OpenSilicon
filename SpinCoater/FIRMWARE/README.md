# OpenSilicon Spin Coater

Open-source firmware for a benchtop **spin coater** built around an ESP32. It
aspirates and dispenses photoresist onto a wafer, sets the dispense height with a
TMC2209-driven Z axis, and spins the wafer at a target RPM with a brushless
motor + ESC to spread the resist into a uniform film.

The machine is configured and operated from a built-in Wi-Fi web UI, with a
local TFT showing live status.

![Wiring overview](docs/wiring_diagram.png)

## Hardware at a glance

| Subsystem            | Part                                   | Role                              |
| -------------------- | -------------------------------------- | --------------------------------- |
| Controller           | ESP32 DevKit (`esp32dev`)              | Runs everything                   |
| Spindle              | AM32 ESC + brushless motor             | Spins the wafer at target RPM     |
| Dispense height (Z)  | TMC2209 + NEMA17 stepper               | Raises/lowers the dispense head   |
| Resist pump          | 28BYJ-48 stepper + ULN2003 driver      | Aspirates / dispenses resist      |
| Display              | ILI9341 320x240 SPI TFT                | Local status screen               |
| Home reference       | Z limit switch                         | Z homing                          |

See [docs/HARDWARE.md](docs/HARDWARE.md) for the full bill of materials and the
software toolchain, [docs/WIRING.md](docs/WIRING.md) for the circuit and pin map,
and [docs/ESC_SETUP.md](docs/ESC_SETUP.md) for configuring the ESC.

## Quick start

1. Install [PlatformIO](https://platformio.org/) (the VS Code extension or the
   `pio` CLI).
2. Wire the hardware per [docs/WIRING.md](docs/WIRING.md) and configure the ESC
   per [docs/ESC_SETUP.md](docs/ESC_SETUP.md).
3. Flash the firmware and the web assets:

   ```bash
   pio run -t upload      # firmware
   pio run -t uploadfs    # web UI (data/ -> LittleFS)
   ```

4. Power on. The ESP32 starts a Wi-Fi access point:

   - **SSID:** `OpenSiliconSC`
   - **Password:** `coater123`
   - **URL:** `http://192.168.4.1`

5. Open the URL, build a recipe, **Load to Device**, then **Start**.

## How a recipe runs

Each recipe is an ordered list of steps. For every step the
`ProcessOrchestrator` sequences the hardware:

```
Prime (aspirate) -> lower Z to dispense height -> dispense resist
  -> raise Z to spin height -> ramp spindle to RPM -> hold -> ramp down -> next step
```

Per-step parameters (set in the web UI):

| Field            | Meaning                                            |
| ---------------- | -------------------------------------------------- |
| Prime (mL)       | Resist to aspirate before dispensing (0 = skip)    |
| Dispense (mL)    | Volume dispensed onto the wafer                    |
| Disp H (mm)      | Z height while dispensing (head lowered)           |
| Spin H (mm)      | Z height while spinning (head retracted)           |
| Speed (RPM)      | Target spindle speed                               |
| Ramp (s)         | Time to spin up to / down from speed               |
| Hold (s)         | Time held at speed                                 |

## Architecture

```
WebSocket / TFT
      |
StateMachine  <—events—  ProcessOrchestrator
      |                        |
      |                   Controllers (Spin / Pump / ZAxis)
      |                        |
      +———————————————————> HAL (EscSpinMotor, TMC2209ZAxis, Pump28BYJ48, TFTDisplayHAL)
```

- **HAL** (`include/hal`, `src/hal`) — thin hardware drivers behind interfaces.
- **Controllers** (`include/controllers`, `src/controllers`) — wrap the HAL,
  service motion, and emit completion events.
- **ProcessOrchestrator** (`src/services/ProcessOrchestrator.cpp`) — the recipe
  state machine; the only place that knows the dispense/spin sequence.
- **StateMachine** — owns the machine state (Idle / Running / Paused / Fault).
- **Services** — `TelemetryService` builds a single snapshot; `DisplayService`
  renders it. Neither touches hardware directly.

## Calibration

Two constants in `include/common/configs.h` must be tuned to your mechanics:

- `Z_STEPS_PER_MM` — steps per mm of head travel.
- `PUMP_STEPS_PER_ML` — pump steps per mL (dispense a known volume and adjust).

## Safety

This drives a high-speed brushless motor and dispenses chemicals. Always run the
spindle inside an enclosure, keep the prop balanced, bench-test the ESC with the
motor **unloaded** first, and handle photoresist/solvents with appropriate PPE
and ventilation.
