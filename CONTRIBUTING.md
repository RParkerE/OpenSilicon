# Contributing to OpenSilicon

OpenSilicon is an early-stage, technically demanding project focused on building a functional semiconductor fabrication pipeline in a home lab environment. Contributions are welcome, but it is important to understand the expectations and constraints before getting involved.

---

## Project Reality

This is not a typical software-only open source project.

Work spans:

* Optics and imaging (FPM / R-FPM)
* Embedded systems and firmware (ESP32-class devices)
* Precision motion systems (lithography positioning)
* Materials and process engineering (oxidation, deposition)
* Experimental validation and metrology

Most contributions require some technical foundation. If you do not have that yet, you are still welcome, but expect to spend time learning before contributing directly.

---

## Ways to Contribute

### 1. Software and Algorithms

* FPM reconstruction and phase retrieval
* Image processing and calibration pipelines
* Control systems and system integration
* Tooling for data collection and analysis

### 2. Firmware and Embedded Systems

* Motor control and ESC integration (spin coater)
* Motion control for lithography systems
* Sensor integration and feedback systems
* Communication layers (UART, SPI, I2C, WebSocket)

### 3. Hardware and Mechanical

* CAD design (STEP / STL)
* Optical alignment systems
* Motion stages and positioning systems
* Thermal system design (furnace, deposition)

### 4. Process and Materials

* Resist handling and coating methods
* Oxidation process development
* Thin film deposition (PVD)
* Process repeatability and characterization

### 5. Documentation and Validation

* Build guides
* Experimental results
* Failure modes and debugging notes
* Process documentation

---

## Where to Start

1. Read the README and understand the current system state
2. Review open issues and discussions
3. Join the [Discord](https://discord.gg/8qV6ZM93C) and observe ongoing work
4. Pick a single area and go deep

Do not try to understand everything at once.

---

## Current Priority Areas

* FPM / R-FPM reconstruction and characterization
* Lithography positioning system (voice coil + alignment)
* Spin coater control issues (ESC and RPM feedback)
* System-level integration and measurement validation

If you are unsure where to start, these are the best entry points.

---

## Contribution Workflow

1. Open an issue or discussion before starting major work
2. Clearly describe what you plan to do
3. Keep changes scoped and focused
4. Submit a pull request early for feedback
5. Iterate based on discussion

This project benefits from visibility and discussion, not isolated work.

---

## Expectations

* Be precise. Avoid vague claims or assumptions
* Document what you do, including failures
* Include data, measurements, and context where possible
* Keep discussions technical and focused
* Do not overstate results

Good contributions are reproducible and well-documented.

---

## What Not to Do

* Do not submit large, unreviewed changes
* Do not treat this as a purely theoretical exercise
* Do not ignore safety constraints
* Do not assume production-level performance

This is experimental work. Ground everything in reality.

---

## Safety

This project involves real hazards:

* High temperatures (1000°C+)
* High voltage systems
* UV exposure (lithography)
* Chemicals and materials processing

You are responsible for safe operation. If you are unsure, stop and ask.

---

## Licensing

This project is licensed under GPLv3.

By contributing, you agree that:

* Your contributions will be licensed under GPLv3
* Source code must remain available
* Modifications must be documented

---

## Final Notes

OpenSilicon is being built in the open, but it is not simplified.

If you are willing to:

* Learn the fundamentals
* Work carefully and methodically
* Share real results

Then you will be able to contribute meaningfully.

If not, you are still welcome to follow along and learn as the project develops.
