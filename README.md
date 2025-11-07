# OpenSilicon

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Status: Alpha](https://img.shields.io/badge/Status-Alpha-orange.svg)](#status)
[![Contributors](https://img.shields.io/badge/Contributors-Welcome-brightgreen)](CONTRIBUTING.md)

Open-source semiconductor fabrication for the home lab. Build and operate IC fabrication equipment with detailed guides, CAD files, firmware, and process recipes.

## About

OpenSilicon brings semiconductor fabrication capabilities into the home lab. This project provides comprehensive documentation, designs, and software for building functional IC fabrication equipment for research, education, and experimentation.

We are currently developing the alpha stage of the fabrication pipeline, targeting 50-100 μm feature resolution across the complete process flow: spin coating, UV exposure, oxidation, and physical vapor deposition (PVD).

## Status

🔄 **Alpha Stage - Active Development**

This is an early-stage project. All equipment designs and processes are works in progress and subject to change. Current development focus:

- Spin coating system development
    - First prototype completed
    - TODO: Dial in steps per mm and uL in positioning and dispensing respectfully
    - TODO: Integrate TFT dispaly
    - TODO: Add homing switch and sequence for positioning
    - TODO: Add a resist fill method
- UV exposure system optimization
- Oxidation furnace fabrication pipeline
- PVD deposition system integration
- Process characterization and documentation

Join us to help shape the project during these formative stages.

## Getting Started

### Prerequisites

- Basic electronics and mechanical assembly experience
- Interest in semiconductor fabrication and experimental work
- Familiarity with working safely with high temperatures and electrical systems
- 3D printer access (optional, for custom components)

### Next Steps

1. **Review the documentation** to understand the current state of each component
3. **Check open issues** to see what's being worked on
4. **Start contributing** - Documentation, testing, and feedback are valuable at this stage

## Equipment in Development

| Equipment | Status | Notes |
|-----------|--------|-------|
| Spin Coater | 🔄 In Progress | Finalizing initial build phase |
| UV Exposure System | 🔄 In Progress | Initial build phase |
| Oxidation Furnace | 🔄 In Progress | Initial build phase |
| PVD Deposition System | 🔄 In Progress | Initial build phase |

## License

OpenSilicon is licensed under the **GNU General Public License v3**. See [LICENSE](LICENSE) for details.

This project integrates components licensed under GPLv3. When using or modifying this project, you must:

- Provide source code to users
- License derivative works under GPLv3
- Include a copy of the license
- State significant changes made

## Contributing

We actively welcome contributions. This project is in its early stages, and input from the community is essential.

### Getting Started with Contributing

1. Check [open issues](../../issues) and [discussions](../../discussions)
2. Reach out with ideas or questions before starting major work
3. Submit pull requests for feedback and discussion

### Ways to Contribute

- **Share feedback**: Comment on design decisions and approaches
- **Document your work**: Help build the knowledge base
- **Report issues**: Share problems, observations, and learnings
- **Suggest improvements**: Propose design changes or optimizations
- **Contribute code**: Firmware, software, and design improvements
- **Test and validate**: Help characterize equipment performance

## Community

- **GitHub Discussions**: [Ask questions and discuss ideas](../../discussions)
- **Issues**: [Report bugs and suggest features](../../issues)
- **Email**: [parker.ellwanger@gmail.com](mailto:parker.ellwanger@gmail.com)

## Roadmap

- [ ] Complete alpha stage (50-100 μm) for all equipment
- [ ] Finalize process procedures and characterization data
- [ ] Publish comprehensive build guides
- [ ] Establish baseline performance metrics
- [ ] Look at improvements to UV exposure system to allow for better mask alignment and etching (5-10 μm)

## Performance & Limitations

OpenSilicon is targeting:

- **Feature resolution**: 50-100 μm (alpha stage)
- **Use case**: Research, education, and experimentation
- **Scale**: Small-batch processing
- **Goal**: Accessible home lab fabrication, not production

## Safety

⚠️ **This project involves significant hazards.** Home semiconductor fabrication includes:

- High temperature operation (up to 1000°C+)
- High voltage electrical systems
- UV radiation exposure
- Chemical hazards

Users are responsible for understanding and mitigating these risks. Always follow safety procedures and never operate equipment unsupervised. 

## Building from Source

All designs are provided in open formats. No proprietary software is required:

- **CAD Files**: STEP and STL formats
- **Firmware**: Open-source microcontroller code
- **Software**: Python and standard development tools

## Acknowledgments

OpenSilicon builds on research and contributions from the open-source hardware community and makers exploring accessible semiconductor fabrication.

## Disclaimer

Use of OpenSilicon designs and instructions is at your own risk. The authors assume no liability for injuries, property damage, or other consequences from assembly, operation, or misuse of equipment built from these designs.

## Support

If you find OpenSilicon interesting:

- Star this repository ⭐
- Share your interests and expertise
- Contribute to development
- Help improve documentation

---

**Have questions?** [Start a discussion](../../discussions) or [email us](mailto:parker.ellwanger@gmail.com).

**Found an issue?** [Report it](../../issues/new).
