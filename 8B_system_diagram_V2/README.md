# Fliptable System Diagrams V2 - Enhanced Documentation

## Overview

This folder contains comprehensive PlantUML diagrams for the Fliptable system, incorporating all subsystem components with professional styling and detailed specifications. These diagrams provide a complete visual representation of the system architecture for documentation section 8C.

## Diagram Files

### 1. Overall_System_Diagram_Detailed.puml
**Comprehensive System Architecture**

The main diagram showing the complete Fliptable system with all four primary subsystems:
- **Power Distribution System**: 3φ 240V AC supply, main breaker, E-Stop circuit, cabling
- **Control Cabinet (Brain Area)**: Siemens PLC with motion control and safety modules, HMI panel with all interface features, EtherNet/IP and EtherCAT communication buses, motor drives (VFDs and brake relay)
- **Lifting Subsystem (Body Area)**: 3-phase AC motor, dual synchronized lead screws, linear guide rails with misalignment compensation, limit switches, position encoders
- **Rotation Subsystem (Body Area)**: 0.33 HP 1-phase motor, 1517:1 self-locking gearbox, shaft assembly with bearings and collars, failsafe spring-applied brake, M20 fasteners with vibration damping, 2.4m × 1.8m tabletop

**Key Features:**
- Complete component hierarchy
- Power flow (red), control signals (blue), mechanical connections (orange), structural support (grey dashed), feedback (orange dashed)
- Professional color-coded subsystems
- Comprehensive legend with specifications
- All missing components from original diagram included

### 2. Rotation_Subsystem_Detailed.puml
**Detailed Rotation Subsystem**

Focuses on the rotation mechanism with enhanced detail:
- VFD inverter input (1-phase)
- 0.33 HP rotation motor with stall torque rating
- 1517:1 gearbox (self-locking safety feature)
- Torque-rated coupling
- Main drive shaft with shaft rings/collars for positioning
- Pillow block bearings for radial load support
- Mounting platform with M20 fasteners and vibration damping washers
- Failsafe brake (spring-applied, electric-release)
- Rotary encoder for angle feedback
- 2.4m × 1.8m custom-built tabletop

**Annotations:**
- Gearbox functions and torque calculations
- Brake safety features and fail-safe operation
- Anti-vibration mounting details

### 3. Lifting_Subsystem_Detailed.puml
**Detailed Lifting Subsystem**

Focuses on the vertical actuation system:
- 3-phase AC lift motor with variable speed VFD
- Synchronization mechanism for parallel motion
- Dual lead screw configuration (corners A and B)
- Linear guide rails with misalignment compensation
- Min-CD design to prevent binding
- Support bracing for structural integrity
- Moving platform carrying rotation subsystem
- Upper and lower limit switches for over-travel protection
- Position encoder for height feedback

**Annotations:**
- Synchronization functions
- Guide rail features and compensation
- Safety interlock system
- Dual actuator configuration benefits

### 4. Control_Subsystem_Detailed.puml
**Detailed Control Architecture**

Comprehensive control system showing all subsystems from the specification table:
- **Power Input**: 3φ 240V AC main supply, main breaker, E-Stop circuit, cabling
- **Main PLC Controller**: Siemens PLC, motion control module, safety logic module
- **HMI Panel**: Touchscreen with angle display, height display, status/alarms, manual jog, recipe selection
- **Communication Bus**: EtherNet/IP (control bus) and EtherCAT (motion network)
- **Motor Drives**: VFD for lift (3-phase), inverter for rotation (1-phase), brake relay
- **Feedback Sensors**: Lift and rotation encoders with position feedback
- **Safety Sensors**: Upper/lower limit switches, E-Stop integration

**Annotations:**
- Main PLC functions
- Communication hierarchy
- Motor drive types and purposes
- Safety features and scope limitations

### 5. System_Integration_Flow.puml
**Brain-to-Body Integration Flow**

High-level architecture showing the brain area (control) to body area (mechanical) flow:
- **Brain Area**: PLC, motion control, HMI, VFDs, brake control
- **Information Flow**: Control signals, setpoints, commands (digital/analog)
- **Energy Flow**: 3φ power, 1φ power, DC current (high power electrical)
- **Feedback Flow**: Position encoders, limit switches, motor status, safety interlocks
- **Body Area**: Lift motor → gearbox → lead screws → platform, rotation motor → gearbox → shaft → tabletop, brake actuation

**Key Concepts:**
- Separation of intelligence (brain) and actuation (body)
- Bidirectional information flow
- Unidirectional energy flow
- Energy conversion chain (electrical → rotational → linear/rotational)

## Styling and Color Scheme

### Professional Color Palette

All diagrams use a consistent, professional color scheme:

| Subsystem | Background | Border | Use Case |
|-----------|------------|--------|----------|
| **Power** | Light Red (#ffebee) | Dark Red (#c62828) | Power distribution, electrical |
| **Control** | Light Blue (#e3f2fd) | Blue (#1565c0) | PLC, logic, control systems |
| **Communication** | Cyan (#e1f5fe) | Teal (#0277bd) | Network buses, data exchange |
| **Lifting** | Light Purple (#f3e5f5) | Purple (#7b1fa2) | Lift motors, actuators |
| **Rotation** | Light Teal (#e0f2f1) | Dark Teal (#00897b) | Rotation motors, gearbox |
| **Mechanical** | Light Orange (#fff3e0) | Orange (#ef6c00) | Mechanical components |
| **Structure** | Light Grey (#eceff1) | Dark Grey (#546e7a) | Structural frame, dashed |
| **Sensor** | Light Amber (#fff8e1) | Orange (#f57c00) | Sensors, feedback |
| **Safety** | Light Red (#ffebee) | Red (#d32f2f) | E-Stop, brakes, thick border |

### Connection Line Styles

- **Power Flow**: Thick red solid lines (`=[#c62828,thickness=3]=>`)
- **Control Signals**: Blue solid lines (`-[#1565c0]->`)
- **Communication**: Light blue lines (`<-[#0277bd]->`)
- **Mechanical Power**: Orange/teal thick lines (`=[#ef6c00,thickness=3]=>`)
- **Structural Support**: Grey dashed lines (`..[#546e7a]..>`)
- **Sensor Feedback**: Orange dashed lines (`-..[#f57c00]..>`)
- **Safety Interlocks**: Red lines (`-[#d32f2f]->`)

### Font and Readability

- **Font**: Arial, 11-12pt
- **DPI**: 300 for high-quality export
- **Spacing**: nodesep 60-80, ranksep 70-100
- **Layout**: Orthogonal lines for clean appearance
- **Labels**: Bold for subsystem headers, specifications in subtitles

## Generating SVG Diagrams

### Prerequisites

Install PlantUML:
- **Java**: PlantUML requires Java Runtime Environment (JRE)
- **PlantUML**: Download from https://plantuml.com/download
- **Graphviz** (optional but recommended): For better layout - https://graphviz.org/download/

### Method 1: Command Line (Recommended)

Generate all diagrams at once:

```bash
# Navigate to diagram folder
cd 8B_system_diagram_V2

# Generate all SVG files (requires plantuml.jar)
java -jar plantuml.jar -tsvg *.puml

# Alternative: Generate PNG files
java -jar plantuml.jar -tpng *.puml

# Generate with specific DPI for high quality
java -jar plantuml.jar -tsvg -Sdpi=300 *.puml
```

Generate individual diagrams:

```bash
java -jar plantuml.jar -tsvg Overall_System_Diagram_Detailed.puml
java -jar plantuml.jar -tsvg Rotation_Subsystem_Detailed.puml
java -jar plantuml.jar -tsvg Lifting_Subsystem_Detailed.puml
java -jar plantuml.jar -tsvg Control_Subsystem_Detailed.puml
java -jar plantuml.jar -tsvg System_Integration_Flow.puml
```

### Method 2: Online PlantUML Editor

1. Visit https://www.plantuml.com/plantuml/uml/
2. Copy the contents of any `.puml` file
3. Paste into the editor
4. Click "Submit" to generate
5. Download as SVG, PNG, or other formats

### Method 3: VSCode Extension

1. Install "PlantUML" extension in VSCode
2. Open any `.puml` file
3. Press `Alt+D` to preview
4. Right-click preview → "Export Current Diagram" → Select format

### Method 4: IntelliJ IDEA / PyCharm

1. Install "PlantUML integration" plugin
2. Open any `.puml` file
3. View diagram in side panel
4. Right-click diagram → "Save Diagram"

## System Specifications Summary

### Rotation Subsystem
- **Motor**: 0.33 HP, 1-Phase AC
- **Gearbox**: 1517:1 ratio, self-locking
- **Max Speed**: 1 RPM (360° continuous)
- **Tabletop**: 2.4m × 1.8m custom built
- **Brake**: Spring-applied, electric-release, failsafe
- **Fasteners**: M20 bolts with vibration damping washers

### Lifting Subsystem
- **Motor**: 3-Phase AC, variable speed
- **Actuators**: Dual synchronized lead screws
- **Configuration**: Two corners for load distribution
- **Guidance**: Linear rails with misalignment compensation
- **Design**: Min-CD to prevent binding
- **Safety**: Upper/lower limit switches, position encoder

### Control Subsystem
- **Power**: 3φ 240V AC main supply
- **Controller**: Siemens PLC with motion and safety modules
- **HMI**: Touchscreen with angle, height, status, jog, recipes
- **Communication**: EtherNet/IP (control) + EtherCAT (motion)
- **Drives**: VFD (3-phase lift), Inverter (1-phase rotation)
- **Feedback**: Encoders for position, limit switches for safety

### Structural Frame
- **Platform**: Moving shelf for rotation subsystem
- **Guides**: Linear guide rails
- **Bracing**: Support structure
- **Fastening**: M20 through-hole bolts

## Design Considerations

### Safety Features
- Self-locking gearbox (>50:1 ratio)
- Spring-applied failsafe brake
- E-Stop emergency circuit
- Upper/lower limit switches
- Safety logic module in PLC
- Brake withstands motor stall torque

### Mechanical Reliability
- Vibration-damping washers on all fasteners
- Dual lead screw configuration for stability
- Misalignment compensation in guide rails
- Min-CD design prevents binding
- Synchronized parallel motion

### Control Architecture
- Dual communication buses (control + motion)
- Real-time position feedback
- Coordinated motion control
- Recipe-based operation capability
- Manual jog for setup

### Load Management
- Off-center load capability
- Distributed load across two corners
- Bearing support for vertical loads
- Torque-rated couplings
- Structural bracing

## Usage in Documentation

These diagrams are designed for **Section 8C: System Block Diagram** in the Fliptable project documentation.

### Recommended Usage

1. **Overall_System_Diagram_Detailed.puml**: Use as the main system block diagram (8C.1)
2. **System_Integration_Flow.puml**: Use to explain brain-to-body architecture
3. **Individual subsystem diagrams**: Use in section 8C.2 subsystem descriptions
   - Rotation_Subsystem_Detailed.puml for rotation subsystem
   - Lifting_Subsystem_Detailed.puml for lifting subsystem
   - Control_Subsystem_Detailed.puml for control subsystem

### Document Integration

For LaTeX/Word documents:
- Export as SVG for vector graphics (scalable, high quality)
- Export as PNG at 300 DPI for raster graphics
- Include legends in captions or reference them

Example caption:
> *Figure 8C.1: Overall Fliptable System Architecture showing power distribution, control cabinet (brain area), lifting subsystem, and rotation subsystem (body area). Color coding indicates subsystem types, with power flow (red), control signals (blue), mechanical connections (orange), and feedback (dashed).*

## Differences from V1

This V2 folder contains significant enhancements over the original `8B_system_diagram/`:

### New Components Added
- Platform and fasteners (M20 with vibration damping)
- Shaft rings/collars
- Motion control and safety logic modules
- Detailed HMI interface features
- EtherCAT motion network (in addition to EtherNet/IP)
- Dual lead screw configuration
- Synchronization mechanism
- Misalignment compensation notation
- Safety sensor processing

### Enhanced Styling
- Higher DPI (300) for professional quality
- Better font selection (Arial)
- Improved spacing and layout
- More detailed annotations and notes
- Comprehensive legends with specifications
- Thicker lines for visual hierarchy
- Professional color palette consistency

### New Diagrams
- System_Integration_Flow.puml: Brain-to-body architecture
- More detailed control subsystem breakdown
- Enhanced rotation and lifting subsystem details

### Documentation
- This comprehensive README file
- Generation instructions
- System specifications summary
- Design considerations

## Maintenance

When updating diagrams:
1. Maintain consistent color scheme across all files
2. Keep line thickness consistent for similar connection types
3. Update legend if adding new connection types
4. Regenerate all SVGs after changes
5. Verify specifications match current design

## Questions or Issues

For questions about the diagrams or to report issues:
- Check PlantUML syntax: https://plantuml.com/
- Verify Java and Graphviz installation
- Ensure file encoding is UTF-8
- Contact project team for specification updates

---

**Version**: 2.0
**Date**: February 2026
**Project**: MECH 458 Fliptable System
**Documentation Section**: 8C - System Block Diagram
