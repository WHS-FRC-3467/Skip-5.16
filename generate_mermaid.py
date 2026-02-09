#!/usr/bin/env python3
"""
Script to generate a Mermaid diagram showing the architectural layers of the robot code.

This script is automatically run during the build process (via the generateMermaidDiagram
Gradle task) to keep the README.md diagram up to date with the latest code structure.

The diagram shows the three-layer architecture:
1. **lib/io** - Hardware abstraction interfaces (MotorIO, BeamBreakIO, etc.)
2. **lib/mechanisms** - Reusable mechanism abstractions (FlywheelMechanism, RotaryMechanism, etc.)
3. **robot/subsystems** - Robot-specific subsystems that use mechanisms

The diagram focuses on showing how these layers interact:
- Mechanisms use IO interfaces
- Subsystems use Mechanisms
- Clear separation of concerns

The diagram is inserted into README.md between the markers:
<!-- MERMAID_DIAGRAM_START --> and <!-- MERMAID_DIAGRAM_END -->
"""

import os
import re
from pathlib import Path
from typing import List, Dict, Set, Optional
from dataclasses import dataclass, field


@dataclass
class JavaClass:
    """Represents a Java class with its methods and relationships"""
    name: str
    package: str
    full_path: str
    extends: Optional[str] = None
    implements: List[str] = field(default_factory=list)
    methods: List[str] = field(default_factory=list)
    fields: List[str] = field(default_factory=list)
    generic_params: List[str] = field(default_factory=list)
    is_interface: bool = False
    layer: str = ""  # 'io', 'mechanism', 'subsystem'


class JavaParser:
    """Parser for Java files to extract class information"""

    def __init__(self, paths: Dict[str, Path], repo_root: Path):
        """
        Initialize parser with multiple paths to parse.
        
        Args:
            paths: Dict mapping layer names ('io', 'mechanism', 'subsystem') to paths
            repo_root: Root of the repository for relative path calculation
        """
        self.paths = paths
        self.repo_root = repo_root
        self.classes: Dict[str, JavaClass] = {}

    def parse_files(self):
        """Parse all Java files in the specified paths"""
        for layer, path in self.paths.items():
            if path.exists():
                print(f"  Parsing {layer} layer from {path.relative_to(self.repo_root)}...")
                for java_file in path.rglob("*.java"):
                    if java_file.is_file():
                        self._parse_file(java_file, layer)

    def _parse_file(self, file_path: Path, layer: str):
        """Parse a single Java file"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract package name
            package_match = re.search(r'package\s+([\w.]+)\s*;', content)
            package = package_match.group(1) if package_match else ""

            # Extract class/interface declaration
            class_pattern = r'public\s+(?:final\s+)?(?:abstract\s+)?(class|interface|enum)\s+(\w+)(?:<([^>]+)>)?(?:\s+extends\s+([\w<>?,\s]+))?(?:\s+implements\s+([\w<>?,\s]+))?'
            class_match = re.search(class_pattern, content)
            
            if not class_match:
                return

            class_type = class_match.group(1)
            class_name = class_match.group(2)
            generic_params_str = class_match.group(3)
            extends = class_match.group(4).strip() if class_match.group(4) else None
            implements_str = class_match.group(5)
            
            # Parse generic parameters
            generic_params = []
            if generic_params_str:
                generic_params = [g.strip() for g in generic_params_str.split(',')]
            
            implements = [i.strip() for i in implements_str.split(',')] if implements_str else []

            # Create JavaClass object - calculate path relative to repo root
            java_class = JavaClass(
                name=class_name,
                package=package,
                full_path=str(file_path.relative_to(self.repo_root)),
                extends=extends,
                implements=implements,
                generic_params=generic_params,
                is_interface=(class_type == 'interface'),
                layer=layer
            )

            # Extract fields (for subsystems to see what mechanisms they use)
            if layer == 'subsystem':
                field_pattern = r'private\s+(?:final\s+)?(\w+(?:<[^>]*>)?)\s+(\w+)\s*[;=]'
                fields = re.findall(field_pattern, content)
                for field_type, field_name in fields:
                    # Clean up generic type markers
                    clean_type = re.sub(r'<.*?>', '', field_type)
                    # Only track mechanism and IO types
                    if 'Mechanism' in clean_type or clean_type.endswith('IO'):
                        java_class.fields.append(f"{field_name}: {field_type}")

            self.classes[class_name] = java_class

        except Exception as e:
            print(f"Warning: Error parsing {file_path}: {e}")


class MermaidGenerator:
    """Generates Mermaid diagram from parsed Java classes"""

    def __init__(self, classes: Dict[str, JavaClass]):
        self.classes = classes

    def generate(self) -> str:
        """Generate the complete Mermaid diagram showing three architectural layers"""
        lines = ["```mermaid", "classDiagram"]
        
        # Generate sections for each layer
        lines.extend(self._generate_io_layer())
        lines.extend(self._generate_mechanism_layer())
        lines.extend(self._generate_subsystem_layer())
        
        # Generate relationships between layers
        lines.extend(self._generate_relationships())
        
        lines.append("```")
        return "\n".join(lines)

    def _generate_io_layer(self) -> List[str]:
        """Generate IO interface layer"""
        lines = []
        io_classes = [c for c in self.classes.values() if c.layer == 'io']
        
        # Filter to main IO interfaces only (exclude implementations, Sim variants, and hardware-specific variants)
        main_ios = [c for c in io_classes if (
            c.name.endswith('IO') and 
            'Sim' not in c.name and
            not any(impl in c.name for impl in ['TalonFX', 'SparkMax', 'CANCoder', 'DIO', 'LaserCAN', 'PWM', 'PhotonVision', 'Candle', 'CANRange'])
        )]
        
        # Sort by name for consistent ordering
        for io in sorted(main_ios, key=lambda x: x.name):
            lines.append(f"    class {io.name} {{")
            lines.append("        <<IO Interface>>")
            lines.append("    }")
        
        return lines

    def _generate_mechanism_layer(self) -> List[str]:
        """Generate Mechanism abstraction layer"""
        lines = []
        mechanisms = [c for c in self.classes.values() if c.layer == 'mechanism']
        
        # Filter to main mechanism classes (abstract base classes)
        main_mechanisms = [c for c in mechanisms if not any(impl in c.name for impl in ['Real', 'Sim', 'Visualizer'])]
        
        for mechanism in sorted(main_mechanisms, key=lambda x: x.name):
            lines.append(f"    class {mechanism.name} {{")
            lines.append("        <<Mechanism>>")
            
            # Show generic parameter if present
            if mechanism.generic_params:
                generic_display = ", ".join(mechanism.generic_params)
                lines.append(f"        ~T: {generic_display}~")
            
            lines.append("    }")
        
        return lines

    def _generate_subsystem_layer(self) -> List[str]:
        """Generate Subsystem layer"""
        lines = []
        subsystems = [c for c in self.classes.values() if c.layer == 'subsystem']
        
        # Filter out Constants and helper classes
        main_subsystems = [s for s in subsystems if not s.name.endswith('Constants') and 'SubsystemBase' in str(s.extends)]
        
        for subsystem in sorted(main_subsystems, key=lambda x: x.name):
            lines.append(f"    class {subsystem.name} {{")
            lines.append("        <<Subsystem>>")
            
            # Show which mechanisms it uses
            if subsystem.fields:
                for field in subsystem.fields[:3]:  # Limit to first 3 fields
                    lines.append(f"        -{field}")
            
            lines.append("    }")
        
        return lines

    def _generate_relationships(self) -> List[str]:
        """Generate relationships showing the three-layer architecture"""
        lines = []
        
        # Mechanism uses IO relationships
        mechanisms = [c for c in self.classes.values() if c.layer == 'mechanism']
        for mechanism in mechanisms:
            if mechanism.generic_params:
                # Check if generic parameter extends an IO interface
                for param in mechanism.generic_params:
                    # Extract the bound if present (e.g., "T extends MotorIO" -> "MotorIO")
                    match = re.search(r'extends\s+(\w+)', param)
                    if match:
                        io_name = match.group(1)
                        if io_name in self.classes and self.classes[io_name].layer == 'io':
                            lines.append(f"    {mechanism.name} ..> {io_name} : uses")
        
        # Subsystem uses Mechanism relationships
        subsystems = [c for c in self.classes.values() if c.layer == 'subsystem']
        for subsystem in subsystems:
            if subsystem.fields:
                for field in subsystem.fields:
                    # Extract the type from "fieldName: Type" or "fieldName: Type<...>"
                    field_match = re.search(r':\s*(\w+)', field)
                    if field_match:
                        type_name = field_match.group(1)
                        # Check if this is a mechanism
                        if type_name in self.classes and self.classes[type_name].layer == 'mechanism':
                            lines.append(f"    {subsystem.name} --> {type_name} : uses")
        
        return lines


def update_readme(diagram: str, readme_path: Path):
    """Update README.md with the generated diagram"""
    # Read existing README
    if readme_path.exists():
        with open(readme_path, 'r', encoding='utf-8') as f:
            content = f.read()
    else:
        content = ""

    # Define markers for the diagram section
    start_marker = "<!-- MERMAID_DIAGRAM_START -->"
    end_marker = "<!-- MERMAID_DIAGRAM_END -->"

    # Create the diagram section with description
    description = """## Robot Code Architecture

This diagram is automatically generated from the codebase and shows the three-layer architecture.

**Architecture Layers:**

1. **IO Layer** (`frc.lib.io.*`) - Hardware abstraction interfaces
   - Define contracts for interacting with hardware (motors, sensors, etc.)
   - Multiple implementations per interface (real hardware, simulation)
   
2. **Mechanism Layer** (`frc.lib.mechanisms.*`) - Reusable mechanism abstractions
   - Provide common patterns for robot mechanisms (flywheels, arms, elevators)
   - Generic over IO interfaces for hardware independence
   
3. **Subsystem Layer** (`frc.robot.subsystems.*`) - Robot-specific logic
   - Implement robot behaviors using mechanisms
   - Extend WPILib's SubsystemBase
   - Expose Command factories for robot actions

**Key Relationships:**
- Mechanisms use IO interfaces (via generics)
- Subsystems use Mechanisms
- Clear separation enables testing, simulation, and code reuse

To manually regenerate the diagram, run: `./gradlew generateMermaidDiagram`

"""
    diagram_section = f"{start_marker}\n{description}{diagram}\n\n{end_marker}"

    # Check if markers exist
    if start_marker in content and end_marker in content:
        # Replace existing diagram
        pattern = f"{re.escape(start_marker)}.*?{re.escape(end_marker)}"
        new_content = re.sub(pattern, diagram_section, content, flags=re.DOTALL)
    else:
        # Append diagram to end of README
        new_content = content.rstrip() + "\n\n" + diagram_section + "\n"

    # Write updated README
    with open(readme_path, 'w', encoding='utf-8') as f:
        f.write(new_content)


def main():
    """Main entry point"""
    # Get the repository root (assuming script is run from repo root)
    repo_root = Path(__file__).parent.absolute()
    src_path = repo_root / "src" / "main" / "java" / "frc"
    readme_path = repo_root / "README.md"

    # Define paths for the three layers
    paths = {
        'io': src_path / "lib" / "io",
        'mechanism': src_path / "lib" / "mechanisms",
        'subsystem': src_path / "robot" / "subsystems"
    }

    # Verify paths exist
    for layer, path in paths.items():
        if not path.exists():
            print(f"Warning: {layer} path not found: {path}")

    # Parse Java files from all three layers
    print("Parsing architectural layers...")
    parser = JavaParser(paths, repo_root)
    parser.parse_files()
    
    # Count classes by layer
    io_count = len([c for c in parser.classes.values() if c.layer == 'io'])
    mech_count = len([c for c in parser.classes.values() if c.layer == 'mechanism'])
    sub_count = len([c for c in parser.classes.values() if c.layer == 'subsystem'])
    
    print(f"  Found {io_count} IO classes")
    print(f"  Found {mech_count} Mechanism classes")
    print(f"  Found {sub_count} Subsystem classes")
    print(f"  Total: {len(parser.classes)} classes")

    # Generate Mermaid diagram
    print("Generating Mermaid diagram...")
    generator = MermaidGenerator(parser.classes)
    diagram = generator.generate()

    # Update README
    print(f"Updating {readme_path}...")
    update_readme(diagram, readme_path)
    print("Done!")

    return 0


if __name__ == "__main__":
    exit(main())
