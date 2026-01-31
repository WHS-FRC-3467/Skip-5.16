#!/usr/bin/env python3
"""
Script to generate a Mermaid diagram showing the structure of the frc/robot package.
This includes subsystems, commands, and their relationships.

This script is automatically run during the build process (via the generateMermaidDiagram
Gradle task) to keep the README.md diagram up to date with the latest code structure.

The diagram shows:
- Main robot classes (Robot, RobotContainer, etc.)
- Subsystems with their key public methods
- Commands (both regular and autonomous)
- Relationships between components

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
    is_subsystem: bool = False
    is_command: bool = False
    is_constants: bool = False


class JavaParser:
    """Parser for Java files to extract class information"""

    def __init__(self, robot_path: Path, repo_root: Path):
        self.robot_path = robot_path
        self.repo_root = repo_root
        self.classes: Dict[str, JavaClass] = {}

    def parse_files(self):
        """Parse all Java files in the robot directory"""
        for java_file in self.robot_path.rglob("*.java"):
            if java_file.is_file():
                self._parse_file(java_file)

    def _parse_file(self, file_path: Path):
        """Parse a single Java file"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract package name
            package_match = re.search(r'package\s+([\w.]+)\s*;', content)
            package = package_match.group(1) if package_match else ""

            # Extract class/interface declaration
            class_pattern = r'public\s+(?:final\s+)?(?:abstract\s+)?(class|interface|enum)\s+(\w+)(?:\s+extends\s+([\w<>?,\s]+))?(?:\s+implements\s+([\w<>?,\s]+))?'
            class_match = re.search(class_pattern, content)
            
            if not class_match:
                return

            class_name = class_match.group(2)
            extends = class_match.group(3).strip() if class_match.group(3) else None
            implements_str = class_match.group(4)
            implements = [i.strip() for i in implements_str.split(',')] if implements_str else []

            # Create JavaClass object - calculate path relative to repo root
            java_class = JavaClass(
                name=class_name,
                package=package,
                full_path=str(file_path.relative_to(self.repo_root)),
                extends=extends,
                implements=implements
            )

            # Determine class type
            java_class.is_subsystem = 'SubsystemBase' in str(extends) or 'subsystems' in package
            java_class.is_command = 'Command' in str(extends) or 'commands' in package
            java_class.is_constants = class_name.endswith('Constants')

            # Extract public methods (excluding constructors)
            method_pattern = r'public\s+(?:static\s+)?(?:final\s+)?(?:<[\w\s,<>?]+>\s+)?(\w+(?:<[\w\s,<>?]+>)?)\s+(\w+)\s*\('
            methods = re.findall(method_pattern, content)
            
            for return_type, method_name in methods:
                # Skip constructors
                if method_name != class_name:
                    java_class.methods.append(f"{method_name}()")

            # Extract fields (subsystems in RobotContainer)
            if class_name == "RobotContainer":
                field_pattern = r'private\s+(?:final\s+)?(\w+)\s+(\w+)\s*[;=]'
                fields = re.findall(field_pattern, content)
                for field_type, field_name in fields:
                    java_class.fields.append(f"{field_name}: {field_type}")

            self.classes[class_name] = java_class

        except Exception as e:
            print(f"Error parsing {file_path}: {e}")


class MermaidGenerator:
    """Generates Mermaid diagram from parsed Java classes"""

    def __init__(self, classes: Dict[str, JavaClass]):
        self.classes = classes

    def generate(self) -> str:
        """Generate the complete Mermaid diagram"""
        lines = ["```mermaid", "classDiagram"]
        
        # Generate class definitions
        lines.extend(self._generate_main_classes())
        lines.extend(self._generate_subsystems())
        lines.extend(self._generate_commands())
        
        # Generate relationships
        lines.extend(self._generate_relationships())
        
        lines.append("```")
        return "\n".join(lines)

    def _generate_main_classes(self) -> List[str]:
        """Generate main robot classes"""
        lines = []
        main_classes = ['Robot', 'RobotContainer', 'Main', 'Constants', 'Ports', 'RobotState', 'FieldConstants']
        
        for class_name in main_classes:
            if class_name in self.classes:
                java_class = self.classes[class_name]
                lines.append(f"    class {class_name} {{")
                
                # Add fields for RobotContainer
                if class_name == "RobotContainer" and java_class.fields:
                    for field in java_class.fields[:10]:  # Limit to first 10 fields
                        lines.append(f"        +{field}")
                
                # Add methods (limit to key methods)
                for method in java_class.methods[:8]:  # Limit to first 8 methods
                    lines.append(f"        +{method}")
                
                lines.append("    }")
        
        return lines

    def _generate_subsystems(self) -> List[str]:
        """Generate subsystem classes"""
        lines = []
        subsystems = [c for c in self.classes.values() if c.is_subsystem and not c.is_constants]
        
        for subsystem in sorted(subsystems, key=lambda x: x.name):
            lines.append(f"    class {subsystem.name} {{")
            lines.append("        <<subsystem>>")
            
            # Add methods (limit to avoid cluttering)
            for method in subsystem.methods[:6]:  # Limit to first 6 methods
                lines.append(f"        +{method}")
            
            lines.append("    }")
        
        return lines

    def _generate_commands(self) -> List[str]:
        """Generate command classes"""
        lines = []
        commands = [c for c in self.classes.values() if c.is_command and 'autos' not in c.package]
        
        # Group autonomous commands separately
        auto_commands = [c for c in self.classes.values() if c.is_command and 'autos' in c.package]
        
        # Regular commands
        for command in sorted(commands, key=lambda x: x.name):
            lines.append(f"    class {command.name} {{")
            lines.append("        <<command>>")
            for method in command.methods[:4]:  # Limit to first 4 methods
                lines.append(f"        +{method}")
            lines.append("    }")
        
        # Auto commands (grouped)
        if auto_commands:
            lines.append("    class AutoCommands {")
            lines.append("        <<namespace>>")
            # List auto command classes (not as methods, so no parentheses)
            for auto in sorted(auto_commands, key=lambda x: x.name):
                # Don't add parentheses - these are class names, not method calls
                lines.append(f"        {auto.name}")
            lines.append("    }")
        
        return lines

    def _generate_relationships(self) -> List[str]:
        """Generate relationships between classes"""
        lines = []
        
        # Robot -> RobotContainer
        if 'Robot' in self.classes and 'RobotContainer' in self.classes:
            lines.append("    Robot --> RobotContainer : uses")
        
        # RobotContainer -> Subsystems
        if 'RobotContainer' in self.classes:
            robot_container = self.classes['RobotContainer']
            subsystems = [c.name for c in self.classes.values() if c.is_subsystem and not c.is_constants]
            
            for subsystem in subsystems:
                # Check if subsystem is used in RobotContainer
                for field in robot_container.fields:
                    if subsystem.lower() in field.lower():
                        lines.append(f"    RobotContainer --> {subsystem} : contains")
                        break
        
        # Commands use subsystems (inheritance relationships)
        for class_name, java_class in self.classes.items():
            if java_class.extends:
                # Clean up generic types
                extends_clean = re.sub(r'<[^>]+>', '', java_class.extends)
                if extends_clean in self.classes:
                    lines.append(f"    {class_name} --|> {extends_clean} : extends")
        
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
    description = """## Robot Structure

This diagram is automatically generated from the code in `src/main/java/frc/robot/` and updated during the build process.
It shows the relationships between subsystems, commands, and core robot classes.

**Legend:**
- `<<subsystem>>` - Robot subsystems that control hardware
- `<<command>>` - Commands that define robot behaviors
- `<<namespace>>` - Grouping of autonomous commands

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
    robot_path = repo_root / "src" / "main" / "java" / "frc" / "robot"
    readme_path = repo_root / "README.md"

    if not robot_path.exists():
        print(f"Error: Robot path not found: {robot_path}")
        return 1

    # Parse Java files
    print(f"Parsing Java files in {robot_path}...")
    parser = JavaParser(robot_path, repo_root)
    parser.parse_files()
    print(f"Found {len(parser.classes)} classes")

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
