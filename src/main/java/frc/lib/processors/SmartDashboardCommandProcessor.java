/*
 * Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */
package frc.lib.processors;

import frc.lib.util.SmartDashboardCommand;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.RoundEnvironment;
import javax.annotation.processing.SupportedAnnotationTypes;
import javax.annotation.processing.SupportedSourceVersion;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.Element;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.TypeElement;
import javax.lang.model.type.DeclaredType;
import javax.lang.model.type.TypeMirror;
import javax.tools.Diagnostic;
import javax.tools.JavaFileObject;

/**
 * Annotation processor for {@link SmartDashboardCommand} that generates code to automatically
 * register annotated commands on the SmartDashboard.
 *
 * <p>
 * This processor runs at compile time and generates a {@code SmartDashboardCommands} class with a
 * {@code register(RobotContainer)} method that contains all the SmartDashboard.putData() calls for
 * annotated commands.
 *
 * <p>
 * The generated class should be called from {@code RobotContainer} after subsystems are
 * initialized.
 */
@SupportedAnnotationTypes("frc.lib.util.SmartDashboardCommand")
@SupportedSourceVersion(SourceVersion.RELEASE_17)
public class SmartDashboardCommandProcessor extends AbstractProcessor {

    private static final String GENERATED_CLASS_NAME = "SmartDashboardCommands";
    private static final String GENERATED_PACKAGE = "frc.robot";

    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        // Only process in the final round
        if (roundEnv.processingOver()) {
            return false;
        }

        // Collect all annotated methods grouped by their containing class
        Map<TypeElement, List<MethodInfo>> subsystemCommands = new HashMap<>();

        for (Element element : roundEnv.getElementsAnnotatedWith(SmartDashboardCommand.class)) {
            if (!(element instanceof ExecutableElement)) {
                processingEnv.getMessager().printMessage(
                    Diagnostic.Kind.ERROR,
                    "@SmartDashboardCommand can only be applied to methods",
                    element);
                continue;
            }

            ExecutableElement method = (ExecutableElement) element;

            // Validate the method
            if (!validateMethod(method)) {
                continue;
            }

            // Get the containing class (subsystem)
            TypeElement subsystemClass = (TypeElement) method.getEnclosingElement();

            // Get annotation values
            SmartDashboardCommand annotation =
                method.getAnnotation(SmartDashboardCommand.class);
            String key = annotation.key();

            if (key == null || key.trim().isEmpty()) {
                processingEnv.getMessager().printMessage(
                    Diagnostic.Kind.ERROR,
                    "@SmartDashboardCommand key cannot be empty",
                    method);
                continue;
            }

            // Store method info
            subsystemCommands.computeIfAbsent(subsystemClass, k -> new ArrayList<>())
                .add(new MethodInfo(method.getSimpleName().toString(), key));
        }

        // Generate the SmartDashboardCommands class if we found any annotations
        if (!subsystemCommands.isEmpty()) {
            try {
                generateSmartDashboardCommandsClass(subsystemCommands);
            } catch (IOException e) {
                processingEnv.getMessager().printMessage(
                    Diagnostic.Kind.ERROR,
                    "Failed to generate SmartDashboardCommands class: " + e.getMessage());
            }
        }

        return true;
    }

    /**
     * Validates that the annotated method meets all requirements.
     */
    private boolean validateMethod(ExecutableElement method) {
        // Check if method is public
        if (!method.getModifiers().contains(Modifier.PUBLIC)) {
            processingEnv.getMessager().printMessage(
                Diagnostic.Kind.ERROR,
                "@SmartDashboardCommand method must be public",
                method);
            return false;
        }

        // Check if method is static
        if (method.getModifiers().contains(Modifier.STATIC)) {
            processingEnv.getMessager().printMessage(
                Diagnostic.Kind.ERROR,
                "@SmartDashboardCommand method cannot be static",
                method);
            return false;
        }

        // Check if method returns Command
        TypeMirror returnType = method.getReturnType();
        if (!isCommandType(returnType)) {
            processingEnv.getMessager().printMessage(
                Diagnostic.Kind.ERROR,
                "@SmartDashboardCommand method must return edu.wpi.first.wpilibj2.command.Command",
                method);
            return false;
        }

        // Check if method has parameters (should have none for simple commands)
        if (!method.getParameters().isEmpty()) {
            processingEnv.getMessager().printMessage(
                Diagnostic.Kind.WARNING,
                "@SmartDashboardCommand method should not have parameters. "
                    + "Commands with parameters cannot be registered on SmartDashboard.",
                method);
            return false;
        }

        return true;
    }

    /**
     * Checks if the given type is or extends edu.wpi.first.wpilibj2.command.Command.
     */
    private boolean isCommandType(TypeMirror type) {
        if (!(type instanceof DeclaredType)) {
            return false;
        }

        DeclaredType declaredType = (DeclaredType) type;
        Element element = declaredType.asElement();

        if (!(element instanceof TypeElement)) {
            return false;
        }

        TypeElement typeElement = (TypeElement) element;
        String qualifiedName = typeElement.getQualifiedName().toString();

        // Check if it's exactly Command or a subclass
        if ("edu.wpi.first.wpilibj2.command.Command".equals(qualifiedName)) {
            return true;
        }

        // Check superclass
        TypeMirror superclass = typeElement.getSuperclass();
        if (superclass != null && isCommandType(superclass)) {
            return true;
        }

        // Check interfaces
        for (TypeMirror iface : typeElement.getInterfaces()) {
            if (isCommandType(iface)) {
                return true;
            }
        }

        return false;
    }

    /**
     * Generates the SmartDashboardCommands class with all registration calls.
     */
    private void generateSmartDashboardCommandsClass(
        Map<TypeElement, List<MethodInfo>> subsystemCommands) throws IOException {

        JavaFileObject sourceFile = processingEnv.getFiler()
            .createSourceFile(GENERATED_PACKAGE + "." + GENERATED_CLASS_NAME);

        try (PrintWriter writer = new PrintWriter(sourceFile.openWriter())) {
            // File header
            writer.println("/*");
            writer.println(" * Copyright (C) 2026 Windham Windup");
            writer.println(" *");
            writer.println(
                " * This file was automatically generated by SmartDashboardCommandProcessor.");
            writer.println(" * Do not modify this file manually.");
            writer.println(" */");
            writer.println("package " + GENERATED_PACKAGE + ";");
            writer.println();
            writer.println("import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;");
            writer.println();
            writer.println("/**");
            writer.println(
                " * Auto-generated class for registering @SmartDashboardCommand annotated methods.");
            writer.println(" * <p>Generated at compile time by SmartDashboardCommandProcessor.");
            writer.println(" */");
            writer.println("public final class " + GENERATED_CLASS_NAME + " {");
            writer.println();
            writer.println("    private " + GENERATED_CLASS_NAME + "() {");
            writer.println("        throw new UnsupportedOperationException("
                + "\"This is a utility class and cannot be instantiated\");");
            writer.println("    }");
            writer.println();
            writer.println("    /**");
            writer.println("     * Registers all commands annotated with @SmartDashboardCommand "
                + "on the SmartDashboard.");
            writer.println("     * <p>This method should be called from RobotContainer after "
                + "all subsystems are initialized.");
            writer.println("     * @param container The RobotContainer instance containing all "
                + "subsystems");
            writer.println("     */");
            writer.println("    public static void register(RobotContainer container) {");

            // Generate SmartDashboard.putData calls for each subsystem/command
            for (Map.Entry<TypeElement, List<MethodInfo>> entry : subsystemCommands.entrySet()) {
                TypeElement subsystem = entry.getKey();
                List<MethodInfo> methods = entry.getValue();

                // Try to determine the field name in RobotContainer
                String subsystemClassName = subsystem.getSimpleName().toString();
                String fieldName = getFieldNameForSubsystem(subsystemClassName);

                writer.println("        // Commands from " + subsystem.getQualifiedName());

                for (MethodInfo method : methods) {
                    writer.println("        SmartDashboard.putData(\"" + method.key + "\", "
                        + "container." + fieldName + "." + method.methodName + "());");
                }
                writer.println();
            }

            writer.println("    }");
            writer.println("}");
        }

        processingEnv.getMessager().printMessage(
            Diagnostic.Kind.NOTE,
            "Generated " + GENERATED_CLASS_NAME + " with "
                + subsystemCommands.values().stream().mapToInt(List::size).sum()
                + " command registrations");
    }

    /**
     * Attempts to determine the field name in RobotContainer for a given subsystem class. Uses
     * simple heuristics: lowercase first letter of class name.
     */
    private String getFieldNameForSubsystem(String className) {
        // Simple heuristic: convert IntakeSuperstructure -> intake, Drive -> drive
        if (className.isEmpty()) {
            return className;
        }

        // Handle special cases
        if (className.endsWith("Superstructure")) {
            className = className.substring(0, className.length() - "Superstructure".length());
        }

        return Character.toLowerCase(className.charAt(0)) + className.substring(1);
    }

    /**
     * Helper class to store information about annotated methods.
     */
    private static class MethodInfo {
        final String methodName;
        final String key;

        MethodInfo(String methodName, String key) {
            this.methodName = methodName;
            this.key = key;
        }
    }
}
