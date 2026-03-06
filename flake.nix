{
  description = "FHS compatible environments for builds";

  outputs = {
    self,
    nixpkgs,
  }: let
    system = "x86_64-linux";
    pkgs = import nixpkgs {inherit system;};
  in {
    packages.${system} = {
      test = pkgs.buildFHSEnv {
        name = "fhs-java17-build";

        targetPkgs = pkgs: [
          pkgs.openjdk17
        ];

        multiPkgs = pkgs: [
          pkgs.openjdk17
        ];

        runScript = ''
          bash -c "
            export JAVA_HOME='${pkgs.openjdk17}'
            export HALSIM_EXTENSIONS=""
            ./gradlew build
          "
        '';
      };

      simulate = pkgs.buildFHSEnv {
        name = "fhs-java17-glx-simulate";

        targetPkgs = pkgs: [
          pkgs.openjdk17
          pkgs.libGL
          pkgs.libGLU
          pkgs.xorg.libX11
          pkgs.xorg.libXext
          pkgs.xorg.libXrender
          pkgs.xorg.libXtst
          pkgs.xorg.libXi
        ];

        multiPkgs = pkgs: [
          pkgs.openjdk17
        ];

        runScript = ''
          bash -c "
            export JAVA_HOME='${pkgs.openjdk17}'
            export HALSIM_EXTENSIONS=\"\$(pwd)/build/jni/release/libhalsim_gui.so\"
            exec ./gradlew simulateJava
          "
        '';
      };
    };

    devShells.${system}.default = pkgs.mkShell {
      buildInputs = [
        pkgs.openjdk17
        pkgs.libGL
        pkgs.libGLU
        pkgs.xorg.libX11
        pkgs.xorg.libXext
        pkgs.xorg.libXrender
        pkgs.xorg.libXtst
        pkgs.xorg.libXi
        pkgs.git
        pkgs.gradle
      ];

      shellHook = ''
        export JAVA_HOME="${pkgs.openjdk17}"
        export HALSIM_EXTENSIONS=""
      '';
    };
  };
}
