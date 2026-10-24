# Third-party source and licensing

## Trkkhrmn/ros2_amr_mecanumbot

- Source: https://github.com/Trkkhrmn/ros2_amr_mecanumbot
- Baseline commit: `385b761c8c96757d2cade890c105f1dbfe187ed4`
- Copyright (c) 2025 Tarik Kahraman.
- License: MIT, retained in root `LICENSE` and `third_party/ros2_amr_mecanumbot.LICENSE`.
- The original `mecanum_*`, hardware and other upstream files are preserved.
- Fortress world ground/light was adapted from `minimal.world`. Robot layout and
  bringup architecture were used as references. See `docs/reuse_matrix.md`.
- The original README is preserved at `docs/upstream_README.md`.

## OSRF gz_ros2_control_demos

- Source: https://github.com/ros-controls/gz_ros2_control
- Installed package version: `0.7.18` (Debian full version in `results/p0/packages.tsv`).
- Reference: `urdf/test_mecanum_drive.xacro.urdf`.
- Copyright 2024 Open Source Robotics Foundation, Inc.
- License: Apache License 2.0, in `third_party/Apache-2.0.LICENSE`.
- Wheel spherical collision and anisotropic friction in
  `src/forklift_description/urdf/mecanum_wheels.xacro` were adapted for scale and
  Fortress's `ignition:expressed_in` attribute. Copyright/license comments retained.
- Baseline scripts derive a temporary headless launch/model from the installed
  demo and do not alter installed system files.

New `forklift_*` code is under Apache-2.0 as declared in each package's
`package.xml` and package `LICENSE`. MIT notices continue to apply to inherited
upstream material. Test/utility code added for P0/P1 is Apache-2.0.

`src/forklift_gazebo/config/gui.config` adapts the installed Gazebo Fortress
default GUI configuration from OSRF (Gazebo 6.17.1, Apache-2.0), changing the
camera pose and suppressing startup/exit dialogs. Its original GUI plugins and
configuration structure are preserved.

## Navigation configuration (P4)

`src/forklift_navigation/config/nav2.yaml` adapts the installed
`nav2_bringup` 1.1.20 `nav2_params.yaml`, Navigation2/Intel contributors,
Apache-2.0: https://github.com/ros-navigation/navigation2 . Geometry, omni motion,
velocity/acceleration bounds, goals and footprint critics were changed.

`src/forklift_navigation/config/slam.yaml` adapts the installed SLAM Toolbox
2.6.10 online async configuration: https://github.com/SteveMacenski/slam_toolbox .
The upstream Debian metadata declares LGPL (retained in
`third_party/slam_toolbox.copyright`); consult the upstream source for its full
license. It remains configuration for the external library; no SLAM Toolbox
implementation is copied into this project. P3 interface/mock and P4 glue,
guard and test code are Apache-2.0.

## User-supplied forklift CAD

The SolidWorks export originally named URDF20 is integrated into
`src/forklift_description`. Original export data is retained under `cad_source`;
active meshes are renamed under `meshes` without scaling. The original package
metadata declares BSD. A full pre-merge copy, including that metadata, is retained
in `results/model_layout/urdf20_before_merge.tar.gz`.
