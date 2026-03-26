---
paths:
  - "src/**/*"
  - "**/CMakeLists.txt"
  - "**/package.xml"
  - "**/*.launch"
  - "**/*.rviz"
  - "**/*.msg"
  - "**/*.cfg"
---

# ROS workspace

- This repository is a ROS Noetic catkin workspace.
- Verify package boundaries before editing and keep commands, package names, and launch references exact.
- Prefer minimal, local changes inside the relevant package.
- Do not introduce alternate build systems or flatten package structure.
- If launch or package behavior changes, update the matching docs.
