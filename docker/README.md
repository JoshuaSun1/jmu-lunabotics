# Container workflow

This directory defines the Phase 0 build-container scaffold. It locks the
target architecture and reported ROS/JetPack/ZED versions in `versions.env`.

## What this container proves

On a native arm64 Jetson, run:

```bash
docker compose --env-file docker/versions.env -f docker/docker-compose.yml build
```

The result establishes that the package scaffold can build on the requested
architecture. The current generic `ubuntu:24.04` base intentionally does not
claim to validate CUDA, L4T, ZED SDK, or camera access.

## Before sensor/runtime use

1. Verify JetPack 7.2.1 / L4T 39.2.1 and the Orin Nano model on the target.
2. Verify the actual L4T, CUDA, ZED SDK, and ZED wrapper combination on the
   physical Jetson and ZED Mini.
3. Record the actual base-image digest and wrapper commit in `versions.env`.
4. Replace the generic build base only with a validated Jetson runtime image.

Do not treat an x86 build or a generic arm64 Ubuntu build as ZED/Jetson runtime
validation.
