
# Raspberry Pi Cross-Compilation Template for Rust + C + OpenBLAS

This repository provides a **turnkey solution** for building high-performance Rust applications for Raspberry Pi (32-bit and 64-bit) with:

✅ Rust + C integration  
✅ Static linking with OpenBLAS  
✅ Cross-compilation using Docker  
✅ GitHub Actions CI/CD for automated builds  
✅ Semantic versioning and changelog generation  

---

## 🚀 Features
- **Rust project** with FFI bindings to OpenBLAS and a custom C ODE solver.
- **Multi-architecture support**: Raspberry Pi 4 (64-bit) and Pi 3/4 (32-bit).
- **Static linking**: No external dependencies required on the Pi.
- **Automated releases**: Builds and publishes binaries to GitHub Releases.
- **Semantic versioning**: Auto version bump and changelog based on commits.

---

## 📦 Prerequisites
- https://docs.docker.com/get-docker/
- https://www.rust-lang.org/tools/install
- https://cli.github.com/ (optional for repo setup)

---

## 🛠 Quick Start

### 1. Clone the Repository
```bash
git clone https://github.com/<your-username>/raspi-cross-template.git
cd raspi-cross-template
```

## Build Locally (Optional)
To cross-compile locally using Docker:
```bash

docker build -f Dockerfile.aarch64-unknown-linux-gnu -t my-cross-aarch64 .
docker run --rm -v $(pwd):/project my-cross-aarch64 \
    bash -c "cd /project && cargo build --release --target aarch64-unknown-linux-gnu"
```

## Deploy to Raspberry Pi
```bash

scp target/aarch64-unknown-linux-gnu/release/my-numerics pi@raspberrypi:/home/pi/
ssh pi@raspberrypi ./my-numerics
```

## 🔄 CI/CD Workflows

- Build & Release: Trigger manually via GitHub Actions → Builds binaries for both architectures and attaches them to a GitHub Release.
 - Semantic Release: Runs on every push to main → Auto version bump, changelog update, and release creation.

## ✅ How to Trigger a Release

- Push commits using Conventional Commits:
   - feat: → minor bump
   - fix: → patch bump
   - BREAKING CHANGE: → major bump


- GitHub Actions will:
   - Generate changelog
   - Create a new version tag
   - Build and publish binaries 