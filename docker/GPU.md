Nice, `nvidia-smi` works — so the driver side is fine. Now we just need to teach **Docker** how to use the GPU.

Below is a **clean step-by-step for Ubuntu + Docker Engine** (works fine with Docker Desktop on Linux too).

---

## 1. Install NVIDIA Container Toolkit

Run these in a terminal on `rover2`:

```bash
# 1) Add the NVIDIA Container Toolkit repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit.gpg

distribution=$(. /etc/os-release; echo $ID$VERSION_ID)  # e.g. ubuntu20.04 or ubuntu22.04
curl -fsSL https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

Then install the toolkit:

```bash
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
```

---

## 2. Configure Docker to use the NVIDIA runtime

Tell Docker about the `nvidia` runtime:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
```

This will add the runtime to `/etc/docker/daemon.json`.
If for some reason it didn’t, you can also create/edit it manually:

```bash
sudo nano /etc/docker/daemon.json
```

Put something like this (merge with existing JSON if you already have one):

```json
{
  "runtimes": {
    "nvidia": {
      "path": "nvidia-container-runtime",
      "runtimeArgs": []
    }
  }
}
```

Then restart Docker:

```bash
sudo systemctl restart docker
```

---

## 3. Test that Docker sees the GPU

Run a test CUDA container:

```bash
docker run --rm --gpus all nvidia/cuda:12.2.0-base nvidia-smi
```

If everything is configured, you should see the same kind of `nvidia-smi` table **from inside the container**.

If this works → Docker + GPU is OK ✅

---

## 4. Make your `frigate` service work with GPU

Your `frigate` service already has the **Compose v3 “deploy.resources.devices”** block and env vars:

```yaml
  frigate:
    container_name: frigate
    image: ghcr.io/blakeblackshear/frigate:stable-tensorrt
    privileged: true
    restart: unless-stopped
    shm_size: "2g"
    network_mode: host

    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]

    environment:
      NVIDIA_VISIBLE_DEVICES: "all"
      NVIDIA_DRIVER_CAPABILITIES: "compute,utility,video"
```

With the NVIDIA runtime installed, **you don’t need to change this** — just re-run:

```bash
docker compose up -d
```

and `frigate` should start instead of giving:

> could not select device driver "nvidia" with capabilities: [[gpu]]

---

## 5. If `docker run --gpus all …` works but `frigate` still fails

Then we can try a simpler Compose syntax that mimics the CLI:

```yaml
  frigate:
    container_name: frigate
    image: ghcr.io/blakeblackshear/frigate:stable-tensorrt
    privileged: true
    restart: unless-stopped
    shm_size: "2g"
    network_mode: host

    # This is the compose equivalent of: --gpus all
    deploy:
      resources:
        reservations:
          devices:
            - capabilities: [gpu]

    environment:
      NVIDIA_VISIBLE_DEVICES: "all"
      NVIDIA_DRIVER_CAPABILITIES: "compute,utility,video"
```

Or (older Docker/Compose) explicitly set default runtime:

```json
// /etc/docker/daemon.json
{
  "default-runtime": "nvidia",
  "runtimes": {
    "nvidia": {
      "path": "nvidia-container-runtime",
      "runtimeArgs": []
    }
  }
}
```

Then restart Docker again.

---

If you paste the output of:

```bash
docker run --rm --gpus all nvidia/cuda:12.2.0-base nvidia-smi
```

and one `docker compose up frigate` log, I can pinpoint the last 1–2 missing tweaks for your exact setup.
