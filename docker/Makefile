# Makefile for OrcaDesktops Docker stack (zenoh, ROS 2, InfluxDB, Grafana, ZoneMinder)
# Place this file next to docker-compose.yml:
#   /home/rover2/OrcaDesktops/docker/Makefile
#
# Usage examples:
#   make help
#   make install        # install docker, prepare dirs, build images
#   make up             # start all services
#   make down           # stop all services
#   make restart        # restart all services
#   make logs           # follow logs from all containers
#   make uninstall      # stop + remove containers/images + local config (safe)

SHELL := /bin/bash
PROJECT_DIR := $(shell pwd)
DOCKER_COMPOSE := docker compose -f $(PROJECT_DIR)/docker-compose.yml

# Adjust if you ever move the ZM storage mount point
ZM_DISK_MOUNT := /mnt/zm_sda1

.PHONY: help install install-docker init-dirs build up start restart stop down logs status ps shell zoneminder-shell clean-volumes uninstall uninstall-hard

help:
	@echo "Available targets:"
	@echo "  make install          - Install Docker & prepare directories, then build images"
	@echo "  make build            - Build all images"
	@echo "  make up               - Start all services in background"
	@echo "  make down             - Stop all services"
	@echo "  make restart          - Restart all services"
	@echo "  make stop             - Alias for down"
	@echo "  make logs             - Tail logs from all services"
	@echo "  make status / ps      - Show running containers for this compose"
	@echo "  make shell            - Open a shell in the ros2-cli container"
	@echo "  make zoneminder-shell - Open a shell in the zoneminder container"
	@echo "  make clean-volumes    - Remove local data dirs (influx/grafana/ZM host dirs)"
	@echo "  make uninstall        - Stop stack, remove containers + images (but keep ZM disk)"
	@echo "  make uninstall-hard   - Also remove ZM data on $(ZM_DISK_MOUNT) (DANGEROUS)"

###############################################################################
# Installation & preparation
###############################################################################

install: install-docker init-dirs build
	@echo "✅ Install complete. You can now run: make up"

install-docker:
	@echo "▶ Checking for Docker..."
	@if ! command -v docker >/dev/null 2>&1; then \
	  echo '  ❌ Docker not found. Installing docker.io...'; \
	  sudo apt-get update && sudo apt-get install -y docker.io; \
	  sudo systemctl enable --now docker; \
	else \
	  echo '  ✅ Docker already installed.'; \
	fi
	@echo "▶ Checking for docker compose plugin..."
	@if ! docker compose version >/dev/null 2>&1; then \
	  echo '  ❌ docker compose plugin not found. Installing docker-compose-plugin...'; \
	  sudo apt-get update && sudo apt-get install -y docker-compose-plugin; \
	else \
	  echo '  ✅ docker compose plugin already installed.'; \
	fi

init-dirs:
	@echo "▶ Preparing local data directories (InfluxDB, Grafana, ZM config)..."
	@mkdir -p $(PROJECT_DIR)/influxdb/data
	@mkdir -p $(PROJECT_DIR)/influxdb/config
	@mkdir -p $(PROJECT_DIR)/grafana/data
	@mkdir -p $(PROJECT_DIR)/grafana/provisioning
	@mkdir -p $(PROJECT_DIR)/zoneminder

	@echo "▶ Checking ZoneMinder disk mount: $(ZM_DISK_MOUNT)"
	@if ! grep -qs '$(ZM_DISK_MOUNT) ' /proc/mounts; then \
	  echo '  ⚠ $(ZM_DISK_MOUNT) is not mounted. Make sure /dev/sda1 is mounted there via /etc/fstab.'; \
	  echo '    Example /etc/fstab line:'; \
	  echo '      UUID=cc92be0e-a1f9-44d3-bbd2-9c34281f4b27  $(ZM_DISK_MOUNT)  ext4  defaults,noatime  0  2'; \
	  echo '  Aborting init-dirs to avoid writing to wrong disk.'; \
	  exit 1; \
	fi

	@echo "▶ Creating ZoneMinder data directories on $(ZM_DISK_MOUNT)..."
	@sudo mkdir -p $(ZM_DISK_MOUNT)/events $(ZM_DISK_MOUNT)/images $(ZM_DISK_MOUNT)/logs
	@sudo chown -R $$(id -u):$$(id -g) $(ZM_DISK_MOUNT)
	@echo "✅ Directories ready."

###############################################################################
# Build & lifecycle
###############################################################################

build:
	@echo "▶ Building Docker images for all services..."
	@$(DOCKER_COMPOSE) build
	@echo "✅ Build finished."

up start:
	@echo "▶ Starting all services with docker compose..."
	@$(DOCKER_COMPOSE) up -d
	@$(DOCKER_COMPOSE) ps

down stop:
	@echo "▶ Stopping all services..."
	@$(DOCKER_COMPOSE) down

restart:
	@echo "▶ Restarting all services..."
	@$(DOCKER_COMPOSE) down
	@$(DOCKER_COMPOSE) up -d
	@$(DOCKER_COMPOSE) ps

logs:
	@echo "▶ Tailing logs (Ctrl+C to stop)..."
	@$(DOCKER_COMPOSE) logs -f

status ps:
	@$(DOCKER_COMPOSE) ps

###############################################################################
# Convenience shells
###############################################################################

shell:
	@echo "▶ Opening shell in ros2-cli container (if running)..."
	@docker exec -it ros2-cli bash || echo '⚠ ros2-cli is not running.'

zoneminder-shell:
	@echo "▶ Opening shell in zoneminder container (if running)..."
	@docker exec -it zoneminder bash || echo '⚠ zoneminder is not running.'

###############################################################################
# Cleanup & uninstall
###############################################################################

clean-volumes:
	@echo "⚠ This will remove local InfluxDB/Grafana/ZM config directories under $(PROJECT_DIR)."
	@read -p 'Type "yes" to proceed: ' ans; \
	if [ "$$ans" = "yes" ]; then \
	  echo '▶ Removing local data dirs (influxdb, grafana, zoneminder - host side only)...'; \
	  rm -rf $(PROJECT_DIR)/influxdb/data $(PROJECT_DIR)/influxdb/config; \
	  rm -rf $(PROJECT_DIR)/grafana/data $(PROJECT_DIR)/grafana/provisioning; \
	  rm -rf $(PROJECT_DIR)/zoneminder; \
	  echo '✅ Local data directories removed (ZM disk on $(ZM_DISK_MOUNT) NOT touched).'; \
	else \
	  echo '❎ Aborted.'; \
	fi

uninstall:
	@echo "⚠ This will stop the stack and remove containers and images for this compose."
	@read -p 'Type "yes" to proceed with uninstall: ' ans; \
	if [ "$$ans" = "yes" ]; then \
	  echo '▶ Stopping stack...'; \
	  $(DOCKER_COMPOSE) down; \
	  echo '▶ Removing images used by this compose (may take some time)...'; \
	  docker image prune -f; \
	  echo '✅ Uninstall complete (ZM data on $(ZM_DISK_MOUNT) is preserved).'; \
	else \
	  echo '❎ Aborted.'; \
	fi

uninstall-hard:
	@echo "‼ DANGER: This will ALSO remove ZoneMinder data on $(ZM_DISK_MOUNT)/{events,images,logs}."
	@read -p 'Type "DELETE-ALL" to ERASE ALL ZM DATA: ' ans; \
	if [ "$$ans" = "DELETE-ALL" ]; then \
	  echo '▶ Stopping stack...'; \
	  $(DOCKER_COMPOSE) down; \
	  echo '▶ Removing images (may take time)...'; \
	  docker image prune -f; \
	  echo '▶ Removing ZM data from $(ZM_DISK_MOUNT)...'; \
	  sudo rm -rf $(ZM_DISK_MOUNT)/events $(ZM_DISK_MOUNT)/images $(ZM_DISK_MOUNT)/logs; \
	  echo '✅ Hard uninstall complete. All ZM video data removed.'; \
	else \
	  echo '❎ Aborted.'; \
	fi

cloud:
	@echo "▶ Ensuring ros2-cli container is running..."
	@docker compose -f $(PROJECT_DIR)/docker-compose.yml up -d ros2_cli
	@echo "▶ Starting RMF Cloud server inside ros2-cli..."
	@docker exec -d ros2-cli bash -lc '\
	  set -e; \
	  source /opt/ros/$$ROS_DISTRO/setup.bash; \
	  cd /root/colcon_ws; \
	  if [ -f install/setup.bash ]; then source install/setup.bash; fi; \
	  export ROS_DOMAIN_ID=1; \
	  ros2 launch rmf_manager_cloud server.launch.py \
	'
	@echo "🌍 RMF Cloud server started (if no errors above)."

cloud-stop:
	@echo "🛑 Stopping RMF Cloud server..."
	@docker exec -it ros2-cli pkill -f rmf_manager_cloud || true
	@echo "🛑 Cloud server stopped."