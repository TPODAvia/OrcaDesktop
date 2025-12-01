# Makefile to control docker-compose stack in ./docker

DC := docker compose
COMPOSE_FILE := docker/docker-compose.yml
PROJECT_NAME := orca

# Images that come from registries (no local build)
PULL_SERVICES := zenohd zenoh_bridge_ros2dds influxdb grafana go2rtc frigate portainer

.PHONY: install run stop remove down uninstall

install:
	# Pull only external images
	$(DC) -f $(COMPOSE_FILE) -p $(PROJECT_NAME) pull $(PULL_SERVICES)
	# Build local image for ros2_cli from Dockerfile
	$(DC) -f $(COMPOSE_FILE) -p $(PROJECT_NAME) build ros2_cli

run:
	$(DC) -f $(COMPOSE_FILE) -p $(PROJECT_NAME) up -d

stop:
	$(DC) -f $(COMPOSE_FILE) -p $(PROJECT_NAME) stop

# remove containers (docker compose down), but keep volumes
remove:
	$(DC) -f $(COMPOSE_FILE) -p $(PROJECT_NAME) down

down:
	$(DC) -f $(COMPOSE_FILE) -p $(PROJECT_NAME) down

# uninstall everything: containers + volumes + orphans
uninstall:
	$(DC) -f $(COMPOSE_FILE) -p $(PROJECT_NAME) down -v --remove-orphans
