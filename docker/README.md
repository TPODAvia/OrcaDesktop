How to run

cd ~/Docker
docker compose build
docker compose up -d zenohd zenoh_bridge_ros2dds
# One-shot check:
docker compose run --rm ros2_cli

docker compose run ros2_cli


sudo apt install zenohd zenoh-plugin-ros2dds
pip install -U zenoh-cli
zenoh-cli sub -e tcp/HOST_A_IP:7447 '/vboxuser_Ubuntu22/**'