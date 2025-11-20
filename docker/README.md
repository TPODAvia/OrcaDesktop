How to run

cd ~/Docker
docker compose build
docker compose up -d zenohd zenoh_bridge_ros2dds
# One-shot check:
docker compose run --rm ros2_cli

docker compose run ros2_cli


sudo apt install zenohd zenoh-plugin-ros2dds
pip install -U zenoh-cli
zenoh-cli sub -e tcp/HOST_A_IP:7447 '/yhs_yhsros2/**'


sudo chown -R 472:472 ./grafana/data
sudo chown -R 472:472 ./grafana/provisioning





export INFLUX_TOKEN=replace_with_a_long_random_token
curl -X POST "http://127.0.0.1:8086/api/v2/delete?org=orca&bucket=rmf" \
  -H "Authorization: Token $INFLUX_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "start":"1970-01-01T00:00:00Z",
    "stop":"2100-01-01T00:00:00Z",
    "predicate":""
  }'


  GST_DEBUG=2 ROS_DOMAIN_ID=1 ros2 launch rmf_manager_cloud server.launch.py