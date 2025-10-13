sudo apt install zenohd zenoh-plugin-ros2dds


curl -L https://github.com/eclipse-zenoh/zenoh/releases/latest/download/zenohd-x86_64-unknown-linux-gnu.tar.gz   | tar xz && sudo mv zenohd /usr/local/bin/
echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null
sudo apt update
sudo apt install zenoh-bridge-ros2dds

