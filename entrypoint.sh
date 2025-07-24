#!/bin/bash

# Example setup task: Add an alias to .bashrc
echo 'alias enter_ros_container="docker exec -it ros_noetic_container /bin/bash"' >> ~/.bashrc

# Example setup task: Check if a certain file exists
if [ ! -f /root/.myapp_config ]; then
    echo "Configuration file not found, creating it..."
    touch /root/.myapp_config
fi

# Finally, run the default command provided by the container (or passed via CMD)
exec "$@"
