#!/bin/bash
# ME48B Library Robot - Save Map Script
# Saves the current GMapping map to the maps directory

MAP_NAME=${1:-library_map}
MAP_DIR="$(rospack find me48b_library_robot)/maps"

echo "Saving map as: $MAP_NAME"
echo "Directory: $MAP_DIR"

rosrun map_server map_saver -f "$MAP_DIR/$MAP_NAME"

echo ""
echo "Map saved successfully!"
echo "Files created:"
echo "  - $MAP_DIR/$MAP_NAME.yaml"
echo "  - $MAP_DIR/$MAP_NAME.pgm"
