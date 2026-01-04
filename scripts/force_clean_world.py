import re

filename = '/home/rocyu/my_ros_project/worlds/bookstore.world'
output_filename = '/home/rocyu/my_ros_project/worlds/bookstore_clean.world'

with open(filename, 'r') as f:
    content = f.read()

# Regex ile turtlebot3_burger modelini bul ve sil
# <model name='turtlebot3_burger'> ... </model> bloğunu siler
# DOTALL flag'i ile nokta karakteri yeni satırları da kapsar
pattern = r"<model name='turtlebot3_burger'>.*?</model>"
clean_content = re.sub(pattern, "", content, flags=re.DOTALL)

with open(output_filename, 'w') as f:
    f.write(clean_content)

print(f"Cleaned world saved to {output_filename}")
