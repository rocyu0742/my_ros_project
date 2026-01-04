import re

filename = '/home/rocyu/my_ros_project/worlds/bookstore.world'

with open(filename, 'r') as f:
    content = f.read()

# Regex: <model name='DeskA_01_005'> ... </model> bloğunu sil
# DOTALL flag'i ile nokta karakteri yeni satırları da kapsar
pattern = r"<model name=['\"]DeskA_01_005['\"]>.*?</model>"
clean_content = re.sub(pattern, "", content, flags=re.DOTALL)

if len(content) != len(clean_content):
    with open(filename, 'w') as f:
        f.write(clean_content)
    print("Successfully removed DeskA_01_005 from world file.")
else:
    print("Model DeskA_01_005 not found!")
