#!/usr/bin/env python3
import qrcode
from PIL import Image
import os

TEXTURE_PATH = "/home/rocyu/my_ros_project/models/book_tags/materials/textures"

if not os.path.exists(TEXTURE_PATH):
    os.makedirs(TEXTURE_PATH)

books = ["SHELF_A", "SHELF_B", "SHELF_C"]

for book_id in books:
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=10,
        border=4,
    )
    qr.add_data(book_id)
    qr.make(fit=True)

    img = qr.make_image(fill_color="black", back_color="white")
    
    # Save as PNG
    filename = f"{TEXTURE_PATH}/{book_id}.png"
    img.save(filename)
    print(f"QR Code created: {filename}")

print("All QR codes generated successfully!")
