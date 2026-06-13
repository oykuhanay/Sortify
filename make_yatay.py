from pathlib import Path
from PIL import Image, ImageOps

folder = Path("/Users/apple/Desktop/Sortify/w_robot")  # change this to your folder path

valid_extensions = {".jpg", ".jpeg", ".png", ".webp"}

for file_path in folder.iterdir():
    if file_path.suffix.lower() not in valid_extensions:
        continue

    try:
        img = Image.open(file_path)

        # Apply EXIF orientation correctly
        img = ImageOps.exif_transpose(img)

        width, height = img.size

        # If photo is vertical, rotate it to horizontal
        if height > width:
            img = img.rotate(90, expand=True)
            img.save(file_path)
            print(f"Rotated: {file_path.name}")
        else:
            print(f"Already horizontal: {file_path.name}")

    except Exception as e:
        print(f"Error with {file_path.name}: {e}")