import os

# Folder containing images
folder_path = "cheackerImages/rectified3"  # Change this to your folder path

# Part of filename to change
old_part = "ColoredBalls"   # Change this to the text you want to replace
new_part = "checkerrect"   # Change this to the new text

# Supported image extensions
valid_extensions = (".jpg", ".png", ".jpeg", ".bmp", ".tiff")

# Iterate over files in the folder
for filename in os.listdir(folder_path):
    if filename.endswith(valid_extensions) and old_part in filename:
        new_filename = filename.replace(old_part, new_part)
        old_path = os.path.join(folder_path, filename)
        new_path = os.path.join(folder_path, new_filename)

        os.rename(old_path, new_path)
        print(f'Renamed: {filename} → {new_filename}')

print("Renaming complete!")