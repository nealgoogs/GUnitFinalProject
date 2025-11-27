import os
import random
import shutil

SOURCE_DIR = "dataset/frames"

# Output folders
OUT_DIR = "dataset"
TRAIN_DIR = os.path.join(OUT_DIR, "train")
VAL_DIR = os.path.join(OUT_DIR, "val")
TEST_DIR = os.path.join(OUT_DIR, "test")

# Create output directories if they don't exist
for d in [TRAIN_DIR, VAL_DIR, TEST_DIR]:
    os.makedirs(d, exist_ok=True)

# Collect only images that have BOTH .jpg and .txt
files = []
for f in os.listdir(SOURCE_DIR):
    if f.endswith(".jpg") or f.endswith(".png"):
        name = os.path.splitext(f)[0]
        txt_file = name + ".txt"
        if os.path.exists(os.path.join(SOURCE_DIR, txt_file)):
            files.append(name)

# Shuffle file list
random.shuffle(files)

# 70/15/15 split
total = len(files)
train_split = int(total * 0.70)
val_split = int(total * 0.15)

train_files = files[:train_split]
val_files = files[train_split:train_split + val_split]
test_files = files[train_split + val_split:]

def move_files(file_list, target_dir):
    for name in file_list:
        for ext in [".jpg", ".png", ".txt"]:
            src = os.path.join(SOURCE_DIR, name + ext)
            if os.path.exists(src):
                shutil.copy(src, os.path.join(target_dir, name + ext))

# Copy files into new folders
move_files(train_files, TRAIN_DIR)
move_files(val_files, VAL_DIR)
move_files(test_files, TEST_DIR)

print("Done!")
print(f"Train: {len(train_files)} images")
print(f"Val:   {len(val_files)} images")
print(f"Test:  {len(test_files)} images")
