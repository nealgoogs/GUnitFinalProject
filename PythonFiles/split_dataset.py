import os
import random
import shutil

SOURCE_DIR = "dataset/frames"

# Output folders
OUT_DIR = "dataset"
IMAGES_DIR = os.path.join(OUT_DIR, "images")
LABELS_DIR = os.path.join(OUT_DIR, "labels")

TRAIN_IMG_DIR = os.path.join(IMAGES_DIR, "train")
VAL_IMG_DIR = os.path.join(IMAGES_DIR, "val")
TEST_IMG_DIR = os.path.join(IMAGES_DIR, "test")

TRAIN_LABEL_DIR = os.path.join(LABELS_DIR, "train")
VAL_LABEL_DIR = os.path.join(LABELS_DIR, "val")
TEST_LABEL_DIR = os.path.join(LABELS_DIR, "test")

# Create output directories if they don't exist
for d in [
    TRAIN_IMG_DIR,
    VAL_IMG_DIR,
    TEST_IMG_DIR,
    TRAIN_LABEL_DIR,
    VAL_LABEL_DIR,
    TEST_LABEL_DIR,
]:
    os.makedirs(d, exist_ok=True)

# Collect every frame image and remember its extension
files = []
for f in os.listdir(SOURCE_DIR):
    if f.lower().endswith((".jpg", ".png")):
        name, ext = os.path.splitext(f)
        files.append((name, ext))

# Shuffle file list
random.shuffle(files)

# 70/15/15 split
total = len(files)
train_split = int(total * 0.70)
val_split = int(total * 0.15)

train_files = files[:train_split]
val_files = files[train_split:train_split + val_split]
test_files = files[train_split + val_split:]


def move_files(file_list, image_dir, label_dir):
    for name, ext in file_list:
        img_src = os.path.join(SOURCE_DIR, name + ext)
        img_dst = os.path.join(image_dir, name + ext)
        shutil.copy(img_src, img_dst)

        label_src = os.path.join(SOURCE_DIR, name + ".txt")
        label_dst = os.path.join(label_dir, name + ".txt")
        if os.path.exists(label_src):
            shutil.copy(label_src, label_dst)
        else:
            # Create an empty annotation file to mark this frame as background
            open(label_dst, "w").close()


# Copy files into new folders
move_files(train_files, TRAIN_IMG_DIR, TRAIN_LABEL_DIR)
move_files(val_files, VAL_IMG_DIR, VAL_LABEL_DIR)
move_files(test_files, TEST_IMG_DIR, TEST_LABEL_DIR)

print("Done!")
print(f"Train: {len(train_files)} images")
print(f"Val:   {len(val_files)} images")
print(f"Test:  {len(test_files)} images")
