from pathlib import Path
import random
import shutil

random.seed(42)

SRC = Path("data")
DST = Path("dataset")

train_ratio = 0.8

for split in ["train", "val"]:
    for cls in ["clean", "dirty"]:
        (DST / split / cls).mkdir(parents=True, exist_ok=True)

for cls in ["clean", "dirty"]:
    imgs = list((SRC / cls).glob("*.*"))
    random.shuffle(imgs)

    split_idx = int(len(imgs) * train_ratio)
    train_imgs = imgs[:split_idx]
    val_imgs = imgs[split_idx:]

    for p in train_imgs:
        shutil.copy(p, DST / "train" / cls / p.name)

    for p in val_imgs:
        shutil.copy(p, DST / "val" / cls / p.name)

print("Dataset split complete")
