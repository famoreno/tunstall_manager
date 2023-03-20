import pathlib

def rename_photos():
    path = pathlib.Path('.') / "faces_dataset"
    for folder in path.iterdir():
        #print(folder)
        if folder.is_dir():
           counter = 1
           for file in folder.iterdir():
                if file.is_file():
                    new_file = str(counter) + file.suffix
                    file.rename(path / folder.name/ new_file)
                    counter += 1


if __name__ == "__main__":
    rename_photos()
