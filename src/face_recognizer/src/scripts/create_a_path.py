import pathlib

def write_file():
    path = pathlib.Path.cwd() / "Roberto"
    #print(path.exists())
    path.mkdir(exist_ok=True)
    #path.mkdir()


    with open(path / "filename.txt", "w+") as f:
        f.write("Hello World!")


if __name__ == "__main__":
    write_file()