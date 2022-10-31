import subprocess
import pathlib
import shutil
import os


def build_react_dashboard():
    current_dir = pathlib.Path(os.getcwd())
    while current_dir.name != "src":
        current_dir = current_dir.parent
    target_dir = current_dir.parent.joinpath("dashboard")

    if target_dir.is_dir():
        print("Dashboard already exists. Cancelling...")
        return

    ui_dir = pathlib.Path(os.getcwd(), "ui")

    print("Installing dependencies")
    output = subprocess.check_output(["npm", "install"], cwd=ui_dir)
    print(output)

    print("Building react dashboard")
    output = subprocess.check_output(["npm", "run", "build"], cwd=ui_dir)
    print(output)

    print("Copying react dashboard")
    shutil.copytree(ui_dir.joinpath("build"), target_dir)


if __name__ == "__main__":
    build_react_dashboard()
