# Dev Container Setup

Clone the repository in the `home/` directory without changing the name of the workspace folder.

```bash
git clone https://github.com/ENEE467/lab-workspace.git
```

## Building the Docker image

Start a terminal session and change to `.devcontainer/` directory

```bash
cd ~/lab-workspace/.devcontainer
```

Build the Docker image with the given arguments

```bash
docker build \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) \
  --build-arg USERNAME=467-terp \
  -t enee467/lab_ws_image \
  -f 467-lab.Dockerfile \
  .
```

## Starting the Dev Container

Now opening the workspace folder in VSCode with the Dev Container configuration will start the
container named after the workspace folder.

This page on [Read the Docs](https://enee467.readthedocs.io/en/latest/Setup.html#opening-the-workspace-in-visual-studio-code)
describes the steps in detail.
