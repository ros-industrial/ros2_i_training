# Linux and terminal basics

The terminal is a text-based way to interact with the computer. ROS 2
development uses terminals to navigate directories, build workspaces, source
environments, and start nodes.

## Essential commands

### Show the current directory

```bash
pwd
```

### List files and directories

```bash
ls
ls -l
```

`ls -l` also displays permissions, ownership, size, and modification time.

### Change directory

```bash
cd ~/ros2_workshop
cd ..
cd -
```

- `..` is the parent directory.
- `~` is your home directory.
- `cd -` returns to the previous directory.

### Create directories

```bash
mkdir my_directory
mkdir -p dev_ws/src
```

The `-p` option creates missing parent directories.

### Copy and move files

```bash
cp source.txt destination.txt
mv old_name.txt new_name.txt
```

`mv` can rename a file or move it into another directory.

### Create and inspect files

```bash
touch notes.txt
less notes.txt
```

Press `q` to leave `less`.

## Permissions and scripts

The permission string shown by `ls -l` may look like:

```text
-rw-rw-r-- 1 user user 29 Apr 13 10:00 hello_world.sh
```

The three permission groups apply to the owner, group, and others:

- `r`: read
- `w`: write
- `x`: execute
- `-`: permission not granted

Make a script executable and run it:

```bash
chmod +x hello_world.sh
./hello_world.sh
```

A script can also be passed directly to Bash:

```bash
bash hello_world.sh
```

## Using sudo safely

`sudo` runs a command with administrator privileges. It is normally needed
for system-wide actions such as installing packages:

```bash
sudo apt update
```

Do not use `sudo` for ordinary work in your home directory. A mistaken
administrator command can modify or damage the system.

## Useful terminal features

- Press `Tab` to complete command and file names.
- Press the up and down arrows to browse command history.
- Press `Ctrl+C` to interrupt a running foreground command.
- Use `clear` or press `Ctrl+L` to clear the terminal display.
- Run `pwd` when you are uncertain about the current directory.

## Exercise

1. Create `~/ros2_workshop/linux_exercise`.
2. Inside it, create the directories `source` and `workspace/src`.
3. In `source`, create `hello_world.sh` with this content:

   ```bash
   #!/usr/bin/env bash

   echo "Hello world"
   ```

4. Copy the script into `workspace/src`.
5. Make the copied script executable.
6. Run it from `workspace/src`.

### Solution

```bash
mkdir -p ~/ros2_workshop/linux_exercise/source
mkdir -p ~/ros2_workshop/linux_exercise/workspace/src
cd ~/ros2_workshop/linux_exercise/source
nano hello_world.sh
cp hello_world.sh ../workspace/src/
cd ../workspace/src
chmod +x hello_world.sh
./hello_world.sh
```

Expected output:

```text
Hello world
```
