# Git and vcstool

Git records changes to files over time. It enables you to inspect history,
develop work on branches, and collaborate through a remote service such as
GitHub.

## Git concepts

| Term | Meaning |
| --- | --- |
| Repository | A project and its Git history |
| Commit | A saved snapshot with a message |
| Branch | A named line of development |
| Remote | A repository stored elsewhere, such as on GitHub |
| Working tree | The files currently checked out |
| Staging area | Changes selected for the next commit |

## Configure your identity

Git stores an author name and email in every commit:

```bash
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
```

Inspect the configuration:

```bash
git config --global --list
```

## Clone a repository

```bash
git clone https://github.com/ros-industrial/ros2_i_training.git
cd ros2_i_training
```

## Typical workflow

Check the current state:

```bash
git status
```

Create and switch to a branch:

```bash
git switch -c workshop-notes
```

Inspect unstaged changes:

```bash
git diff
```

Select changes and create a commit:

```bash
git add notes.txt
git commit -m "Add workshop notes"
```

Inspect the history:

```bash
git log --oneline
```

Download changes from the remote and upload local commits:

```bash
git pull
git push
```

Pushing requires permission to the remote repository. When contributing through
a fork, push the branch to your fork and open a pull request against the
upstream repository.

## HTTPS and SSH remotes

An HTTPS remote looks like:

```text
https://github.com/ros-industrial/ros2_i_training.git
```

An SSH remote looks like:

```text
git@github.com:ros-industrial/ros2_i_training.git
```

HTTPS is simple for read-only access. SSH is convenient for regular
authenticated pushes after an SSH key has been configured.

Inspect the configured remotes:

```bash
git remote -v
```

Change the URL of the remote named `origin`:

```bash
git remote set-url origin git@github.com:ros-industrial/ros2_i_training.git
```

## Good habits

- Run `git status` frequently.
- Work on a branch rather than directly on `main`.
- Keep commits focused on one logical change.
- Write commit messages that explain the change.
- Review `git diff` before committing.
- Pull recent upstream changes before beginning new work.

## Git exercise

This exercise uses a new local repository and does not require GitHub access:

```bash
mkdir -p ~/ros2_workshop/git_exercise
cd ~/ros2_workshop/git_exercise
git init
git switch -c workshop-notes
touch notes.txt
git status
git add notes.txt
git commit -m "Add workshop notes"
git log --oneline
```

If Git refuses to create the commit, configure your author name and email as
shown earlier on this page.

## Managing several repositories with vcstool

A ROS 2 workspace can depend on several Git repositories. `vcstool` reads a
`.repos` file and checks out each repository at the requested branch, tag, or
commit.

Install it if necessary:

```bash
sudo apt update
sudo apt install python3-vcstool
```

Example `dependencies.repos`:

```yaml
repositories:
  example_interfaces:
    type: git
    url: https://github.com/ros2/example_interfaces.git
    version: jazzy
```

Import the repositories into a workspace:

```bash
mkdir -p dev_ws/src
cd dev_ws
vcs import src < dependencies.repos
```

Inspect their versions and local changes:

```bash
vcs status src
```

The space before `<` is optional to Bash but is kept here for readability.
The operator redirects the contents of `dependencies.repos` into
`vcs import`.
