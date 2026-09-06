## Introduction

The slides are written in `Markdown` language and converter to slides using [remark](https://github.com/gnab/remark) tool. Please see the [documentation](https://github.com/gnab/remark/wiki) for further details.

## File structure
Slides are structured as follows for each session:
`source`: html files
`static`: Images, videos and other resource files

## Usage
### Build with Docker Compose

Docker is the only local dependency. Node.js, Chromium, and DeckTape run inside
the container.

From the `slides` directory, run:

```bash
HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose run --rm slides
```

The command exports all slide decks listed in `../export_list.yaml` to the
`../export` directory, grouped into `Day1`, `Day2`, and `Day3` subdirectories.
