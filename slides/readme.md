## Introduction

The slides are written in `Markdown` language and converter to slides using [remark](https://github.com/gnab/remark) tool. Please see the [documentation](https://github.com/gnab/remark/wiki) for further details.

## File structure
Slides are structured as follows for each session:
`source`: html files
`static`: Images, videos and other resource files

## Usage
### Visualize the slides in the browser

The slides are already HTML files; no build or virtual environment is needed.
From the repository root, run:

```sh
python3 -m http.server 8000 --bind 127.0.0.1 --directory slides
```

Open [the slide index](http://localhost:8000/) and select a presentation.
Internet access is required to load remark.js.

### GitHub Pages

The documentation workflow publishes the [slide index](index.html), slide decks,
and their assets alongside the workshop when changes are pushed to `main`.
On the published site, open `slides/` or follow the **Training slides** link
from the workshop homepage.

### Build the pdf slides with Docker Compose

Docker is the only local dependency. Node.js, Chromium, and DeckTape run inside
the container.

From the `slides` directory, run:

```bash
HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose run --rm slides
```

The command exports all slide decks listed in `../export_list.yaml` to the
`../export` directory, grouped into `Day1`, `Day2`, and `Day3` subdirectories.
