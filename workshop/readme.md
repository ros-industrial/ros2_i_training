## Introduction

The workshop documents are written in `Markdown` language and built using [Sphinx](https://docs.readthedocs.io/en/stable/intro/getting-started-with-sphinx.html?) documentation generator.

## Generate the docs
### Install dependencies

#### Clone the repository
```bash
git clone <repo-name>
```

#### Setup the virtual environment in the workshop folder to generate the docs
Navigate to the workshop folder
```bash
cd ros2_i_training/workshop
```

Create a virtual environment with `uv` and activate it
```bash
uv venv .venv
source .venv/bin/activate
```

Install the dependencies from `requirements.txt` using `uv`
```bash
uv pip install -r requirements.txt
```

### Build the html doc
From the workshop directory:
 ```bash
 make html
 ```

### Open the docs
From the workshop directory:
```bash
xdg-open build/html/index.html
```

or simply preview the result using uv:
```bash
uv run python -m http.server 8000 --directory build/html
```

### To add new content
Place the content source in the following folders in the appropriate session.
-  `_source`: Markup (.md) files
- `_static`: Images and other resource files

Add the workshop's heading and filepath relative to `~/ros2_i_training/workshop/source/_source` to `index.rst`. To build the html:
 ````bash
 cd ~/ros2_i_training/workshop/
 make html
 ````
`index.rst` is built into `index.html` in the documentation output directory `~/ros2_i_training/workshop/build/html/index.html`.

![docs](/workshop/source/_static/demo_rtd.png)

