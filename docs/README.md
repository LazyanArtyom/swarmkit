# Building SwarmKit Documentation

The documentation site uses Doxygen for C++ XML, Sphinx for the site, Breathe
for API import, MyST for Markdown pages, and Furo as the HTML theme.

Install Doxygen first:

```bash
# macOS
brew install doxygen

# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y doxygen graphviz
```

Build the site:

```bash
cd docs
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
make html
open build/html/index.html
```

`make html` runs Doxygen first and writes XML to `build/doxygen/xml`, then
builds the Sphinx site into `build/html`.
