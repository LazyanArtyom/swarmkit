from pathlib import Path

project = "SwarmKit"
author = "Artyom Lazyan"
copyright = "2026, Artyom Lazyan"

docs_dir = Path(__file__).resolve().parent
repo_root = docs_dir.parent
version_file = repo_root / "VERSION"
release = version_file.read_text(encoding="utf-8").strip() if version_file.exists() else "0.0.0"
version = release

extensions = [
    "breathe",
    "myst_parser",
]

source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

primary_domain = "cpp"
highlight_language = "cpp"
templates_path = ["_templates"]
exclude_patterns = ["build", "Thumbs.db", ".DS_Store"]

breathe_projects = {
    "SwarmKit": str(docs_dir / "build" / "doxygen" / "xml"),
}
breathe_default_project = "SwarmKit"
breathe_domain_by_extension = {
    "h": "cpp",
    "cpp": "cpp",
}

html_theme = "furo"
html_title = f"SwarmKit {release}"
html_static_path = []

myst_heading_anchors = 3
myst_enable_extensions = [
    "colon_fence",
    "deflist",
]
