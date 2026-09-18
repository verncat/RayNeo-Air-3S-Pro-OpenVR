import json
import os
from pathlib import Path
import re
import subprocess
import uuid


VERSION_HEADER = "include/rayneo_api.h"


SEMVER = re.compile(
    r"(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)"
    r"(?:-((?:0|[1-9][0-9]*|[0-9]*[A-Za-z-][0-9A-Za-z-]*)"
    r"(?:\.(?:0|[1-9][0-9]*|[0-9]*[A-Za-z-][0-9A-Za-z-]*))*))?"
    r"(?:\+[0-9A-Za-z-]+(?:\.[0-9A-Za-z-]+)*)?"
)


def git(*args):
    return subprocess.check_output(["git", *args], text=True, encoding="utf-8").strip()


def version_from(text, allow_missing=False):
    text = re.sub(r"/\*.*?\*/|//[^\r\n]*", "", text, flags=re.DOTALL)
    definitions = {}
    for component in ("MAJOR", "MINOR", "PATCH"):
        definitions[component] = re.findall(
            rf"^[ \t]*#[ \t]*define[ \t]+RAYNEO_API_VERSION_{component}\b([^\r\n]*)",
            text, re.MULTILINE,
        )
    # Older headers had no patch component and were not release metadata.
    if not definitions["PATCH"] and allow_missing:
        return None
    components = []
    for component, values in definitions.items():
        value = re.fullmatch(r"[ \t]+(0|[1-9][0-9]*)[ \t]*", values[0]) if len(values) == 1 else None
        if value is None:
            raise ValueError(
                f"{VERSION_HEADER} must contain exactly one decimal integer "
                f"RAYNEO_API_VERSION_{component} (no leading zeros)"
            )
        if component != "PATCH" and int(value[1]) > 65535:
            raise ValueError(f"RAYNEO_API_VERSION_{component} must fit in 16 bits")
        components.append(value[1])
    return ".".join(components)


def main():
    version = version_from(Path(VERSION_HEADER).read_text(encoding="utf-8"))
    prefix = os.environ["TAG_PREFIX"]
    tag = f"{prefix}v{version}"
    event = json.loads(Path(os.environ["GITHUB_EVENT_PATH"]).read_text(encoding="utf-8"))
    before = event.get("before")
    if not before or set(before) == {"0"}:
        before = git("rev-parse", "HEAD^") if git("rev-list", "--count", "HEAD") != "1" else None
    previous_version = None
    if before and VERSION_HEADER in git("ls-tree", "--name-only", before, "--", VERSION_HEADER).splitlines():
        previous_version = version_from(git("show", f"{before}:{VERSION_HEADER}"), allow_missing=True)
    changed = version != previous_version and tag not in git("tag", "--list").splitlines()
    body = ""
    if changed:
        tags = git("tag", "--merged", "HEAD").splitlines()
        # Include legacy timestamp tags and the SDK's original v-prefixed tags.
        candidates = [t for t in tags if t.startswith(prefix) or (
            prefix == "sdk-" and t.startswith("v") and SEMVER.fullmatch(t[1:])
        )]
        previous_tag = git("describe", "--tags", "--abbrev=0", *[
            arg for candidate in candidates for arg in ("--match", candidate)
        ], "HEAD") if candidates else None
        commit_range = f"{previous_tag}..HEAD" if previous_tag else "HEAD"
        url = f"{os.environ['GITHUB_SERVER_URL']}/{os.environ['GITHUB_REPOSITORY']}"
        lines = [f"## Changes in {tag}", ""]
        for commit in git("log", "--reverse", "--format=%H %s", commit_range).splitlines():
            sha, subject = commit.split(" ", 1)
            lines.append(f"- {subject} ([{sha[:7]}]({url}/commit/{sha}))")
        if previous_tag:
            lines += ["", f"[Full diff]({url}/compare/{previous_tag}...{tag})"]
        body = "\n".join(lines)
    outputs = {
        "changed": str(changed).lower(), "version": version, "tag": tag,
        "prerelease": str(SEMVER.fullmatch(version).group(4) is not None).lower(),
        "body": body,
    }
    with open(os.environ["GITHUB_OUTPUT"], "a", encoding="utf-8") as output:
        for key, value in outputs.items():
            delimiter = uuid.uuid4().hex
            output.write(f"{key}<<{delimiter}\n{value}\n{delimiter}\n")
    print(f"Version: {version}; previous: {previous_version}; build: {changed}")


if __name__ == "__main__":
    main()
