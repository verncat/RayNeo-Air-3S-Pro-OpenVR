import contextlib
import importlib.util
import io
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


spec = importlib.util.spec_from_file_location("release_meta", Path(__file__).with_name("release_meta.py"))
meta = importlib.util.module_from_spec(spec)
spec.loader.exec_module(meta)


class ReleaseMetaTests(unittest.TestCase):
    def test_semver(self):
        for version in ("0.0.0", "1.2.3", "1.2.3-rc.1+build.42"):
            self.assertEqual(meta.version_from(f"VERSION={version}\n"), version)
        for version in ("v1.0.0", "1.0", "01.0.0", "1.0.0-01", "1.0.0+", ""):
            with self.assertRaises(ValueError):
                meta.version_from(f"VERSION={version}\n")
        with self.assertRaises(ValueError):
            meta.version_from("VERSION=1.0.0\nVERSION=2.0.0\n")

    def run_meta(self, old="1.0.0", new="1.1.0", tags=(), reachable=(), manual=False):
        calls = []

        def git(*args):
            calls.append(args)
            if args[:2] == ("rev-list", "--count"):
                return "3"
            if args[0] == "rev-parse":
                return "parent"
            if args[0] == "ls-tree":
                return ".env" if old is not None else ""
            if args[0] == "show":
                return f"VERSION={old}"
            if args[:2] == ("tag", "--list"):
                return "\n".join(tags)
            if args[:2] == ("tag", "--merged"):
                return "\n".join(reachable)
            if args[0] == "describe":
                return reachable[0]
            if args[0] == "log":
                return "abcdef0123456789 Example commit"
            raise AssertionError(args)

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "output"
            env = dict(TAG_PREFIX="sdk-", GITHUB_EVENT_PATH="event.json",
                       GITHUB_OUTPUT=str(output), GITHUB_SERVER_URL="https://github.com",
                       GITHUB_REPOSITORY="owner/repo")
            def read_text(path, **kwargs):
                return f"VERSION={new}" if str(path) == ".env" else (
                    "{}" if manual else '{"before":"before-push"}'
                )
            with patch.dict(os.environ, env), patch.object(meta, "git", git), \
                    patch.object(Path, "read_text", read_text), contextlib.redirect_stdout(io.StringIO()):
                meta.main()
            lines = output.read_text(encoding="utf-8").splitlines()
        result = {}
        while lines:
            key, delimiter = lines.pop(0).split("<<")
            end = lines.index(delimiter)
            result[key] = "\n".join(lines[:end])
            del lines[:end + 1]
        return result, calls

    def test_unchanged_and_existing_tag_skip(self):
        for kwargs in ({"new": "1.0.0"}, {"tags": ["sdk-v1.1.0"]}):
            result, calls = self.run_meta(**kwargs)
            self.assertEqual(result["changed"], "false")
            self.assertFalse(any(args[0] == "log" for args in calls))

    def test_changed_version_and_legacy_notes(self):
        result, calls = self.run_meta(reachable=["sdk-20260824-184539-c580918"])
        self.assertEqual(result["changed"], "true")
        self.assertIn("Example commit", result["body"])
        self.assertIn("sdk-20260824-184539-c580918...sdk-v1.1.0", result["body"])
        self.assertIn(("show", "before-push:.env"), calls)

    def test_first_version_prerelease_and_manual(self):
        result, calls = self.run_meta(old=None, new="1.1.0-rc.1", manual=True)
        self.assertEqual(result["changed"], "true")
        self.assertEqual(result["prerelease"], "true")
        self.assertIn(("rev-parse", "HEAD^"), calls)
        self.assertIn(("log", "--reverse", "--format=%H %s", "HEAD"), calls)


if __name__ == "__main__":
    unittest.main()
