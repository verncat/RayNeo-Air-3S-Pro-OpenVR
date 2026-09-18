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


def header(version):
    return "\n".join(f"#define RAYNEO_API_VERSION_{name} {value}"
                     for name, value in zip(("MAJOR", "MINOR", "PATCH"), version.split(".")))


class ReleaseMetaTests(unittest.TestCase):
    def test_semver(self):
        for version in ("0.0.0", "1.2.3", "1.3.0", "65535.65535.42"):
            self.assertEqual(meta.version_from(header(version)), version)
        for version in ("v1.0.0", "1.0", "01.0.0", "1.0.0-01", "1.0.0+", "", "65536.0.0", "1.65536.0", "1.0.-1"):
            with self.assertRaises(ValueError):
                meta.version_from(header(version))
        with self.assertRaises(ValueError):
            meta.version_from(header("1.0.0") + '\n#define RAYNEO_API_VERSION_PATCH 1')

    def test_header_comments_and_invalid_definitions(self):
        self.assertIsNone(meta.version_from('#define RAYNEO_API_VERSION_MAJOR 1', allow_missing=True))
        text = ('/*\n#define RAYNEO_API_VERSION_MAJOR 9\n*/\n'
                '// #define RAYNEO_API_VERSION_PATCH 8\n' + header("1.2.3") + ' // release\n')
        self.assertEqual(meta.version_from(text), "1.2.3")
        for text in ('', header("1.0.OTHER"), header("1.0.0 extra"),
                     header("1.0.0").replace('PATCH 0', 'PATCH(x) 0')):
            with self.assertRaises(ValueError):
                meta.version_from(text)

    def run_meta(self, old="1.0.0", new="1.1.0", tags=(), reachable=(), manual=False):
        calls = []

        def git(*args):
            calls.append(args)
            if args[:2] == ("rev-list", "--count"):
                return "3"
            if args[0] == "rev-parse":
                return "parent"
            if args[0] == "ls-tree":
                return meta.VERSION_HEADER if old is not None else ""
            if args[0] == "show":
                if old == "legacy":
                    return '#define RAYNEO_API_VERSION_MAJOR 1'
                return header(old)
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
                return header(new) if path.as_posix() == meta.VERSION_HEADER else (
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
        self.assertIn(("show", f"before-push:{meta.VERSION_HEADER}"), calls)

    def test_first_version_and_manual(self):
        result, calls = self.run_meta(old=None, new="1.3.0", manual=True)
        self.assertEqual(result["changed"], "true")
        self.assertEqual(result["prerelease"], "false")
        self.assertIn(("rev-parse", "HEAD^"), calls)
        self.assertIn(("log", "--reverse", "--format=%H %s", "HEAD"), calls)

    def test_existing_header_without_release_version(self):
        result, calls = self.run_meta(old="legacy")
        self.assertEqual(result["changed"], "true")
        self.assertIn(("show", f"before-push:{meta.VERSION_HEADER}"), calls)

    def test_patch_only_change(self):
        result, _ = self.run_meta(old="1.3.0", new="1.3.1")
        self.assertEqual(result["changed"], "true")
        self.assertEqual(result["version"], "1.3.1")
        self.assertEqual(result["tag"], "sdk-v1.3.1")


if __name__ == "__main__":
    unittest.main()
