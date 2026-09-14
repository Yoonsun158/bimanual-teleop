"""Read JSON configuration with // line comments and /* block comments */."""

import json
from pathlib import Path
import re


_TOKENS = re.compile(r'"(?:[^"\\]|\\.)*"|//[^\r\n]*|/\*[\s\S]*?\*/')


def load_json_config(path: str | Path):
    text = Path(path).read_text(encoding="utf-8")

    def strip_comment(match):
        token = match.group()
        # Preserve strings, including URLs, and error line/column positions.
        return token if token.startswith('"') else re.sub(r"[^\r\n]", " ", token)

    return json.loads(_TOKENS.sub(strip_comment, text))
