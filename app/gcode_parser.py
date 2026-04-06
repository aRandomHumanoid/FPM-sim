from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional


@dataclass(slots=True)
class ParsedCommand:
    letter: str
    code: float
    params: Dict[str, float]
    raw: str

    @property
    def normalized(self) -> str:
        if self.letter == "G" and self.code.is_integer():
            return f"G{int(self.code)}"
        if self.letter == "M" and self.code.is_integer():
            return f"M{int(self.code)}"
        return f"{self.letter}{self.code}"


class GcodeParser:
    def parse(self, line: str) -> Optional[ParsedCommand]:
        cleaned = self._strip_comments(line).strip()
        if "*" in cleaned:
            cleaned = cleaned.split("*", 1)[0].strip()
        if not cleaned:
            return None

        tokens = [t for t in cleaned.split() if t]
        if not tokens:
            return None

        if tokens[0].upper().startswith("N") and tokens[0][1:].isdigit():
            tokens = tokens[1:]
            if not tokens:
                return None

        cmd_index = None
        for idx, tok in enumerate(tokens):
            up = tok.upper()
            if len(up) >= 2 and up[0] in {"G", "M"}:
                cmd_index = idx
                break

        if cmd_index is None:
            raise ValueError(f"Missing G/M command: {line}")

        command_token = tokens[cmd_index].upper()
        if len(command_token) < 2:
            raise ValueError(f"Malformed command: {line}")

        letter = command_token[0]
        if letter not in {"G", "M"}:
            raise ValueError(f"Unsupported command family: {command_token}")

        try:
            code = float(command_token[1:])
        except ValueError as exc:
            raise ValueError(f"Invalid command number: {command_token}") from exc

        params: Dict[str, float] = {}
        for idx, tok in enumerate(tokens):
            if idx == cmd_index:
                continue
            up = tok.upper()
            if len(up) < 2:
                continue
            k = up[0]
            if not k.isalpha():
                continue
            try:
                params[k] = float(up[1:])
            except ValueError as exc:
                raise ValueError(f"Invalid parameter token: {tok}") from exc

        return ParsedCommand(letter=letter, code=code, params=params, raw=cleaned)

    @staticmethod
    def _strip_comments(line: str) -> str:
        semicolon = line.split(";", 1)[0]

        out_chars: list[str] = []
        depth = 0
        for ch in semicolon:
            if ch == "(":
                depth += 1
                continue
            if ch == ")" and depth > 0:
                depth -= 1
                continue
            if depth == 0:
                out_chars.append(ch)
        return "".join(out_chars)
