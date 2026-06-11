"""Deterministic natural-language command parser for the drawing task.

The drawing task command always has the same shape: a set of objects to be
*connected* and a set of objects to be *avoided*, e.g.::

    "connect the car and the plane, avoid the football"
    "spoji auto i avion, izbjegni nogometnu loptu"

This module turns such a sentence into::

    {"connect": ["car", "airplane"], "avoid": ["sports ball"]}

It is intentionally dependency-free (pure Python, no ROS, no LLM) so it can be
unit-tested on any machine and runs instantly during the time challenge.  Words
that are not in the synonym table are passed through unchanged, so the parser
also works with custom-trained YOLO classes.
"""

from __future__ import annotations

import re
from typing import Iterable


# Keywords that introduce the "connect these" part of the sentence.
CONNECT_KEYWORDS = (
    "connect", "join", "link", "draw between", "draw a line between",
    "spoji", "spojiti", "povezi", "poveži", "povezati", "nacrtaj",
)

# Keywords that introduce the "avoid these" part of the sentence.
AVOID_KEYWORDS = (
    "avoid", "avoiding", "without", "except", "but not", "do not cross",
    "don't cross", "dont cross", "not crossing", "skip",
    "izbjegni", "izbjegavaj", "izbjegavajuci", "izbjegavajući", "zaobidi",
    "zaobiđi", "zaobilazi", "bez",
)

# Tokens that separate objects within a clause ("a and b", "a, b", "a i b").
_SEPARATOR_RE = re.compile(r"\s*(?:,|;|\band\b|\bi\b|&|\+|\bte\b)\s*", re.IGNORECASE)

# Filler words removed from each object phrase before normalisation.
_STOPWORDS = {
    "the", "a", "an", "of", "object", "objekt", "objekata", "sliku", "slika",
    "picture", "pictogram", "piktogram", "image", "to", "with", "please",
    "robot", "marker", "pen", "line", "liniju", "crtu",
}

# Map common aliases / languages to canonical YOLO (COCO-style) class names.
# Unknown words pass through unchanged, so this only needs the frequent cases.
SYNONYMS = {
    # vehicles
    "plane": "airplane", "aeroplane": "airplane", "avion": "airplane",
    "zrakoplov": "airplane", "zrakoplova": "airplane", "airplane": "airplane",
    "car": "car", "auto": "car", "automobil": "car", "automobila": "car",
    "vehicle": "car", "voziilo": "car", "vozilo": "car",
    "truck": "truck", "kamion": "truck",
    "bus": "bus", "autobus": "bus",
    "motorcycle": "motorcycle", "motor": "motorcycle", "motocikl": "motorcycle",
    "bicycle": "bicycle", "bike": "bicycle", "bicikl": "bicycle",
    "train": "train", "vlak": "train",
    "boat": "boat", "brod": "boat", "camion": "truck",
    # balls / sport
    "football": "sports ball", "soccer": "sports ball",
    "soccer ball": "sports ball", "ball": "sports ball",
    "lopta": "sports ball", "nogomet": "sports ball",
    "nogometna lopta": "sports ball", "nogometnu loptu": "sports ball",
    "sports ball": "sports ball",
    # signs / street
    "stop": "stop sign", "stop sign": "stop sign", "stopsign": "stop sign",
    "znak": "stop sign", "stop znak": "stop sign",
    "traffic light": "traffic light", "trafficlight": "traffic light",
    "semafor": "traffic light", "semafora": "traffic light",
    "semafore": "traffic light", "semaforima": "traffic light",
    "lights": "traffic light", "light": "traffic light",
    "fire hydrant": "fire hydrant", "hidrant": "fire hydrant",
    # animals
    "cat": "cat", "macka": "cat", "mačka": "cat", "macku": "cat", "mačku": "cat",
    "macke": "cat", "mačke": "cat", "mace": "cat",
    "dog": "dog", "pas": "dog", "psa": "dog", "psi": "dog", "kucni ljubimac": "dog",
    "auti": "car", "automobili": "car", "avioni": "airplane",
    "semafori": "traffic light", "lopte": "sports ball", "boce": "bottle",
    "bird": "bird", "ptica": "bird", "horse": "horse", "konj": "horse",
    "cow": "cow", "krava": "cow", "sheep": "sheep", "ovca": "sheep",
    "bear": "bear", "medvjed": "bear", "elephant": "elephant", "slon": "elephant",
    # household
    "clock": "clock", "sat": "clock", "sata": "clock",
    "bottle": "bottle", "boca": "bottle", "bocu": "bottle", "flasa": "bottle",
    "cup": "cup", "salica": "cup", "šalica": "cup", "kup": "cup",
    "chair": "chair", "stolica": "chair", "scissors": "scissors", "skare": "scissors",
    "book": "book", "knjiga": "book", "laptop": "laptop", "phone": "cell phone",
    "cell phone": "cell phone", "mobitel": "cell phone", "telefon": "cell phone",
    "tv": "tv", "televizor": "tv", "umbrella": "umbrella", "kisobran": "umbrella",
    "apple": "apple", "jabuka": "apple", "banana": "banana",
}


def _normalise_phrase(phrase: str) -> str:
    """Lower-case, strip stopwords/punctuation and map a phrase to a class name."""
    phrase = phrase.strip().lower()
    phrase = re.sub(r"[^a-z0-9čćžšđ\s]", " ", phrase)
    words = [w for w in phrase.split() if w and w not in _STOPWORDS]
    if not words:
        return ""

    cleaned = " ".join(words)

    # Try the longest match first: full phrase, then trailing/leading bigram,
    # then individual words.
    candidates = [cleaned]
    if len(words) >= 2:
        candidates.append(" ".join(words[-2:]))
        candidates.append(" ".join(words[:2]))
    candidates.extend(reversed(words))  # prefer the last (head) noun

    # Add singular variants (drop a trailing plural 's') for each candidate.
    expanded: list[str] = []
    for candidate in candidates:
        expanded.append(candidate)
        if len(candidate) > 3 and candidate.endswith("s") and not candidate.endswith("ss"):
            expanded.append(candidate[:-1])

    for candidate in expanded:
        if candidate in SYNONYMS:
            return SYNONYMS[candidate]

    # No synonym: return the cleaned phrase, de-pluralised if it is one word.
    if len(words) == 1 and len(cleaned) > 3 and cleaned.endswith("s") \
            and not cleaned.endswith("ss"):
        return cleaned[:-1]
    return cleaned


def _split_objects(clause: str) -> list[str]:
    """Split a clause like 'car and the plane, bottle' into normalised classes."""
    objects: list[str] = []
    for raw in _SEPARATOR_RE.split(clause):
        name = _normalise_phrase(raw)
        if name and name not in objects:
            objects.append(name)
    return objects


def _find_first_keyword(text: str, keywords: Iterable[str]) -> tuple[int, int]:
    """Return (start, end) index of the earliest keyword occurrence, or (-1, -1)."""
    best = (-1, -1)
    for kw in keywords:
        match = re.search(r"\b" + re.escape(kw) + r"\b", text, re.IGNORECASE)
        if match and (best[0] == -1 or match.start() < best[0]):
            best = (match.start(), match.end())
    return best


def parse_command(text: str) -> dict:
    """Parse a drawing command into connect/avoid object lists.

    Args:
        text: Free-form instruction, English or Croatian.

    Returns:
        Dict with keys:
            connect: ordered list of canonical class names to connect.
            avoid:   list of canonical class names to avoid.
            ok:      True if at least two connect targets were found.
            raw:     the original command string.
            error:   human-readable reason when ``ok`` is False.
    """
    result = {"connect": [], "avoid": [], "ok": False, "raw": text, "error": ""}
    if not text or not text.strip():
        result["error"] = "Empty command."
        return result

    cleaned = text.strip()

    avoid_start, _ = _find_first_keyword(cleaned, AVOID_KEYWORDS)
    if avoid_start >= 0:
        connect_part = cleaned[:avoid_start]
        avoid_part = cleaned[avoid_start:]
    else:
        connect_part = cleaned
        avoid_part = ""

    # Drop the leading connect keyword from the connect clause if present.
    conn_start, conn_end = _find_first_keyword(connect_part, CONNECT_KEYWORDS)
    if conn_start >= 0:
        connect_part = connect_part[conn_end:]

    # Drop the leading avoid keyword from the avoid clause if present.
    if avoid_part:
        _, av_end = _find_first_keyword(avoid_part, AVOID_KEYWORDS)
        if av_end >= 0:
            avoid_part = avoid_part[av_end:]

    result["connect"] = _split_objects(connect_part)
    result["avoid"] = _split_objects(avoid_part)

    if len(result["connect"]) < 2:
        result["error"] = (
            "Need at least two objects to connect; "
            f"found {result['connect']}."
        )
        return result

    result["ok"] = True
    return result


if __name__ == "__main__":  # pragma: no cover - manual smoke test
    import json
    import sys

    command = " ".join(sys.argv[1:]) or "connect the car and the plane, avoid football"
    print(json.dumps(parse_command(command), ensure_ascii=False, indent=2))
