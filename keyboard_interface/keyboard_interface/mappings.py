from __future__ import annotations

from collections.abc import Iterable


def parse_key_topic_mappings(raw_mappings: Iterable[str]) -> dict[str, str]:
    """Parse ``key=/topic`` entries into a case-insensitive key mapping."""
    mappings: dict[str, str] = {}

    for raw_mapping in raw_mappings:
        key_name, topic_name = split_mapping(raw_mapping)
        normalized_key = normalize_config_key(key_name)

        if normalized_key in mappings:
            raise ValueError(f"Duplicate key mapping for '{normalized_key}'.")

        mappings[normalized_key] = topic_name

    if not mappings:
        raise ValueError('At least one key/topic mapping must be configured.')

    return mappings


def split_mapping(raw_mapping: str) -> tuple[str, str]:
    """Split and validate one ``key=/topic`` mapping entry."""
    if '=' not in raw_mapping:
        raise ValueError(
            'Invalid key_topic_mappings entry '
            f"'{raw_mapping}'. Expected format 'key=/topic_name'."
        )

    key_name, topic_name = raw_mapping.split('=', 1)
    key_name = key_name.strip()
    topic_name = topic_name.strip()

    if not key_name:
        raise ValueError(f"Invalid mapping '{raw_mapping}': key cannot be empty.")
    if not topic_name:
        raise ValueError(f"Invalid mapping '{raw_mapping}': topic cannot be empty.")

    return key_name, topic_name


def normalize_config_key(key_name: str) -> str:
    """Normalize a configured key name for matching against pynput events."""
    return key_name.strip().lower()
