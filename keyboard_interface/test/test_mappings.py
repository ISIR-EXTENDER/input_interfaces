from keyboard_interface.mappings import parse_key_topic_mappings
import pytest


def test_parse_key_topic_mappings_normalizes_keys_and_whitespace():
    mappings = parse_key_topic_mappings(
        [' Space = /keyboard/space ', 'A=/keyboard/a']
    )

    assert mappings == {
        'space': '/keyboard/space',
        'a': '/keyboard/a',
    }


@pytest.mark.parametrize(
    'raw_mapping, expected_message',
    [
        ('space', "Expected format 'key=/topic_name'"),
        ('=/keyboard/space', 'key cannot be empty'),
        ('space=', 'topic cannot be empty'),
    ],
)
def test_parse_key_topic_mappings_rejects_invalid_entries(
    raw_mapping, expected_message
):
    with pytest.raises(ValueError, match=expected_message):
        parse_key_topic_mappings([raw_mapping])


def test_parse_key_topic_mappings_rejects_empty_configuration():
    with pytest.raises(ValueError, match='At least one key/topic mapping'):
        parse_key_topic_mappings([])


def test_parse_key_topic_mappings_rejects_case_insensitive_duplicates():
    with pytest.raises(ValueError, match="Duplicate key mapping for 'space'"):
        parse_key_topic_mappings(
            ['space=/keyboard/space', 'SPACE=/keyboard/alternate_space']
        )
