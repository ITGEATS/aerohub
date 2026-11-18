import pytest
from uav_spf_planner.prompt import VlmParseError, parse_point_response


def test_parse_point_response_ok():
    r = parse_point_response('[{"point": [450, 800], "depth": 5, "label": "ball"}]')
    assert r == {"point": [450, 800], "depth": 5, "label": "ball"}


def test_parse_point_response_depth_default():
    r = parse_point_response('[{"point": [1, 2], "label": "x"}]')
    assert r["depth"] == 4  # 上游 point_info.get("depth", 4)


def test_parse_point_response_markdown():
    r = parse_point_response('```json\n[{"point":[1,2],"depth":3,"label":"l"}]\n```')
    assert r["depth"] == 3


def test_parse_error_codes():
    for text, code in [("", "empty_response"), ("   ", "empty_response"),
                       ("not json", "bad_json"), ("{'point':[1,2]}", "bad_json"),
                       ("[]", "no_points")]:
        with pytest.raises(VlmParseError) as e:
            parse_point_response(text)
        assert e.value.code == code
