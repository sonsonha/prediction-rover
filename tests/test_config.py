from prediction_core.config import load_config


def test_legacy_yaml_maps_single_footprint_to_body_and_support(tmp_path) -> None:
    path = tmp_path / "legacy.yaml"
    path.write_text(
        """
rover:
  length_m: 1.2
  width_m: 0.8
  cg_x_m: 0.0
  cg_y_m: 0.0
  cg_height_m: 0.45
prediction:
  collision_margin_m: 0.2
""",
        encoding="utf-8",
    )

    config = load_config(path)

    assert config.body_length_m == 1.2
    assert config.body_width_m == 0.8
    assert config.support_length_m == 1.2
    assert config.support_width_m == 0.8
    assert config.com_height_m == 0.45

