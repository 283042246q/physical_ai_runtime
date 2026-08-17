import importlib.util
from pathlib import Path
from types import SimpleNamespace

from launch.actions import EmitEvent
from launch.events import Shutdown
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_launch_module():
    path = PACKAGE_ROOT / "launch" / "record_episode.launch.py"
    spec = importlib.util.spec_from_file_location("record_episode_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_description_is_constructible():
    module = _load_launch_module()
    description = module.generate_launch_description()
    assert description.entities


def test_activation_failure_is_fail_stop():
    module = _load_launch_module()
    assert module._shutdown_on_activation_failure(
        SimpleNamespace(returncode=0), None
    ) == []

    actions = module._shutdown_on_activation_failure(
        SimpleNamespace(returncode=1), None
    )
    assert len(actions) == 1
    assert isinstance(actions[0], EmitEvent)
    assert isinstance(actions[0].event, Shutdown)


def test_bundled_profile_has_unique_stream_contract():
    path = PACKAGE_ROOT / "config" / "marvin_teleop_8cam_streams.yaml"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    streams = data["streams"]
    ids = [stream["id"] for stream in streams]
    topics = [stream["topic"] for stream in streams]

    assert streams
    assert len(ids) == len(set(ids))
    assert len(topics) == len(set(topics))
