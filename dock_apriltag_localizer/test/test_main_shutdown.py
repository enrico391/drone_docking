"""Regression tests for clean localizer process shutdown."""

from dock_apriltag_localizer import dock_localizer


class FakeNode:
    def __init__(self):
        self.destroyed = False

    def destroy_node(self):
        self.destroyed = True


def test_main_does_not_shutdown_an_already_stopped_context(monkeypatch):
    """ROS launch may stop the context before spin returns on SIGINT."""
    node = FakeNode()
    shutdown_calls = []

    monkeypatch.setattr(dock_localizer, "DockLocalizer", lambda: node)
    monkeypatch.setattr(dock_localizer.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(dock_localizer.rclpy, "spin", lambda unused: None)
    monkeypatch.setattr(dock_localizer.rclpy, "ok", lambda: False)
    monkeypatch.setattr(
        dock_localizer.rclpy,
        "shutdown",
        lambda: shutdown_calls.append(True),
    )

    dock_localizer.main()

    assert node.destroyed
    assert shutdown_calls == []
