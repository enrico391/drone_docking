"""Flight-action transport tests with fake services, never vehicle commands."""

from concurrent.futures import Future, ThreadPoolExecutor
import json
import time
from types import SimpleNamespace

import pytest

from flight_cycle_manager.pid_tuner import FlightActionBridge, TelemetryStore, parse_action_request


@pytest.mark.parametrize("payload", [
    None, [], {}, {"action": "start_cycle"},
    {"action": "start_cycle", "confirmed": False},
    {"action": "start_cycle", "confirmed": 1},
    {"action": "arm", "confirmed": True},
    {"action": [], "confirmed": True},
    {"action": "request_dock", "confirmed": True, "service": "/other"},
])
def test_actions_require_explicit_confirmation_and_allowlisted_name(payload):
    with pytest.raises(ValueError):
        parse_action_request(payload)


class FakeClient:
    def __init__(self):
        self.ready = True
        self.calls = []
        self.future = Future()

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.calls.append(request)
        return self.future


@pytest.fixture
def bridge():
    clock = [10.0]
    store = TelemetryStore(now=lambda: clock[0])
    store.update(json.dumps({"flight_actions": {
        action: {"allowed": True, "reason": ""} for action in ("start_cycle", "request_dock")}}))
    clients = {action: FakeClient() for action in ("start_cycle", "request_dock")}
    return FlightActionBridge(store, clients, lambda: "trigger request", now=lambda: clock[0]), store, clients, clock


def wait_for_work(bridge):
    deadline = time.monotonic() + 1
    while bridge._work is None:
        assert time.monotonic() < deadline
        time.sleep(.001)


@pytest.mark.parametrize("action", ["start_cycle", "request_dock"])
@pytest.mark.parametrize("accepted", [True, False])
def test_bridge_dispatches_once_and_returns_actual_service_message(bridge, action, accepted):
    transport, _, clients, _ = bridge
    with ThreadPoolExecutor(max_workers=1) as workers:
        result = workers.submit(transport.submit, action)
        wait_for_work(transport)
        assert not clients[action].calls  # HTTP thread never executes ROS calls.
        assert not transport.submit(action)["ok"]
        transport.process()
        transport.process()
        assert clients[action].calls == ["trigger request"]
        clients[action].future.set_result(SimpleNamespace(success=accepted, message="manager reply"))
        transport.process()
        assert result.result(timeout=1) == {"ok": accepted, "message": "manager reply"}
        assert transport._work is None
    assert not clients["request_dock" if action == "start_cycle" else "start_cycle"].calls


def test_stale_or_ineligible_requests_never_reach_ros(bridge):
    transport, store, clients, clock = bridge
    clock[0] += 2.1
    assert not transport.submit("start_cycle")["ok"]
    store.update('{"flight_actions":{"start_cycle":{"allowed":false,"reason":"not landed"}}}')
    assert transport.submit("start_cycle")["message"] == "not landed"
    assert not transport.submit("request_dock")["ok"]
    assert not transport.submit("arm")["ok"]
    assert all(not client.calls for client in clients.values())


@pytest.mark.parametrize("condition", ["stale", "blocked", "service_unavailable", "expired"])
def test_bridge_rechecks_prerequisites_before_dispatch(bridge, condition):
    transport, store, clients, clock = bridge
    with ThreadPoolExecutor(max_workers=1) as workers:
        result = workers.submit(transport.submit, "start_cycle")
        wait_for_work(transport)
        if condition == "stale":
            clock[0] += 2.1
        elif condition == "expired":
            clock[0] += 3.1
        elif condition == "blocked":
            store.update('{"flight_actions":{"start_cycle":{"allowed":false,"reason":"busy"}}}')
        else:
            clients["start_cycle"].ready = False
        transport.process()
        assert not result.result(timeout=1)["ok"]
    assert not clients["start_cycle"].calls


@pytest.mark.parametrize("failure", ["timeout", "exception"])
def test_uncertain_outcome_is_not_retried(bridge, failure):
    transport, _, clients, clock = bridge
    with ThreadPoolExecutor(max_workers=1) as workers:
        result = workers.submit(transport.submit, "start_cycle")
        wait_for_work(transport)
        transport.process()
        if failure == "timeout":
            clock[0] += 3.1
        else:
            clients["start_cycle"].future.set_exception(RuntimeError("transport failed"))
        transport.process()
        outcome = result.result(timeout=1)
        assert not outcome["ok"] and outcome["uncertain"]
        assert "Do not retry" in outcome["message"]
        transport.process()
    assert len(clients["start_cycle"].calls) == 1