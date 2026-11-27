"""ContactSwitch without hardware: a fake gpiod request stands in for the header lines."""
from types import SimpleNamespace

import pytest

gpiod = pytest.importorskip('gpiod')
from gpiod.line import Value  # noqa: E402

import swarm_magnet.contact_switch as cs  # noqa: E402


class Request:
    def __init__(self): self.pressed, self.released = False, False
    def get_values(self, lines): return [Value.ACTIVE if self.pressed else Value.INACTIVE for _ in lines]
    def release(self): self.released = True


def switch(monkeypatch, chip='/dev/gpiochip0'):
    req, out, errors = Request(), [], []
    monkeypatch.setattr(cs, 'find_header_chip', lambda: chip)
    monkeypatch.setattr(cs.gpiod, 'request_lines', lambda *a, **k: req)
    logger = lambda: SimpleNamespace(info=lambda *a: None, error=errors.append)  # noqa: E731
    params = {'gpio_lines': [5, 6], 'rate_hz': 20.0, 'debounce_samples': 3}
    s = cs.ContactSwitch(logger, lambda n, d: params.get(n, d), SimpleNamespace(publish=lambda m: out.append(m.data)))
    return s, req, out, errors


def test_debounced_level_stream(monkeypatch):
    s, req, out, _ = switch(monkeypatch)
    assert s.period == pytest.approx(0.05)
    s.tick(); req.pressed = True
    for _ in range(4): s.tick()
    req.pressed = False; s.tick(); req.pressed = True; s.tick()          # a one-sample bounce changes nothing
    assert out == [False, False, False, True, True, True, True]         # True after 3 samples, published every tick
    s.release(); assert req.released


def test_no_header_chip_means_no_request(monkeypatch):
    s, _, out, errors = switch(monkeypatch, chip=None)
    assert s.request is None and errors and out == []
    s.release()                                                          # safe without a request
