"""Package style checks run by colcon test."""

from ament_flake8.main import main_with_errors
from ament_pep257.main import main as pep257_main


def test_flake8() -> None:
    """Require PEP 8 compliance."""
    return_code, errors = main_with_errors(argv=[])
    assert return_code == 0, errors


def test_pep257() -> None:
    """Require public Python objects to document their contract."""
    assert pep257_main(argv=['.', 'test']) == 0
