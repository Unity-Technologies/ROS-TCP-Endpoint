import pytest
import sys

@pytest.fixture(scope='session', autouse=True)
def reverse_path():
    sys.path.reverse()
    print(sys.path)
